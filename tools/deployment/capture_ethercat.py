"""Passive EtherCAT flight recorder for the 64-bit Linux Orin. No ROS dependencies."""
import argparse
import csv
import ctypes
import fcntl
import json
import os
import re
import select
import signal
import socket
import struct
import sys
import threading
import time
from collections import deque
from contextlib import ExitStack, contextmanager
from pathlib import Path


ETH_P_ALL = 3
ETH_P_ECAT = 0x88A4
SOL_PACKET = 263
SO_TIMESTAMPING = 37  # Linux 64-bit SCM_TIMESTAMPING, three native timespecs
FRAME_LIMIT = 8192
FRAME_FIELDS = ('observed_mono_ns', 'direction', 'command', 'index', 'address',
                'data_len', 'first_wkc', 'software_realtime_ns', 'hardware_phc_ns')
BAD_CYCLE = re.compile(rb'\[ECAT-FAULT-CYCLE\].*?sequence=(\d+) first_bad=1\b.*?'
                       rb'send_ns=(\d+)')


def frame_record(frame, direction, ancillary, observed_ns):
    if len(frame) < 28 or int.from_bytes(frame[12:14], 'big') != ETH_P_ECAT:
        return None
    length = int.from_bytes(frame[22:24], 'little') & 0x7ff
    end = 26 + length
    if end + 2 > len(frame):
        return None
    software = hardware = None
    for level, kind, value in ancillary:
        if (level, kind) == (socket.SOL_SOCKET, SO_TIMESTAMPING):
            sw_s, sw_ns, _, _, hw_s, hw_ns = struct.unpack('=6q', value[:48])
            software = (sw_s * 10**9 + sw_ns) or None
            hardware = (hw_s * 10**9 + hw_ns) or None
    return (observed_ns, direction, frame[16], frame[17], int.from_bytes(frame[18:22], 'little'),
            length, int.from_bytes(frame[end:end+2], 'little'), software, hardware)


def filter_ethercat(sock):
    # Classic BPF: load Ethernet type, accept only 0x88a4. No promiscuous mode.
    program = ctypes.create_string_buffer(b''.join(struct.pack('=HBBI', *op) for op in (
        (0x28, 0, 0, 12), (0x15, 0, 1, ETH_P_ECAT), (0x06, 0, 0, 65535), (0x06, 0, 0, 0))))
    sock.setsockopt(socket.SOL_SOCKET, 26, struct.pack('@HP', 4, ctypes.addressof(program)))


@contextmanager
def rx_hardware_filter(sock, interface, enable):
    if not enable:
        yield
        return
    # SIOCG/SIOCSHWTSTAMP, struct ifreq with pointer to struct hwtstamp_config.
    config = (ctypes.c_int * 3)()
    request = struct.pack('@16sP16x', interface.encode(), ctypes.addressof(config))
    fcntl.ioctl(sock, 0x89b1, request)
    previous = tuple(config)
    changed = config[2] != 1  # HWTSTAMP_FILTER_ALL
    if changed:
        config[2] = 1
        fcntl.ioctl(sock, 0x89b0, request)
    try:
        if config[2] != 1:
            raise RuntimeError('driver did not select HWTSTAMP_FILTER_ALL')
        yield
    finally:
        if changed:
            config[:] = previous
            fcntl.ioctl(sock, 0x89b0, request)


def clock_pair(clock_id):
    before = time.monotonic_ns()
    source = time.clock_gettime_ns(clock_id)
    return before, source, time.monotonic_ns()


def new_fault(log):
    # Leave a partially written line for the next poll.
    while True:
        pos = log.tell()
        line = log.readline()
        if not line or not line.endswith(b'\n'):
            log.seek(pos)
            return None
        match = BAD_CYCLE.search(line)
        if match:
            sequence, send_ns = map(int, match.groups())
            fault = dict(sequence=sequence, send_ns=send_ns)
            for field in ('receive_ns', 'recv_begin_ns', 'recv_end_ns'):
                value = re.search(rb'\b'+field.encode()+rb'=(\d+)', line)
                if value:
                    fault[field] = int(value[1])
            wkc = re.search(rb'\bWKC=(-?\d+)/(\d+)\b', line)
            if wkc:
                fault['wkc'], fault['expected_wkc'] = map(int, wkc.groups())
            return fault


def fault_frames(frames, metadata):
    """Align on the fault window's bracketing clock samples; retain packet identities."""
    fault = metadata['fault']
    if not fault:
        return None
    if 'recv_begin_ns' not in fault or 'recv_end_ns' not in fault:
        return {'status': 'missing_receive_call_timestamps'}
    send, end = fault['send_ns'], fault['recv_end_ns']
    clocks = metadata['clocks']
    before = [pair for pair in clocks if pair[0][2] <= send]
    after = [pair for pair in clocks if pair[0][0] >= end]
    if not before or not after:
        return {'status': 'missing_bracketing_clock_samples'}
    # Offset envelopes include both calibration read intervals. These are estimates
    # under continuous clocks, not a bound across a PHC/realtime clock step.
    def aligned(stamp, source):
        samples = [before[-1][source], after[0][source]]
        if stamp is None or None in samples:
            return None
        return [stamp + min(a-b for a,b,c in samples), stamp + max(c-b for a,b,c in samples)]

    transmit = []
    for i, row in enumerate(frames):
        if row[1] != 'TX':
            continue
        bounds = aligned(row[7], 0)
        if bounds and bounds[0] <= fault['recv_begin_ns'] and bounds[1] >= send:
            transmit.append((i, row, bounds))
    if len(transmit) != 1:
        return {'status': 'ambiguous_tx', 'candidate_count': len(transmit)}
    i, tx, tx_bounds = transmit[0]
    if tx_bounds[0] < send or tx_bounds[1] > fault['recv_begin_ns']:
        return {'status': 'uncertain_tx_clock_alignment', 'tx_mono_estimate_ns': tx_bounds}
    returns = []
    # Stop at the next TX reusing this index, even if its command differs.
    for j in range(i+1, len(frames)):
        row = frames[j]
        if row[1] == 'TX' and row[3] == tx[3]:
            break
        if row[1] == 'RX' and row[2:6] == tx[2:6]:
            returns.append(dict(record=j, first_wkc=row[6], hw_mono_estimate_ns=aligned(row[8], 1),
                                sw_mono_estimate_ns=aligned(row[7], 0)))
    return dict(status='tx_identified', tx_record=i, index=tx[3], tx_mono_estimate_ns=tx_bounds,
                matching_rx=returns, nominal_deadline_ns=fault['recv_begin_ns']+1_000_000)


def collect(sock, log, phc_clock, stopped):
    frames = deque(maxlen=FRAME_LIMIT)
    clocks = deque(maxlen=128)
    packets, invalid, truncated = 0, 0, 0
    reported_rx = set()
    fault = stop_at = None
    next_poll = next_clock = 0
    while not stopped():
        now = time.monotonic_ns()
        if now >= next_clock:
            clocks.append((clock_pair(time.CLOCK_REALTIME), clock_pair(phc_clock) if phc_clock is not None else None))
            next_clock = now + 250_000_000
        if not fault and now >= next_poll:
            fault = new_fault(log)
            if fault:
                stop_at = now + 100_000_000
            next_poll = now + 20_000_000
        if stop_at and now >= stop_at:
            break
        if not select.select([sock], [], [], .02)[0]:
            continue
        frame, ancillary, flags, addr = sock.recvmsg(4096, socket.CMSG_SPACE(48))
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            truncated += 1
            continue
        direction = 'TX' if addr[2] == 4 else 'RX'  # PACKET_OUTGOING
        row = frame_record(frame, direction, ancillary, time.monotonic_ns())
        if row is None:
            invalid += 1
            continue
        frames.append(row)
        packets += 1
        if direction == 'RX' and bool(row[-1]) not in reported_rx:
            reported_rx.add(bool(row[-1]))
            print('RX hardware timestamp:', 'present' if row[-1] else 'missing', flush=True)
    clocks.append((clock_pair(time.CLOCK_REALTIME), clock_pair(phc_clock) if phc_clock is not None else None))
    # PACKET_STATISTICS resets on read. Query once, covering the whole run.
    received, drops = struct.unpack('=II', sock.getsockopt(SOL_PACKET, 6, 8))
    return frames, dict(fault=fault, stop='fault' if fault else 'manual', clocks=list(clocks),
                        clock_fields=['monotonic_before_ns', 'source_ns', 'monotonic_after_ns'],
                        clock_sources=['CLOCK_REALTIME', 'interface_PHC'], kernel_packets=received,
                        kernel_socket_drops=drops, invalid_frames=invalid, truncated_messages=truncated,
                        ring_overwrites=packets-len(frames), retained_frames=len(frames),
                        hardware_stamped_rx=sum(r[1]=='RX' and r[-1] is not None for r in frames))


def interface_counters(netdev):
    # Interface-wide counters sampled outside the control process.
    counters = {}
    started = time.monotonic_ns()
    for name in ('rx_errors', 'rx_crc_errors', 'rx_missed_errors', 'rx_dropped', 'carrier_changes'):
        path = netdev/name if name == 'carrier_changes' else netdev/'statistics'/name
        try:
            counters[name] = int(path.read_text())
        except (OSError, ValueError):
            counters[name] = None
    return dict(read_begin_ns=started, read_end_ns=time.monotonic_ns(), counters=counters)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--servo-log', type=Path, required=True, help='current run, followed from EOF')
    parser.add_argument('--output', type=Path, required=True, help='new directory for this window')
    parser.add_argument('--enable-hardware-rx', action='store_true',
                        help='temporarily request all-frame RX hardware timestamps and restore the prior filter')
    args = parser.parse_args(argv)
    if sys.platform != 'linux' or struct.calcsize('P') != 8:
        parser.error('capture requires 64-bit Linux')
    socket.if_nametoindex(args.interface)
    with ExitStack() as stack:
        log = stack.enter_context(args.servo_log.open('rb'))
        stopped = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, lambda *_: stopped.set())
            stack.callback(signal.signal, sig, previous)
        phc = next((Path('/sys/class/net')/args.interface/'device/ptp').glob('ptp[0-9]*'), None)
        phc_clock = None
        if phc:
            fd = os.open('/dev/'+phc.name, os.O_RDONLY)
            stack.callback(os.close, fd)
            phc_clock = ((~fd) << 3) | 3  # Linux FD_TO_CLOCKID
        if args.enable_hardware_rx and phc_clock is None:
            parser.error('no interface PHC found for alignment with PDO CLOCK_MONOTONIC')
        # Protocol zero keeps the observer inactive until the filtered socket is bound.
        sock = stack.enter_context(socket.socket(socket.AF_PACKET, socket.SOCK_RAW, 0))
        filter_ethercat(sock)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
        sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPING, (1<<2)|(1<<3)|(1<<4)|(1<<6))
        if args.output.exists():
            parser.error('output directory already exists')
        stack.enter_context(rx_hardware_filter(sock, args.interface, args.enable_hardware_rx))
        netdev = Path('/sys/class/net')/args.interface
        counters_before = interface_counters(netdev)
        log.seek(0, os.SEEK_END)
        sock.bind((args.interface, ETH_P_ALL))
        args.output.mkdir(parents=True, exist_ok=False)
        print('ECAT_CAPTURE_READY', args.interface, str(args.servo_log), flush=True)
        frames, metadata = collect(sock, log, phc_clock, stopped.is_set)
        metadata['interface_counters'] = dict(start=counters_before, end=interface_counters(netdev))
    with (args.output/'frames.csv').open('x', newline='') as out:
        writer = csv.writer(out)
        writer.writerow(FRAME_FIELDS)
        writer.writerows(frames)
    metadata['fault_frames'] = fault_frames(list(frames), metadata)
    metadata.update(interface=args.interface, servo_log=str(args.servo_log.resolve()),
                    phc=phc.name if phc else None, frame_limit=FRAME_LIMIT)
    (args.output/'window.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('ECAT_CAPTURE_SAVED', args.output, flush=True)


if __name__ == '__main__':
    main()

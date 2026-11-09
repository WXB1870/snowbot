"""Passive Orin Fast DDS history and scheduler capture for the real adapter."""
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import select
import socket
import struct
import sys
import tempfile
import time

# ARM64 Fast DDS 2.6.12, Ubuntu Humble build on the field machine.
DDS = Path('/opt/ros/humble/lib/libfastrtps.so.2.6.12')
DDS_SHA = 'afe6588b08febeffce4c1fc6ad705726ef6c87a02c1adc9bdac22b9065c70258'
PRIVATE_DDS = Path('lib/snowbot_hardware/pdo-trace') / DDS.name
ADDRESS = b'\x00snowbot-orin-pdo-reader-v1'
EVENT = re.compile(r'\s(\d+)\.(\d+):\s+(?:\w+:)?(sched_switch|sched_wakeup|pdo_enqueue):\s+(.*)')
PRE_NS, POST_NS = 500_000_000, 100_000_000


def pinned(path):
    return path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == DDS_SHA


def adapter_trace_env():
    from ament_index_python.packages import get_package_prefix
    private = Path(get_package_prefix('snowbot_hardware')) / PRIVATE_DDS
    if not private.is_file() or not pinned(DDS) or not pinned(private):
        return {}
    preload = ' '.join(filter(None, (str(private), os.getenv('LD_PRELOAD'))))
    return {'LD_PRELOAD': preload, 'SNOWBOT_PDO_TRACE': '1'}


def notify(payload, logger):
    # Only on registration and completed fault windows; never wait for the collector.
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
            sender.setblocking(False)
            sender.sendto(payload.encode(), ADDRESS)
    except OSError as exc:
        logger.warn(f'PDO_TRACE_UNAVAILABLE: {exc}', throttle_duration_sec=5.)


def summarize(lines, pid, window):
    trigger = window['trigger_steady_ns']
    anchors = {(r['boot_id'], r['pdo_sample'], r['header_ros_ns'])
               for r in window['before'] + window['after']}
    queues, sched = [], []
    matched = set()
    dropped_sched = 0
    sched_peer = re.compile(r'\b(?:prev_pid|next_pid|pid)='+str(pid)+r'\b')
    for line in lines.splitlines():
        m = EVENT.search(line)
        if not m:
            continue
        ns = int(m[1])*10**9 + int(m[2].ljust(9, '0'))
        if not trigger-PRE_NS <= ns <= trigger+POST_NS:
            continue
        if m[3] != 'pdo_enqueue':
            if sched_peer.search(m[4]):
                if len(sched) < 8192:
                    sched.append(line)
                else:
                    dropped_sched += 1
            continue
        fields = dict(re.findall(r'(\w+)=(0x[0-9a-fA-F]+|\d+)', m[4]))
        if not all(k in fields for k in ('history','boot','sample','sec','nsec')):
            continue
        row = {k:int(v, 0) for k,v in fields.items()}
        header = row['sec']*10**9 + row['nsec']
        if (row['boot'], row['sample'], header) in anchors:
            matched.add(row['history'])
        queues.append((row['history'], dict(boot_id=row['boot'], pdo_sample=row['sample'],
                      header_ros_ns=header, enqueue_steady_ns=ns)))
    records = [r for history,r in queues if history in matched] if len(matched)==1 else []
    return dict(reader_identity='matched' if len(matched)==1 else 'ambiguous' if matched else 'unmatched',
                reader_history_ids=sorted(matched), enqueue=records, scheduler=sched, scheduler_rows_omitted=dropped_sched)


def collect(parent_pid, private, output):
    if not pinned(private) or not pinned(DDS) or private.samefile(DDS):
        raise ValueError('Private Fast DDS binary does not match the Orin probe')
    tracefs = next(p for p in (Path('/sys/kernel/tracing'), Path('/sys/kernel/debug/tracing'))
                   if (p/'uprobe_events').is_file())
    group = 'snowbot_pdo_'+str(os.getpid())
    instance = tracefs/'instances'/group
    def write(name, value):
        (instance/name).write_text(str(value))
    def probe(command):
        with (tracefs/'uprobe_events').open('a') as stream:
            stream.write(command+'\n')

    # 0x228f84 is immediately after the successful vector insertion in
    # ReaderHistory::add_change and before unlock. x19=this, stack[1]=CacheChange*.
    # SerializedPayload.data is at +56. Empty frame_id, little-endian XCDR1:
    # Header + wrench[12] + sensor mask/DataNo/ages precede boot_id and pdo_sample.
    definition = (f'p:{group}/pdo_enqueue {private}:0x228f84 history=%x19:x64 '
                  'length=+52($stack1):u32 cdr=+0(+56($stack1)):u16 '
                  'frame_len=+12(+56($stack1)):u32 sec=+4(+56($stack1)):u32 '
                  'nsec=+8(+56($stack1)):u32 boot=+92(+56($stack1)):u64 '
                  'sample=+100(+56($stack1)):u64')
    with ExitStack() as stack:
        sock = stack.enter_context(socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
        sock.bind(ADDRESS)
        parent = os.pidfd_open(parent_pid)
        stack.callback(os.close, parent)
        instance.mkdir()
        stack.callback(instance.rmdir)
        write('tracing_on', 0)
        write('trace_clock', 'mono')
        write('buffer_size_kb', 256)
        write('options/overwrite', 1)
        if (instance/'options/nsecs').exists():
            write('options/nsecs', 1)
        probe(definition)
        stack.callback(probe, '-:'+group+'/pdo_enqueue')
        filters = {'sched/sched_switch':'prev_comm == "snowbot_adapter" || next_comm == "snowbot_adapter"',
                   'sched/sched_wakeup':'comm == "snowbot_adapter"',
                   group+'/pdo_enqueue':'length >= 1024 && cdr == 256 && frame_len == 1'}
        for event, expression in filters.items():
            path = 'events/'+event
            write(path+'/filter', expression)
            write(path+'/enable', 1)
            stack.callback(write, path+'/enable', 0)
        write('tracing_on', 1)
        stack.callback(write, 'tracing_on', 0)
        armed = time.monotonic_ns()
        slot = 0
        print('PDO_READER_TRACE_READY '+str(output), flush=True)
        while True:
            ready, _, _ = select.select([sock, parent], [], [])
            if parent in ready:
                return
            data, ancillary, flags, _ = sock.recvmsg(32768, socket.CMSG_SPACE(12))
            credentials = next((v for level,k,v in ancillary if level==socket.SOL_SOCKET and k==socket.SCM_CREDENTIALS), None)
            if flags & socket.MSG_TRUNC or credentials is None:
                continue
            pid, uid, _ = struct.unpack('3i', credentials)
            if uid != int(os.environ['SUDO_UID']):
                continue
            try:
                maps = Path(f'/proc/{pid}/maps').read_text()
            except (FileNotFoundError, ProcessLookupError):
                continue
            if str(private)+'\n' not in maps:
                continue
            if data == b'armed':
                write('events/sched/sched_switch/filter', f'prev_pid == {pid} || next_pid == {pid}')
                write('events/sched/sched_wakeup/filter', f'pid == {pid}')
                continue
            window = json.loads(data)
            trigger = window['trigger_steady_ns']
            time.sleep(max(0, min(.1, (trigger+POST_NS-time.monotonic_ns())/1e9)))
            write('tracing_on', 0)
            frozen = time.monotonic_ns()
            try:
                trace = (instance/'trace').read_text()
                stats = {p.parent.name:p.read_text() for p in (instance/'per_cpu').glob('cpu*/stats')}
            finally:
                write('tracing_on', 1)
            result = summarize(trace, pid, window)
            result.update(adapter_pid=pid, window=window, clock='mono', armed_ns=armed,
                          frozen_ns=frozen, pre_ns=PRE_NS, post_ns=POST_NS, per_cpu_stats=stats,
                          dds_sha256=DDS_SHA)
            # All output file operations use the operator's privileges.
            os.seteuid(uid)
            try:
                with tempfile.NamedTemporaryFile(mode='w', dir=output, delete=False) as tmp:
                    json.dump(result, tmp, separators=(',', ':'))
                dest = output/f'pdo-reader-{slot%4}.json'
                os.replace(tmp.name, dest)
            finally:
                os.seteuid(0)
            slot += 1
            print(f'PDO_READER_TRACE_SAVED {dest} reader={result["reader_identity"]} '
                  f'enqueue={len(result["enqueue"])} sched={len(result["scheduler"])}', flush=True)


if __name__ == '__main__':
    import signal
    # Unwind tracefs registrations on terminal shutdown as well as parent exit.
    def stop(*_):
        raise SystemExit(0)
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, stop)
    collect(int(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))

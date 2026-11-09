"""Isolated Orin/Humble reader experiment; never creates a hardware backend or command topic."""
import ctypes as ct
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from snowbot_ctl_msgs.msg import PositionObservation
from snowbot_hardware.pdo_executor import HumbleObservationExecutor
from snowbot_hardware.session import SessionHealth

ns = time.monotonic_ns
TOPIC = '/snowbot_bench/reader_window/observation'
BOOT = 0x3456789ABC
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def publish(node):
    pub = node.create_publisher(PositionObservation, TOPIC, QOS)
    deadline = time.monotonic() + 8
    while pub.get_subscription_count() != 2:
        if time.monotonic() > deadline:
            raise TimeoutError('expected exactly two test readers')
        time.sleep(.01)
    sent = []
    for sample in range(1, 51):
        m = PositionObservation()
        m.boot_id, m.pdo_sample = BOOT, sample
        m.expected_wkc = m.wkc = 67
        m.valid_mask = 0xfffff
        m.header.frame_id = '' if sample % 2 else 'bench_frame'
        m.header.stamp = node.get_clock().now().to_msg()
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        start = ns()
        pub.publish(m)
        sent.append([sample, stamp, start, ns()])
        time.sleep(.075 if sample == 30 else .01)
    print(json.dumps(sent))


class TimedExecutor(HumbleObservationExecutor):
    def __init__(self):
        super().__init__()
        self.callbacks = []

    def _make_handler(self, entity, node, take_from_wait_list, call_coroutine):
        async def timed(entity, arg):
            start, cpu = ns(), time.thread_time_ns()
            try:
                await call_coroutine(entity, arg)
            finally:
                self.callbacks.append([entity.callback.__name__, start, ns(),
                                       time.thread_time_ns() - cpu])
        return super()._make_handler(entity, node, take_from_wait_list, timed)


class Enqueue(ct.Structure):
    _fields_ = [(k, ct.c_uint64) for k in ('boot', 'sample', 'header_ns', 'begin_ns', 'end_ns')]


def observe(node):
    executor = TimedExecutor()
    health = SessionHealth(lambda: node.get_clock().now().nanoseconds / 1e9)
    taken, stale = [], []
    triggered = False

    def check():
        if health.observation is None:
            return
        try:
            health.require_observation()
        except RuntimeError as exc:
            stale.append([ns(), health.sample, str(exc)])

    def long_callback():
        time.sleep(.060)
        check()

    guard = node.create_guard_condition(long_callback)

    def accept(msg):
        nonlocal triggered
        entered = ns()
        delivery = executor.devices.pdo_delivery
        received = health.now()
        delivery.callback(msg, received, entered / 1e9, time.time_ns())
        health.observe(msg, received_ros=received)
        taken.append([msg.pdo_sample, entered, delivery.history[-1]['header_ros_ns']])
        if not triggered and msg.pdo_sample >= 8:
            triggered = True
            guard.trigger()

    sub = node.create_subscription(PositionObservation, TOPIC, accept, QOS)
    # Same type, writer, topic and process. Matching by topic alone would record duplicates.
    def other_reader(msg):
        pass
    node.create_subscription(PositionObservation, TOPIC, other_reader, QOS)
    node.snowbot_devices = SimpleNamespace(feedback_subscription=sub)
    node.create_timer(.005, check)
    executor.add_node(node)
    probe = ct.CDLL(None)
    probe.bench_bind_reader.argtypes = [ct.c_void_p]
    probe.bench_bind_reader.restype = ct.c_bool
    with sub.handle:
        assert probe.bench_bind_reader(sub.handle.pointer), 'cannot bind target reader GUID'
    env = os.environ.copy()
    env.pop('LD_PRELOAD', None)
    with subprocess.Popen([sys.executable, __file__, 'publish'], stdout=subprocess.PIPE,
                          text=True, env=env) as child:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline and (not taken or taken[-1][0] != 50):
            executor.spin_once(timeout_sec=.05)
            if child.poll() not in (None, 0):
                raise RuntimeError(f'publisher exited with {child.returncode}')
        stdout, _ = child.communicate(timeout=2)
        assert child.returncode == 0
    sent = json.loads(stdout)
    buf, lost = (Enqueue * 256)(), ct.c_uint64()
    probe.bench_snapshot.argtypes = [ct.POINTER(Enqueue), ct.c_size_t, ct.POINTER(ct.c_uint64)]
    probe.bench_snapshot.restype = ct.c_size_t
    count = probe.bench_snapshot(buf, len(buf), ct.byref(lost))
    queue = [{k:getattr(r, k) for k, _ in Enqueue._fields_} for r in buf[:count]]
    executor.shutdown()
    return dict(sent=sent, enqueue=queue, lost=lost.value, taken=taken, stale=stale,
                callbacks=executor.callbacks)


def analyze(data):
    sent, queue, taken = (data[k] for k in ('sent','enqueue','taken'))
    assert len(queue) == len(sent) == 50 and data['lost'] == 0, (len(queue), data['lost'])
    for (sample, stamp, start, _), r in zip(sent, queue):
        assert (BOOT, sample, stamp) == (r['boot'], r['sample'], r['header_ns'])
        assert start <= r['begin_ns'] <= r['end_ns']
    for sample, entered, stamp in taken:
        r = queue[sample-1]
        assert r['header_ns'] == stamp and r['end_ns'] <= entered
    callbacks = data['callbacks']
    [blocked] = [row for row in callbacks if row[0] == 'long_callback']
    _, start, end, cpu = blocked
    arriving = [r['sample'] for r in queue if start < r['end_ns'] < end]
    observed = {r[0] for r in taken}
    skipped = [n for n in arriving if n not in observed]
    assert len(skipped) >= 2 and not any(start < r[1] < end for r in taken)
    assert any(start < t < end and code == 'SNOWBOT_PDO_NOT_ADVANCING'
               for t, _, code in data['stale'])
    # Exclude transport of the last published sample from the no-input interval.
    pause_start, pause_end = max(sent[29][3], queue[29]['end_ns']), sent[30][2]
    pause_enqueues = sum(pause_start < r['end_ns'] < pause_end for r in queue)
    checks = sum(name == 'check' and pause_start < t < pause_end for name,t,_,_ in callbacks)
    assert pause_end-pause_start > 40_000_000 and pause_enqueues == 0 and checks > 1
    assert any(pause_start < t < pause_end and sample == 30 and code == 'SNOWBOT_PDO_NOT_ADVANCING'
               for t,sample,code in data['stale'])
    return dict(published=len(sent), enqueued=len(queue), taken=len(taken), lost=data['lost'],
                long_callback_ms=(end-start)/1e6, long_callback_cpu_ms=cpu/1e6,
                samples_enqueued_during_callback=arriving, enqueued_never_taken=skipped,
                no_publish_ms=(pause_end-pause_start)/1e6, enqueues_during_pause=pause_enqueues,
                executor_checks_during_pause=checks)


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '199' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        sys.exit('requires domain 199 and localhost-only discovery')
    rclpy.init(args=[])
    writer = sys.argv[1] == 'publish'
    node = rclpy.create_node('pdo_bench_writer' if writer else 'pdo_bench_reader',
                             enable_rosout=False, start_parameter_services=False)
    try:
        if writer:
            publish(node)
        else:
            result = observe(node)
            path = Path(sys.argv[1])
            path.write_text(json.dumps(result, separators=(',', ':'))+'\n')
            result['summary'] = analyze(result)
            path.write_text(json.dumps(result, separators=(',', ':'))+'\n')
            print(json.dumps(result['summary']))
    finally:
        node.destroy_node()
        rclpy.shutdown()

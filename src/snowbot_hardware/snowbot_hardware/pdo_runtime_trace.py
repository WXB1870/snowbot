"""Bounded userspace diagnostics for the Orin Humble adapter's PDO reader."""
from collections import deque
import ctypes as ct
import gc
import os
import resource
import sys
import threading
import time

from .pdo_executor import HumbleObservationExecutor
from .session import PdoDeliveryWindow


class Enqueue(ct.Structure):
    _fields_ = [(name, ct.c_uint64) for name in
                ('boot_id', 'pdo_sample', 'header_ros_ns', 'begin_ns', 'end_ns')]


class ReaderWindow(PdoDeliveryWindow):
    def __init__(self, executor, probe):
        super().__init__()
        self.executor = executor
        self.probe = probe
        self.slots = (Enqueue * 256)()

    def trigger(self, detail, now):
        first = self.window is None
        super().trigger(detail, now)
        if first:
            frame = sys._getframe(1)
            stack = []
            while frame and len(stack) < 18:
                stack.append([frame.f_code.co_name, frame.f_lineno])
                frame = frame.f_back
            self.window['check_stack'] = stack
            self.window['check_counters'] = self.executor.counters()
            self.window['check_entity'] = self.executor.entity_name

    def finish(self, now):
        window = super().finish(now)
        if window is None:
            return None
        start = window['trigger_steady_ns'] - 140_000_000
        omitted = ct.c_uint64()
        n = self.probe.bench_snapshot(self.slots, len(self.slots), ct.byref(omitted))
        window['reader_enqueue'] = [[row.boot_id, row.pdo_sample, row.header_ros_ns,
                                     row.begin_ns, row.end_ns] for row in self.slots[:n]
                                    if row.end_ns >= start]
        # The native total includes records displaced before this window.
        window['reader_cumulative_omitted'] = omitted.value
        window['reader_buffer_oldest_end_ns'] = self.slots[0].end_ns if n else None
        exe = self.executor
        window['dispatch_fields'] = ['entity', 'spin_begin', 'handler_ready', 'spin_end']
        window['counter_fields'] = ['mono_ns', 'thread_cpu_ns', 'voluntary', 'involuntary', 'runqueue_wait_ns']
        window['dispatch'] = [entry for entry in exe.recent if entry[-1][0] >= start]
        window['in_progress'] = [exe.entity_name, exe.started, exe.ready]
        window['gc'] = [row for row in tuple(exe.gc_recent) if row[2] >= start]
        return window


class ReaderTraceExecutor(HumbleObservationExecutor):
    def add_node(self, node):
        added = super().add_node(node)
        self.recent = deque(maxlen=192)
        self.gc_recent = deque(maxlen=64)
        self.schedstat = None
        try:
            self.schedstat = os.open('/proc/thread-self/schedstat', os.O_RDONLY | os.O_CLOEXEC)
        except OSError as exc:
            node.get_logger().warn('PDO_USERSPACE_SCHEDSTAT unavailable: '+str(exc))
        probe = ct.CDLL(None)
        if not hasattr(probe, 'bench_bind_reader') or not hasattr(probe, 'bench_snapshot'):
            node.get_logger().warn('PDO_USERSPACE_READER bound=False library unavailable')
            return added
        probe.bench_bind_reader.argtypes = [ct.c_void_p]
        probe.bench_bind_reader.restype = ct.c_bool
        probe.bench_snapshot.argtypes = [ct.POINTER(Enqueue), ct.c_size_t, ct.POINTER(ct.c_uint64)]
        probe.bench_snapshot.restype = ct.c_size_t
        sub = self.devices.feedback_subscription
        with sub.handle:
            bound = probe.bench_bind_reader(sub.handle.pointer)
        if bound:
            self.devices.pdo_delivery = ReaderWindow(self, probe)
            gc.callbacks.append(self.record_gc)
        node.get_logger().info('PDO_USERSPACE_READER bound='+str(bound))
        return added

    def counters(self):
        usage = resource.getrusage(resource.RUSAGE_THREAD)
        wait_ns = None
        if self.schedstat is not None:
            try:
                wait_ns = int(os.pread(self.schedstat, 96, 0).split()[1])
            except (OSError, ValueError, IndexError):
                os.close(self.schedstat)
                self.schedstat = None
        return (time.monotonic_ns(), time.thread_time_ns(), usage.ru_nvcsw, usage.ru_nivcsw, wait_ns)

    def record_gc(self, phase, info):
        self.gc_recent.append((phase, info['generation'], time.monotonic_ns(),
                               time.thread_time_ns(), threading.get_native_id()))

    def wait_for_ready_callbacks(self, *args, **kwargs):
        result = super().wait_for_ready_callbacks(*args, **kwargs)
        entity = result[1]
        self.entity_name = (getattr(entity, 'topic_name', None) or getattr(entity, 'srv_name', None)
                            or getattr(getattr(entity, 'callback', None), '__name__', 'task'))
        self.ready = self.counters()
        return result

    def spin_once(self, timeout_sec=None):
        self.started = self.counters()
        self.entity_name, self.ready = 'wait', None
        try:
            super().spin_once(timeout_sec)
        finally:
            self.recent.append((self.entity_name, self.started, self.ready, self.counters()))

    def shutdown(self, *args, **kwargs):
        if self.record_gc in gc.callbacks:
            gc.callbacks.remove(self.record_gc)
        if getattr(self, 'schedstat', None) is not None:
            os.close(self.schedstat)
            self.schedstat = None
        return super().shutdown(*args, **kwargs)

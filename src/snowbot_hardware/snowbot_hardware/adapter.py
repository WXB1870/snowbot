"""Snowbot session_v3 execution endpoint; controller nodes are never launched here."""
import json
import os
import time
import threading
from rcl_interfaces.msg import ParameterDescriptor
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_runtime.execution_node import ExecutionNode, run_execution


def assemble_snowbot(node, profile):
    node.gripper_only = node.declare_parameter('gripper_only', False,
        descriptor=ParameterDescriptor(read_only=True)).value
    if 'position_protocol' in node._parameter_overrides:
        raise ValueError('SNOWBOT_POSITION_PROTOCOL_REMOVED: session_v3 is fixed; remove position_protocol')
    mode = node.declare_parameter('execution_mode', 'position',
        descriptor=ParameterDescriptor(read_only=True)).value
    if mode not in ('position', 'impedance'):
        raise ValueError('SNOWBOT_EXECUTION_MODE')
    node.snowbot_execution_mode = mode
    from .position_session import PositionSessionDevices
    devices = PositionSessionDevices(node, profile)
    node.snowbot_devices = devices
    return devices.io, {}


class SnowbotAdapter(ExecutionNode):
    def __init__(self):
        super().__init__(allow_hardware=True, device_factory=assemble_snowbot,
                         executor_factory=DelegatedPositionExecutor)
        from .gripper_node import SnowbotGrippers
        self.grippers = self.snowbot_devices.grippers = SnowbotGrippers(self.snowbot_devices)
        self.get_logger().info('Snowbot adapter: command_enabled=%s; interpolation handled by the low-level controller' %
                               self.robot_io.command_enabled)
        self.get_logger().info('Snowbot publication mode env: '+os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE', 'UNSET')+
                               '; qos_from_xml='+os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML', 'UNSET'))
        self.get_logger().info('[SCHED-THREAD] role=adapter pid=%d tid=%d mono_ns=%d' %
                               (os.getpid(), threading.get_native_id(), time.monotonic_ns()))

    def fault(self, request, reply):
        reply.ok, reply.code = False, 'SNOWBOT_SIM_FAULT_UNAVAILABLE'
        return reply

    def event(self, state, code, task_id='', command_id='', observation_id=''):
        if state in ('DEVICE_COMMAND_DETAIL', 'DEVICE_FEEDBACK_DETAIL'):
            logged = getattr(self, '_device_fault_commands', {})
            if state not in logged or logged[state] != command_id:
                data = dict(state=state, code=code, command_id=command_id, task_id=task_id,
                    steady_ns=time.monotonic_ns(),
                    snapshot_at='event_emission',
                    feedback_callbacks=self.snowbot_devices.health.observation_diagnostic(),
                    startup_timing=getattr(self.snowbot_devices, 'startup_timing', None),
                    send_timing=getattr(self.snowbot_devices, 'send_timing', None),
                    tick_timing=getattr(self, '_tick_timing', None),
                    runtime_tick_timing=getattr(self, 'runtime_tick_timing', None),
                    previous_runtime_tick_timing=getattr(self, 'previous_runtime_tick_timing', None))
                try:
                    payload = json.dumps(data, separators=(',', ':'), allow_nan=False)
                except (TypeError, ValueError) as exc:
                    self.get_logger().warn('DEVICE_FAULT_LOG_ERROR:'+str(exc), throttle_duration_sec=1.)
                else:
                    self.get_logger().warn('DEVICE_FAULT '+payload)
                    logged[state] = command_id
                    self._device_fault_commands = logged
        return super().event(state, code, task_id, command_id, observation_id)

    def begin_shutdown(self):
        self.grippers.stop()
        self.robot_io.command_enabled = False
        super().begin_shutdown()

    def lease(self, request, reply):
        startup = getattr(self.snowbot_devices, 'startup_hold_state', 'disabled')
        if request.operation in ('ready', 'acquire', 'acquire_replay') and startup not in ('ready', 'disabled'):
            reply.ok, reply.code, reply.profile_hash = False, 'STARTUP_HOLD_'+startup.upper(), self.profile.digest
            return reply
        if request.operation in ('acquire', 'acquire_replay') and getattr(self, 'grippers', None) and self.grippers.core and self.grippers.core.busy and (
                self.grippers.core.fault or not self.grippers.core.cfg.get('independent_trigger_control',False)):
            reply.ok, reply.code, reply.profile_hash = False, 'GRIPPERS_BUSY_OR_FAULT', self.profile.digest
            return reply
        return super().lease(request, reply)

    def tick(self):
        wall, cpu = time.monotonic(), time.thread_time()
        self._tick_timing = dict(started_monotonic=wall, points=[])
        self.snowbot_devices.advance_session()
        self._tick_timing['points'].append(('session', (time.monotonic()-wall)*1000, (time.thread_time()-cpu)*1000))
        if not self.closing:
            self.snowbot_devices.advance_startup_hold(self.engine)
            self._tick_timing['points'].append(('startup_hold', (time.monotonic()-wall)*1000, (time.thread_time()-cpu)*1000))
            self.snowbot_devices.brake_before_source_expiry(self.engine)
            self._tick_timing['points'].append(('source_expiry', (time.monotonic()-wall)*1000, (time.thread_time()-cpu)*1000))
        super().tick()
        self._tick_timing['points'].append(('engine', (time.monotonic()-wall)*1000, (time.thread_time()-cpu)*1000))
        self.record_stop_check()
        self._tick_timing['points'].append(('stop_log', (time.monotonic()-wall)*1000, (time.thread_time()-cpu)*1000))
        if getattr(self, 'grippers', None) and self.grippers.core:
            self.grippers.tick()
        self._tick_timing['points'].append(('gripper', (time.monotonic()-wall)*1000, (time.thread_time()-cpu)*1000))
        self._previous_tick_timing = self._tick_timing

    def record_stop_check(self):
        # Emit from the explicit non-RT loop, never from a health predicate.
        devices = getattr(self, 'snowbot_devices', None)
        health = getattr(devices, 'health', None)
        engine = getattr(self, 'engine', None)
        if engine is None or not hasattr(health, 'hold_failure'):
            return
        from snowbot_hardware.session import stop_diagnostic
        data = stop_diagnostic(health, engine, self.now())
        if data is None:
            self._stop_check_signature = None
            return
        terminal = (data['reason'], tuple(data['stop_failures'])) if engine.stop_fault or data['stop_failures'] else None
        signature = (data['boot_id'], data['session'], data['sequence'], terminal)
        steady = engine.steady()
        first = signature != getattr(self, '_stop_check_signature', None)
        if (first or
                engine.stopping and steady-getattr(self, '_stop_check_at', 0.) >= .5):
            if first:
                data['feedback_callbacks'] = health.observation_diagnostic()
                data['startup_timing'] = getattr(devices, 'startup_timing', None)
                data['send_timing'] = getattr(devices, 'send_timing', None)
                data['tick_timing'] = getattr(self, '_tick_timing', None)
                data['previous_tick_timing'] = getattr(self, '_previous_tick_timing', None)
                data['runtime_tick_timing'] = getattr(self, 'runtime_tick_timing', None)
                data['previous_runtime_tick_timing'] = getattr(self, 'previous_runtime_tick_timing', None)
                data['publication_mode_env'] = os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE', 'UNSET')
                data['publication_qos_from_xml'] = os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML', 'UNSET')
            try:
                payload = json.dumps(data, separators=(',', ':'), allow_nan=False)
            except (TypeError, ValueError) as exc:
                self.get_logger().warn('STOP_CHECK_LOG_ERROR:'+str(exc), throttle_duration_sec=1.)
                return  # Diagnostic encoding must not terminate the control loop.
            self._stop_check_signature, self._stop_check_at = signature, steady
            reason = data['reason']
            if reason == 'PAUSE_SEQUENCE_MISMATCH':
                summary = '等待暂停回执：期望序号%s，当前序号%s' % (
                    data['sequence'], data['observed']['command_sequence'])
            elif reason == 'STOP_SPEED_NOT_ZERO':
                summary = '等待停稳：速度%.3f rad/s，阈值%.3f rad/s' % (
                    data['max_speed_rad_s'], data['velocity_tolerance_rad_s'])
            else:
                summary = reason
            summary = 'STOP_CHECK '+summary+' session=%s sequence=%s' % (data['session'], data['sequence'])
            if not terminal and reason in ('PAUSE_SEQUENCE_MISMATCH', 'STOP_SPEED_NOT_ZERO',
                                           'WAIT_REST_WINDOW', 'WAIT_EXECUTOR'):
                self.get_logger().info(summary)
            else:
                self.get_logger().warn(summary+' failures=%s；详情见 runtime-events.jsonl' % data['stop_failures'])
            motion = engine.stopping[0] if engine.stopping else getattr(self, 'last_event', None)
            self.event('STOP_CHECK', payload,
                       task_id=motion.task_id if motion else '',
                       command_id=motion.command_id if motion else '')

    def shutdown_pending(self):
        return super().shutdown_pending() or self.grippers.stopping

    def shutdown_failed(self):
        return super().shutdown_failed() or bool(self.grippers.core and self.grippers.core.fault)

    def abort_shutdown(self):
        if self.grippers.core and self.grippers.stopping:
            self.grippers.core.fail('SHUTDOWN_TIMEOUT')
        self.robot_io.stop_failures = ('vendor_hold',)
        self.engine.halt(self.now(), 'STOP_UNCONFIRMED', revoke=True, emergency=True)

    def destroy_node(self):
        self.snowbot_devices.heartbeat_running = False
        return super().destroy_node()


def main():
    os.environ.setdefault('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS')
    run_execution(SnowbotAdapter)

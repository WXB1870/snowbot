"""Snowbot session_v3 execution endpoint; controller nodes are never launched here."""
import json
import os
import time
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

    def fault(self, request, reply):
        reply.ok, reply.code = False, 'SNOWBOT_SIM_FAULT_UNAVAILABLE'
        return reply

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
        signature = (data['boot_id'], data['session'], data['sequence'], data['reason'],
                     data['ack_reason'], tuple(data['stop_failures']))
        steady = engine.steady()
        if (signature != getattr(self, '_stop_check_signature', None) or
                engine.stopping and steady-getattr(self, '_stop_check_at', 0.) >= .5):
            data['feedback_callbacks'] = health.observation_diagnostic()
            data['tick_timing'] = getattr(self, '_tick_timing', None)
            data['previous_tick_timing'] = getattr(self, '_previous_tick_timing', None)
            data['publication_mode_env'] = os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE', 'UNSET')
            data['publication_qos_from_xml'] = os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML', 'UNSET')
            try:
                payload = json.dumps(data, separators=(',', ':'), allow_nan=False)
            except (TypeError, ValueError) as exc:
                self.get_logger().warn('STOP_CHECK_LOG_ERROR:'+str(exc), throttle_duration_sec=1.)
                return  # Diagnostic encoding must not terminate the control loop.
            self._stop_check_signature, self._stop_check_at = signature, steady
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

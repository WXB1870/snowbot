"""Native Snowbot gripper commands; measured state comes from PositionObservation."""
import json
import math
import time
from pathlib import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from snowbot_hardware.gripper import GripperController, SIDES, GROUPS, JOINTS, MASKS, native_command
from snowbot_hardware.parameters import TARGET_FEEDBACK_MAX_AGE_S, load_mode_config, mode_config_digest
from bindu_contracts.teleoperation import VRFrame
from bindu_interfaces.msg import GripperState, VRInput
from bindu_interfaces.srv import GripperControl, Lease
from bindu_runtime.common import seconds, stamp


class SnowbotGrippers:
    def __init__(self, devices):
        self.devices, self.node = devices, devices.node
        node = self.node
        node.declare_parameter('gripper_config', '', descriptor=ParameterDescriptor(read_only=True))
        self.core = None
        path = node.get_parameter('gripper_config').value
        if not path:
            return
        cfg = load_mode_config(path,node.runtime_mode)
        self.config_hash = mode_config_digest(cfg,node.runtime_mode)
        if cfg.get('simulation_only', False) is not False:
            raise ValueError('REAL_GRIPPER_CONFIG_REQUIRED')
        prefix = node.get_parameter('vendor_namespace').value.rstrip('/')
        self.topics = {side: prefix+'/gripper/'+side+'_cmd' for side in SIDES}
        self.sent_positions = {}
        enabled = node.robot_io.command_enabled or getattr(node, 'gripper_only', False)
        self.publishers = ({side: node.create_publisher(devices.array_type, topic, 1)
                            for side, topic in self.topics.items()}
                           if enabled and (cfg.get('calibrated') or
                               cfg.get('allow_uncalibrated_direct', False)) else {})
        self.core = GripperController(node.profile, cfg, self.send, self.health, node.now, self.arms_idle,
                                      enabled=enabled, require_driver_ack=False,
                                      runtime_parameters=getattr(devices,'runtime_parameters',None))
        self.bind_services()

    def bind_services(self):
        node = self.node
        self.state_pub = node.create_publisher(GripperState, 'gripper/state', 10)
        node.create_service(Lease, 'gripper/lease', self.lease)
        node.create_service(GripperControl, 'gripper/control', self.control)
        input_topic=('teleop/vr/gripper_input' if self.core.cfg.get('independent_trigger_control',False)
                     else 'teleop/vr/input')
        node.create_subscription(VRInput, input_topic, self.input, qos_profile_sensor_data)

    def arms_idle(self):
        engine = getattr(self.node, 'engine', None)
        if getattr(self.node, 'gripper_only', False):
            from .session import CONTROL_NAMES, CONTROL_MASK
            obs = self.devices.health.require_observation(TARGET_FEEDBACK_MAX_AGE_S)
            velocities = self.devices.io.last_feedback.joint_velocities
            return bool(engine and not engine.stop_fault and not engine.lease_id and
                not engine.active and not engine.stopping and obs.valid_mask & CONTROL_MASK == CONTROL_MASK and
                all(math.isfinite(velocities.get(n, math.inf)) and
                    abs(velocities[n]) <= self.node.profile.feedback_tolerances['joint_velocity']
                    for n in CONTROL_NAMES))
        # Independent fingers may move during arm execution; fault barriers remain.
        return bool(engine and not engine.stop_fault and
            (self.core.cfg.get('independent_trigger_control',False) or
             not engine.lease_id and not engine.active and not engine.stopping))

    def health(self):
        sides = self.core.feedback_sides()
        obs = self.devices.health.require_observation(TARGET_FEEDBACK_MAX_AGE_S)
        for side, slot in (('left', 7), ('right', 15)):
            if side not in sides:
                continue
            if not obs.valid_mask & MASKS[side]:
                raise RuntimeError(f'GRIPPER_DRIVER_FEEDBACK_INVALID:{side}:slot={slot}')
            if obs.status[slot] & 8:
                raise RuntimeError(f'GRIPPER_DRIVER_FAULT:{side}:slot={slot}:status=0x{obs.status[slot]:04x}')
        for side in sides:
            topic = self.topics[side]
            if side not in self.publishers or self.devices.graph.count_publishers(topic) != 1:
                raise RuntimeError('GRIPPER_COMMAND_PUBLISHER_CONFLICT:'+side)
            readers = self.devices.graph.get_subscriptions_info_by_topic(topic)
            if not any(info.node_namespace.rstrip('/')+'/'+info.node_name == self.devices.receivers['head']
                       and info.topic_type == 'std_msgs/msg/Float64MultiArray' for info in readers):
                raise RuntimeError('GRIPPER_RECEIVER_UNAVAILABLE:'+side)
        result = dict(self.devices.health.messages['servo'])
        result['sample'] = obs.pdo_sample
        return result

    def send(self, side, position, hold, sequence):
        self.health()
        cfg = self.core.cfg['grippers'][side]
        # Native servo topics interpolate each target but have no speed cap.
        # Bound target increments here; a delivery gap cannot create a large step.
        now = time.monotonic()
        previous, when = self.sent_positions.get(side, (self.core.q[JOINTS[side]], now))
        if not hold:
            step = cfg['max_speed']*min(max(now-when, 0.), self.node.profile.control_period)
            position = previous+max(-step, min(step, position-previous))
        values = native_command(position, cfg)
        self.publishers[side].publish(self.devices.array_type(data=values))
        self.sent_positions[side] = (position, now)

    def lease(self, req, reply):
        try:
            if self.node.closing:
                raise RuntimeError('SHUTDOWN')
            self.core.lease(req.operation, req.owner, req.resources, req.lease_id, req.epoch)
            reply.ok, reply.code = True, 'OK'
        except RuntimeError as exc:
            reply.ok, reply.code = False, str(exc)
        reply.lease_id, reply.epoch, reply.profile_hash = self.core.lease_id, self.core.epoch, self.node.profile.digest
        return reply

    def control(self, req, reply):
        try:
            if self.node.closing:
                raise RuntimeError('SHUTDOWN')
            self.core.command(req.operation, req.side, req.lease_id, req.epoch, req.sequence,
                seconds(req.stamp), req.valid_for, position=req.position,
                load_state=req.load_state, external_support=req.external_support)
            reply.accepted, reply.code = True, 'REQUEST_ACCEPTED'
        except (RuntimeError, ValueError) as exc:
            reply.accepted, reply.code = False, str(exc)
        return reply

    def input(self, msg):
        if self.node.matches_mode(msg):
            previous=self.core.last_request_rejection
            frame = VRFrame(msg.source_id, msg.seq, seconds(msg.stamp), msg.side, tuple(msg.pose), msg.grip, msg.trigger,
                msg.init, msg.stop, msg.valid, msg.clutch_seq, msg.mapping, msg.buttons, tuple(int(value) for value in msg.button_edges), msg.virtual_grip)
            self.core.input(frame)
            if self.core.last_request_rejection is not previous:
                self.node.event('GRIPPER_REQUEST_REJECTED',json.dumps(self.core.last_request_rejection))

    def tick(self):
        self.core.tick()
        for side in SIDES:
            data = self.core.snapshot(side)
            when = data.pop('feedback_stamp')
            self.state_pub.publish(GripperState(stamp=stamp(self.node.now()), feedback_stamp=stamp(when),
                runtime_mode=self.node.runtime_mode, instance_id=self.node.instance_id, profile_hash=self.node.profile.digest,
                config_hash=self.config_hash, **data))

    def stop(self):
        if self.core:
            self.core.stop('GRIPPER_SHUTDOWN', revoke=True)

    @property
    def stopping(self):
        return bool(self.core and any(c['state'] == 'STOPPING' for c in self.core.channels.values()))


def run_gripper_open(node, spin, clock=time.monotonic):
    """One explicit endpoint operation; never acquires the arm execution lease."""
    core = node.grippers.core
    deadline = clock()+5.
    # DDS can deliver feedback before resolving the command receiver's name.
    while (core.feedback_stamp == 0. or node.snowbot_devices.health.progress is None or
           not all(any(info.node_namespace.rstrip('/')+'/'+info.node_name == node.snowbot_devices.receivers['head']
                       and info.topic_type == 'std_msgs/msg/Float64MultiArray'
                       for info in node.snowbot_devices.graph.get_subscriptions_info_by_topic(node.grippers.topics[side]))
                   for side in SIDES)):
        if clock() >= deadline:
            raise RuntimeError('GRIPPER_FEEDBACK_OR_RECEIVER_WAIT_TIMEOUT')
        spin()
    core.lease('acquire', 'open-grippers', tuple(GROUPS.values()))
    lease, epoch = core.lease_id, core.epoch
    failure = None
    try:
        sequence = 0
        for operation, side in (('authorize_direct', ''), ('open', 'left'), ('open', 'right')):
            sequence += 1
            core.command(operation, side, lease, epoch, sequence, node.now(),
                         core.cfg['command_ttl'], load_state='unknown')
        renewed = clock()
        deadline = renewed+core.cfg['motion_timeout']+core.cfg['stop_timeout']
        while not all(c['state'] == 'SUCCEEDED' for c in core.channels.values()):
            for c in core.channels.values():
                if c['state'] in ('FAILED', 'CANCELED'):
                    raise RuntimeError(c['code'])
            if clock() >= deadline:
                raise RuntimeError('GRIPPER_OPEN_TIMEOUT')
            if clock()-renewed >= .5:
                core.lease('renew', lease_id=lease, epoch=epoch)
                renewed = clock()
            spin()
    except BaseException as exc:
        failure = exc
    finally:
        if core.lease_id == lease and core.epoch == epoch:
            core.lease('release', lease_id=lease, epoch=epoch)
        deadline = clock()+core.cfg['stop_timeout']
        while any(c['state'] == 'STOPPING' for c in core.channels.values()) and clock() < deadline:
            spin()
        if core.fault or any(c['state'] == 'STOPPING' for c in core.channels.values()):
            raise RuntimeError(str(failure or '')+';'+(core.fault or 'GRIPPER_STOP_UNCONFIRMED')) from failure
    if failure is not None:
        raise failure
    return {side: core.q[JOINTS[side]] for side in SIDES}

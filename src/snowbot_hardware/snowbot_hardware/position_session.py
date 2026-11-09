"""Typed three-layer position sessions, using the independent PDO observation."""
import json
import math
import os
import time
import resource

from snowbot_hardware.protocol import CommandNotSent, SnowbotIO, GROUPS
from snowbot_hardware.session import CONTROL_GROUPS, CONTROL_NAMES, ROS_FEEDBACK_TIMEOUT_S, SessionHealth, SourceEnvelope, new_session_id
from bindu_runtime.teleop_deployment import TARGET_STREAM_LIMIT_DEFAULTS, target_stream_limits
from .parameters import load_parameters, parameters_for


from bindu_runtime.stop_evidence import sampled_nominal_reference


class PositionSessionIO(SnowbotIO):
    requesting_pause = False
    trajectory_reference = True
    target_stream = True
    servo_reference = None
    captured_pause_request = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pause_targets = None
        self.writable_groups = CONTROL_GROUPS
        self.coupled_group_sets = (CONTROL_GROUPS,)
        # One atomic envelope and one PAUSE identity for waist and both arms.
        self.stop_group_sets = (('waist', 'left_arm', 'right_arm'),)

    @property
    def stop_arrival_joints(self):
        # A quiet speed sample cannot prove CSP rest while the drive still
        # has position travel left to its fixed, acknowledged hold point.
        return tuple(self.profile.groups.get('waist', ()))

    def validate_motion(self, motion, now):
        SourceEnvelope.validate_motion(motion, now)

    def capture(self, groups):
        if set(groups) != CONTROL_GROUPS:
            raise RuntimeError('SNOWBOT_POSITION_SESSION_RESOURCE_LAYOUT')
        previous_pause = getattr(self, 'pause_request', None) or self.captured_pause_request
        super().capture(groups)
        self.captured_pause_request = previous_pause
        self.pause_request = None
        self.servo_reference = None
        self.after_capture()

    def sampled_reference(self):
        # ROS snapshot lifetime; the servo's local RT deadline remains 20 ms.
        sample = self.servo_reference
        if (sample is None or not (self.targets or self.stop_targets) or
                not 0 <= self.now()-sample[0] <= ROS_FEEDBACK_TIMEOUT_S):
            return (self.now(), {}, {}, {})
        return sample

    @property
    def reference_positions(self):
        return self.sampled_reference()[1]

    def confirmed_stop_targets(self):
        targets = self.pause_feedback(getattr(self, 'pause_request', None))
        if targets is None:
            return None
        if any(not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n, q in targets.items()):
            return {}  # Executor rejects an invalid ACK without changing its budget.
        if self.pause_targets is None:
            self.pause_targets = targets
        # A hold point is fixed for this PAUSE. Reject a changed acknowledgement;
        # measured drift must never move the expected stopping point.
        return self.pause_targets if targets == self.pause_targets else {}

    def stop(self):
        previous = self.requesting_pause
        self.requesting_pause = True
        try:
            # PAUSE carries the existing session authority. Its position
            # fields are only a wire/layout footprint: the servo brakes
            # from its own reference, and its ACK supplies the hold point.
            # In particular, do not require a fresh ROS measurement to ask
            # that a following session stop producing motion.
            self.reference.clear()
            if not self.targets:
                return  # Pending PAUSE is not renewed on repeated faults.
            self.stop_requested_at = self.now()
            target = dict(self.targets)
            try:
                self._send(target, hold=True)
            except RuntimeError:
                self.stop_failures = tuple(sorted(self._groups(target)))
            else:
                self.stop_targets = target
            self.targets.clear()
        finally:
            self.requesting_pause = previous


class PositionSessionDevices:
    def __init__(self, node, profile):
        from rcl_interfaces.msg import ParameterDescriptor
        from snowbot_ctl_msgs.msg import PositionObservation, PositionSessionCommand, PositionSessionState
        from std_msgs.msg import Float64MultiArray
        from copy import copy
        from rclpy.qos import qos_profile_sensor_data
        if node.runtime_mode != 'real':
            raise ValueError('SNOWBOT_VENDOR_REQUIRES_REAL_MODE')
        self.node, self.grippers = node, None
        from .protocol import GraphSnapshot
        self.graph = GraphSnapshot(node)
        self.runtime_parameters=load_parameters(node.declare_parameter('parameters_config','',descriptor=ParameterDescriptor(read_only=True)).value or None)
        node.device_backend = 'snowbot_vendor'
        node.declare_parameter('command_enabled', False, descriptor=ParameterDescriptor(read_only=True))
        if getattr(node, 'gripper_only', False) and node.get_parameter('command_enabled').value:
            raise ValueError('GRIPPER_ONLY_REQUIRES_ARM_COMMANDS_DISABLED')
        node.declare_parameter('vendor_namespace', '/', descriptor=ParameterDescriptor(read_only=True))
        if node.get_parameter('vendor_namespace').value != '/':
            raise ValueError('SNOWBOT_POSITION_SESSION_ROOT_NAMESPACE_REQUIRED')
        if 'target_stream_brake_lead_s' in node._parameter_overrides:
            raise ValueError('SNOWBOT_BRAKE_LEAD_MOVED: set execution.target_stream_brake_lead_s in parameters_config')
        for key in ('arm_receiver_node', 'mid_receiver_node', 'servo_receiver_node'):
            node.declare_parameter(key, '', descriptor=ParameterDescriptor(read_only=True))
        self.receivers = {key: node.get_parameter(parameter).value or default for key, parameter, default in (
            ('arms', 'arm_receiver_node', '/robot_upperMonitor'),
            ('middle', 'mid_receiver_node', '/dual_arm_control_node'),
            ('head', 'servo_receiver_node', '/servo_node'))}
        self.health = SessionHealth(node.now, execution_mode=node.snowbot_execution_mode)
        self.trajectory_limits = target_stream_limits({
            key: node.declare_parameter(key, default,
                                        descriptor=ParameterDescriptor(read_only=True)).value
            for key, default in TARGET_STREAM_LIMIT_DEFAULTS.items()})
        self.envelopes = SourceEnvelope(new_session_id(node.now()), node.now)
        self.sent = {}
        self.target_instance = None
        self.source_identity = None
        self.target_fault = ''
        self.applied_sequence = 0
        self.takeover_pending = None
        self.takeover_previous_hold = None
        self.io = PositionSessionIO(profile, self.send, node.now,
                            command_enabled=node.get_parameter('command_enabled').value,
                            simulated=False, check_ready=self.check_ready, check_health=self.check_health)
        self.io.source_motion = None
        self.io.trajectory_reference = True
        self.io.target_stream = True
        self.io.supports_rigid_reference = True
        self.io.supports_prepared_replay = True
        self.io.execution_mode = node.snowbot_execution_mode
        self.io.replay_timing = dict(self.runtime_parameters['replay'],
            completion_timeout_s=self.runtime_parameters['pico']['motion_completion_grace_s'],
            settle_time_s=self.runtime_parameters['execution']['stop_confirmation_window_s'],
            velocity_tolerance_rad_s=max(self.runtime_parameters['execution']['motion_arrival_velocity_tolerance_rad_s'],
                                         profile.feedback_tolerances['joint_velocity']))
        self.io.replay_position_tolerances = {n:max(math.radians(2), profile.stop_position_tolerances.get(
            n, profile.feedback_tolerances['joint_position'])) for n in CONTROL_NAMES}
        from pathlib import Path
        from .parameters import load_mode_config
        self.io.replay_prepare_tolerance = load_mode_config(Path(node.profile_path).with_name('teleop.json'),
                                                          'real')['prepare_position_tolerance_rad']
        self.io.held_reference = self.held_reference
        self.io.prepare_replay = self.prepare_replay
        self.io.stop_confirmation_window = self.runtime_parameters['execution']['stop_confirmation_window_s']
        self.io.stop_confirmation_max_gap = ROS_FEEDBACK_TIMEOUT_S
        self.io.stop_requires_arrival = False
        self.startup_hold_state = ('waiting' if self.io.command_enabled else 'disabled')
        self.startup_home = node.declare_parameter('startup_home', os.environ.get('SNOWBOT_STARTUP_HOME') == '1',
            descriptor=ParameterDescriptor(read_only=True)).value
        if getattr(node, 'gripper_only', False) and self.startup_home:
            raise ValueError('GRIPPER_ONLY_FORBIDS_HOME')
        self.io.after_capture = lambda: self.new_session()
        self.io.pause_feedback = self.health.confirmed_hold
        self.arm_type, self.array_type = PositionSessionCommand, Float64MultiArray
        self.feedback_topic = '/bindu/position/observation'
        self.command_topics = {'arms': '/bindu/position/command'}
        self.publishers = {'arms': node.create_publisher(PositionSessionCommand, self.command_topics['arms'], 1)} if self.io.command_enabled else {}
        self.heartbeat_running = False
        # This is current state, not a lossless history. Do not drain obsolete
        # samples after a callback stall; all existing freshness gates still apply.
        feedback_qos=copy(qos_profile_sensor_data)
        feedback_qos.depth=1
        node.create_subscription(PositionObservation, self.feedback_topic, self.receive, feedback_qos)
        for layer, receiver in (('upper', 'arms'), ('middle', 'middle'), ('servo', 'head')):
            topic = '/bindu/position/'+layer+'_state'
            node.create_subscription(PositionSessionState, topic,
                lambda msg, l=layer, t=topic, r=receiver: self.receive_state(l, t, r, msg), 1)
        node.create_subscription(PositionSessionState, '/bindu/position/servo_receipt', self.receive_receipt, 1)
        from std_srvs.srv import Trigger
        node.create_service(Trigger, 'execution/held_reference', self.read_held_reference)

    def read_held_reference(self, request, reply):
        """Read the confirmed PAUSE point without acquiring or sending anything."""
        engine = self.node.engine
        try:
            instance, boot, session, sequence, positions = self.held_reference()
            targets = dict(positions)
            obs = self.health.observation
            reply.success, reply.message = True, json.dumps(dict(
                instance_id=instance, profile_hash=self.io.profile.digest,
                lease_id=engine.lease_id, epoch=engine.epoch, revision=engine.revision,
                stamp=obs.header.stamp.sec+obs.header.stamp.nanosec*1e-9-obs.pdo_age,
                positions=targets, boot_id=boot, session=session, sequence=sequence))
        except RuntimeError as exc:
            reply.success, reply.message = False, str(exc)
        return reply

    def held_reference(self):
        """Authenticate one fixed PAUSE reference; acquisition owns measured rest."""
        engine = self.node.engine
        if (self.node.closing or engine.active or engine.stopping or engine.stop_fault or self.io.stop_failures):
            raise RuntimeError('SNOWBOT_HELD_REFERENCE_REQUIRES_STOPPED_LEASE')
        identity = getattr(self.io, 'pause_request', None) or self.io.captured_pause_request
        targets = self.health.confirmed_hold(identity)
        if targets is None:
            reason = self.health.hold_failure(identity)
            detail = json.dumps(dict(reason=reason, pause_request=identity,
                **self.health.observation_diagnostic()), separators=(',', ':'))
            self.node.event('HELD_REFERENCE_REJECTED', detail)
            self.node.get_logger().warn('SNOWBOT_HELD_REFERENCE_DIAGNOSTIC:'+detail,
                                       throttle_duration_sec=1.)
            raise RuntimeError('SNOWBOT_HELD_REFERENCE:'+reason)
        if any(not self.io.profile.limits[n][0] <= q <= self.io.profile.limits[n][1] for n,q in targets.items()):
            raise RuntimeError('SNOWBOT_HELD_REFERENCE_LIMIT')
        if self.io.pause_targets is None:
            self.io.pause_targets = targets
        elif targets != self.io.pause_targets:
            raise RuntimeError('SNOWBOT_HELD_REFERENCE_CHANGED')
        obs = self.health.observation
        return self.node.instance_id, obs.boot_id, obs.session, obs.command_sequence, tuple(sorted(targets.items()))

    def prepare_replay(self, prepared):
        """Populate the bulk ROS arrays before acquiring the lease."""
        wire = self.arm_type().trajectory
        wire.version = 4 if prepared.inner_reference else 5
        order = [prepared.motion.names.index(n) for n in CONTROL_NAMES]
        for start, duration, source_duration, elapsed, controls in prepared.description:
            wire.piece_start.append(start)
            wire.duration.append(duration)
            wire.source_duration.append(source_duration)
            wire.source_elapsed.append(elapsed)
            for i in order:
                wire.control_count.append(len(controls[i]))
                wire.controls.extend(controls[i])
        self.replay_wire = wire

    def receive_receipt(self, msg):
        """Takeover ACK opens its initial barrier; ordinary targets/PAUSE never wait."""
        try:
            self.identity('/bindu/position/servo_receipt', self.receivers['head'],
                          'snowbot_ctl_msgs/msg/PositionSessionState')
            sequence = msg.sequence if msg.state == 'accepted' else msg.rejected_sequence
            source = self.sent.get(sequence)
            if source is None:
                return
            metadata, instance, command_id = source
            session = msg.session if msg.state == 'accepted' else msg.rejected_session
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if (msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000 or
                    msg.layer != 'servo' or msg.wire_version != 5 or msg.reference_mode != 2 or
                    msg.state not in ('accepted', 'rejected') or msg.instance_id != instance or
                    msg.feedback_boot_id != self.health.boot or session != metadata['session'] or
                    not metadata['source_stamp_ns'] <= stamp < metadata['expires_at_ns'] or
                    stamp > round(self.node.now()*1e9)):
                return
            self._settle_takeover(msg if msg.state == 'accepted' else None, receipt=True)
            if msg.state == 'rejected' and sequence == self.envelopes.sequence:
                self.target_fault = 'SNOWBOT_TARGET_REJECTED:'+msg.reason
            self.node.event('TARGET_'+msg.state.upper(), 'SNOWBOT_TARGET:'+json.dumps(dict(
                session=session, sequence=sequence, reason=msg.reason), separators=(',', ':')),
                command_id=command_id)
        except (ValueError, RuntimeError) as exc:
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def identity(self, topic, expected, kind, *, discovering=False):
        infos = self.graph.get_publishers_info_by_topic(topic)
        # Fast DDS may deliver samples before its graph resolves the node name.
        # Before the first measured frame, discard these samples and keep waiting.
        if discovering and self.health.observation is None and (not infos or
                len(infos) == 1 and infos[0].topic_type == kind and
                (infos[0].node_name == '_NODE_NAME_UNKNOWN_' or
                 infos[0].node_namespace == '_NODE_NAMESPACE_UNKNOWN_')):
            return False
        matched = [i for i in infos if i.node_namespace.rstrip('/')+'/'+i.node_name == expected and i.topic_type == kind]
        if len(matched) != 1 or len(infos) != 1:
            found = [dict(node=i.node_namespace.rstrip('/')+'/'+i.node_name, type=i.topic_type) for i in infos]
            raise RuntimeError('SNOWBOT_SESSION_ENDPOINT:'+topic+';expected='+expected+
                               ';type='+kind+';found='+json.dumps(found, separators=(',', ':')))
        return True

    def receive_state(self, layer, topic, receiver, msg):
        try:
            if not self.identity(topic, self.receivers[receiver],
                                 'snowbot_ctl_msgs/msg/PositionSessionState', discovering=True):
                return
            self.health.state(layer, msg)
        except (ValueError, RuntimeError) as exc:
            self.health.states.pop(layer, None)
            self.node.get_logger().warn(str(exc), throttle_duration_sec=1.)

    def receive(self, msg):
        entered = self.health.steady()
        source = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        source_age = self.node.now()-source
        self.health.observation_callback_count += 1
        identity_finished, outcome = None, 'publisher_discovering'
        try:
            if not self.identity(self.feedback_topic, self.receivers['head'],
                                 'snowbot_ctl_msgs/msg/PositionObservation', discovering=True):
                return
            identity_finished = self.health.steady()
            q, v, stamp = self.health.observe(msg)
            if self.grippers and self.grippers.core and stamp > self.grippers.core.feedback_stamp:
                try:
                    self.grippers.core.observe(q, v, {'left': float(msg.current[7]),
                        'right': float(msg.current[15])}, stamp)
                except ValueError as exc:
                    self.grippers.core.invalid = str(exc)
            self.io.observe(q, v, stamp, self.node.now(), sample=msg.pdo_sample)
            if msg.session_selected and msg.session == self.envelopes.session:
                self.takeover_previous_hold = None
            self.io.servo_reference = None
            sample = getattr(msg, 'sample_time_ns', 0)*1e-9
            arrays = [getattr(msg, field, ()) for field in
                      ('reference_position','reference_speed','reference_acceleration')]
            if (msg.session_selected and msg.session == self.envelopes.session and
                    getattr(msg, 'reference_mode', 0) == 2 and getattr(msg, 'reference_valid', False) and
                    getattr(msg, 'output_valid', False) and getattr(msg, 'segment_id', 0) == msg.command_sequence and
                    0 <= self.node.now()-sample <= ROS_FEEDBACK_TIMEOUT_S and
                    all(all(math.isfinite(x) for x in values) for values in arrays)):
                self.io.servo_reference = (sample, *[dict(zip(CONTROL_NAMES, values)) for values in arrays])
                source = self.sent.get(msg.command_sequence)
                if (source is not None and msg.command_sequence > self.applied_sequence and
                        getattr(msg, 'trajectory_phase', 0) in (2, 3, 4, 5) and
                        source[0]['source_stamp_ns'] <= msg.sample_time_ns < source[0]['expires_at_ns']):
                    self.applied_sequence = msg.command_sequence
                    self.node.event('TARGET_APPLIED', 'SNOWBOT_TARGET:'+json.dumps(dict(
                        session=msg.session, sequence=msg.command_sequence,
                        sample_time_ns=msg.sample_time_ns), separators=(',', ':')),
                        command_id=source[2])
            outcome = 'accepted'
        except (ValueError, RuntimeError) as exc:
            outcome = str(exc)
            self.io.invalid = str(exc)
            detail=str(exc)
            if detail=='SNOWBOT_PDO_STALE':
                detail+=' '+json.dumps(dict(publish_age_ms=1000*(self.node.now()-
                    (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)),
                    pdo_age_ms=1000*msg.pdo_age,pdo_sample=msg.pdo_sample,
                    previous_sample=self.health.sample,wkc=msg.wkc,expected_wkc=msg.expected_wkc))
            self.node.get_logger().warn(detail, throttle_duration_sec=1.)
        finally:
            finished = self.health.steady()
            self.health.observation_callbacks.append((entered, finished, msg.pdo_sample,
                msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, msg.pdo_age,
                (finished if identity_finished is None else identity_finished)-entered,
                source_age, outcome))

    def _check_session_health(self, groups, *, acquiring=False):
        obs = self.health.observation
        waiting = obs is not None and self._waiting_for_takeover_pdo(obs)
        self.health.require(groups, acquiring=(acquiring or waiting or self.io.requesting_pause or bool(self.io.stop_targets)))
        if obs.session_selected and obs.session != self.envelopes.session:
            # A paused/expired session can be explicitly replaced after capture.
            # During a pending stop, ignore its delayed observation and wait for
            # the exact new PAUSE identity; it cannot confirm this stop.
            if not (acquiring or self.io.requesting_pause or self.io.stop_targets or
                    waiting) or obs.session_reason < 1:
                raise RuntimeError('SNOWBOT_SESSION_OWNER_CONFLICT')

    def _waiting_for_takeover_pdo(self, obs):
        # ACK may precede PDO adoption. Only the captured old hold/stop may
        # remain until adoption or the original TAKEOVER deadline.
        previous = self.takeover_previous_hold
        return bool(previous is not None and
            (obs.boot_id, obs.session, obs.command_sequence,
             self.health.states['servo'][1].instance_id) == previous[:4] and
            round(self.node.now()*1e9) < previous[4] and
            obs.session_reason == previous[5] and
            (obs.session_reason >= 2 or obs.hold_valid and
             getattr(obs, 'trajectory_phase', 0) == 4 and
             getattr(obs, 'reference_valid', False) and getattr(obs, 'output_valid', False)))

    def check_health(self, groups, *, acquiring=False):
        self._check_session_health(groups, acquiring=acquiring)
        if self.target_fault and not (self.io.requesting_pause or self.io.sending_hold):
            if not (acquiring and self._completed_pause_rejection()):
                raise RuntimeError(self.target_fault)

    def _completed_pause_rejection(self):
        # An expired repeat of a completed PAUSE cannot poison the next capture.
        # Keep the rejection recorded; only a successful capture opens a new session.
        source = self.sent.get(self.envelopes.sequence)
        if (self.target_fault != 'SNOWBOT_TARGET_REJECTED:duplicate_or_expired_frame' or
                source is None or self.io.targets or self.io.stop_targets or self.io.stop_failures):
            return False
        metadata = source[0]
        request = getattr(self.io, 'pause_request', None)
        return bool(metadata['operation'] == 2 and
            source[1] == self.health.states['servo'][1].instance_id and
            round(self.node.now()*1e9) >= metadata['expires_at_ns'] and
            request and request[:2] == (metadata['session'], metadata['sequence']) and
            self.io.pause_targets and self.io.confirmed_stop_targets() == self.io.pause_targets)

    def new_session(self):
        self.envelopes = SourceEnvelope(new_session_id(self.node.now(), self.envelopes.session), self.node.now)
        self.io.source_motion = None
        self.sent = {}
        self.target_instance = None
        self.source_identity = None
        self.target_fault = ''
        self.applied_sequence = 0
        self.takeover_pending = None
        self.takeover_previous_hold = None

    def _settle_takeover(self, report=None, *, receipt=False):
        """Only takeover has a delivery barrier; normal targets never wait."""
        pending = self.takeover_pending
        if pending is None:
            return
        metadata, instance = pending
        _, current_servo = self.health.states.get('servo', (0., None))
        if current_servo is not None and current_servo.instance_id != instance:
            self.target_fault = 'SNOWBOT_TARGET_INSTANCE_CHANGED_RECONNECT_REQUIRED'
            return
        now_ns = round(self.node.now()*1e9)
        if now_ns >= metadata['expires_at_ns']:
            self.target_fault = 'SNOWBOT_TARGET_TAKEOVER_UNCONFIRMED_RECONNECT_REQUIRED'
            return
        if report is None or self.target_fault:
            return
        stamp = report.header.stamp.sec*1_000_000_000+report.header.stamp.nanosec
        if (report.header.stamp.sec >= 0 and 0 <= report.header.stamp.nanosec < 1_000_000_000 and
                getattr(report, 'feedback_boot_id', 0) == self.health.boot and
                report.instance_id == instance and getattr(report, 'session', 0) == metadata['session'] and
                getattr(report, 'sequence', 0) == metadata['sequence'] and
                metadata['source_stamp_ns'] <= stamp <= now_ns and
                report.wire_version == 5 and report.reference_mode == 2 and
                (receipt or report.selected and report.feedback_ready and report.startup_complete)):
            self.takeover_pending = None

    def advance_session(self):
        """Observe restart/rejection without gating subsequent target production."""
        _, servo = self.health.states.get('servo', (0., None))
        self._settle_takeover(servo)
        if servo is not None:
            if self.target_instance is not None and servo.instance_id != self.target_instance:
                self.target_fault = 'SNOWBOT_TARGET_INSTANCE_CHANGED_RECONNECT_REQUIRED'
            if (getattr(servo, 'rejected_session', 0) == self.envelopes.session and
                    getattr(servo, 'rejected_sequence', 0) == self.envelopes.sequence and
                    self.envelopes.sequence in self.sent):
                self.target_fault = 'SNOWBOT_TARGET_REJECTED:'+servo.reason

    def check_ready(self, groups):
        if self.io.sending_hold:
            # This is a stop-only request, never a takeover or target refresh.
            # Keep the known boot/instances and all endpoint/authority checks;
            # the servo still validates its local PDO and the stop trajectory.
            sent = self.sent.get(self.envelopes.sequence)
            servo = self.health.states.get('servo', (0., None))[1]
            middle = self.health.states.get('middle', (0., None))[1]
            if (sent is None or servo is None or middle is None or
                    servo.instance_id != sent[1] or
                    middle.instance_id != sent[0]['middle_instance_id'] or
                    self.health.boot != sent[0]['feedback_boot_id']):
                raise RuntimeError('SNOWBOT_PAUSE_AUTHORITY_UNAVAILABLE')
        else:
            self.check_health(groups, acquiring=True)
            for layer, (_, state) in self.health.states.items():
                if state.reason in ('command_source_discovering', 'command_source_conflict', 'legacy_source_conflict'):
                    raise RuntimeError('SNOWBOT_COMMAND_ENDPOINT:'+layer+':'+state.reason)

    def advance_startup_hold(self, engine):
        """One write-enabled startup, using the normal lease/command/ACK path.

        Never retry a granted lease or failed enable. Subsequent squeeze actions
        acquire following sessions; they do not initiate this startup action.
        Optional zero targets follow confirmed powered hold under the same lease.
        """
        from bindu_contracts.contracts import Motion
        from bindu_execution.executor import Rejected
        from dataclasses import replace
        state = self.startup_hold_state
        if state in ('disabled', 'ready', 'failed') or self.node.closing:
            return
        now = self.node.now()
        if engine.stop_fault or self.io.stop_failures:
            self.startup_hold_state = 'failed'
            self.node.event('STARTUP_HOLD_FAILED', engine.stop_fault or 'STOP_UNCONFIRMED')
            return
        if state == 'waiting':
            self.startup_timing = dict(points=[])
            def mark(phase):
                self.startup_timing['points'].append(dict(phase=phase,
                    steady=self.health.steady(), cpu=time.thread_time(),
                    ros=self.node.now(), pdo_sample=self.health.sample,
                    measurement_stamp=self.health.measurement_stamp))
            mark('acquire_begin')
            try:
                lease, epoch = engine.acquire('snowbot_startup_hold',
                    ('waist', 'left_arm', 'right_arm'), parameters_for(self,'execution')['startup_lease_ttl_s'], now)
            except (Rejected, RuntimeError):
                return  # Discovery/fresh measured feedback may not yet be ready.
            mark('acquired')
            self.startup_hold_state = 'takeover'
            names = tuple(n for group in ('waist', 'left_arm', 'right_arm') for n in GROUPS[group])
            try:
                measured = tuple(engine.feedback.positions[n] for n in names)
                tolerance = self.io.profile.feedback_tolerances['joint_position']
                positions = []
                for name, q in zip(names, measured):
                    lo, hi = self.io.profile.limits[name]
                    if not math.isfinite(q) or not lo-tolerance <= q <= hi+tolerance:
                        raise Rejected('STARTUP_HOLD_JOINT_LIMIT:'+name)
                    positions.append(min(hi, max(lo, q)))
                motion = Motion('startup_hold_'+lease, lease, epoch, self.io.profile.digest,
                    'startup_hold', '', 'joint_target', '', now, parameters_for(self,'execution')['startup_hold_ttl_s'], names=names,
                    positions=tuple(positions),
                    schema_version=2, resource_groups=('waist', 'left_arm', 'right_arm'), runtime_mode='real')
                mark('motion_ready')
                engine.submit(motion, now)
                mark('submitted')
                engine.tick(now)
                mark('first_tick')
                self.startup_motion = motion
            except (Rejected, RuntimeError) as exc:
                engine.halt(now, 'STARTUP_HOLD_FAILED', revoke=True)
                self.startup_hold_state = 'failed'
                self.node.event('STARTUP_HOLD_FAILED', str(exc))
                return
            self.node.event('STARTUP_HOLD_REQUESTED', 'BOUNDARY_REFERENCE' if tuple(positions)!=measured else 'MEASURED_POSITION',
                            command_id=motion.command_id)
        elif state == 'takeover':
            if engine.stopping or self.target_fault or engine.active is None:
                self.startup_hold_state = 'failed'
                self.node.event('STARTUP_HOLD_FAILED', self.target_fault or 'TAKEOVER_NOT_CONFIRMED')
            elif self.takeover_pending is None and self.applied_sequence:
                home = getattr(self, 'startup_home', False)
                engine.halt(now, revoke=not home)
                self.startup_hold_state = 'holding' if home else 'stopping'
        elif state == 'holding' and not engine.stopping:
            self.startup_home_deadline = now+self.io.profile.max_transition_duration
            self.startup_hold_state = 'homing'
            self.node.event('STARTUP_HOME_REQUESTED', 'ARMS_WAIST_ZERO')
        elif state == 'homing':
            try:
                if now >= self.startup_home_deadline:
                    raise Rejected('STARTUP_HOME_TIMEOUT')
                if engine.stopping or self.target_fault:
                    raise Rejected(self.target_fault or 'STARTUP_HOME_INTERRUPTED')
                motion = replace(self.startup_motion, command_id='startup_home_'+str(round(now*1e9)),
                    stamp=now, positions=(0.,)*len(self.startup_motion.names))
                engine.renew(motion.lease_id, motion.epoch, now)
                engine.submit(motion, now)
                engine.tick(now)
                reference = self.io.sampled_reference()[1]
                tolerance = self.io.profile.feedback_tolerances['joint_position']
                arrival = dict(self.io.replay_position_tolerances)
                arrival.update({n: self.io.profile.stop_position_tolerances.get(n, tolerance)
                                for n in self.io.stop_arrival_joints})
                if (self.takeover_pending is None and engine.references_arrived(motion.names,
                        position_tolerances=arrival) and
                        all(abs(reference.get(n, math.inf)) < tolerance for n in motion.names)):
                    engine.halt(now, revoke=True)
                    self.startup_hold_state = 'stopping'
            except (Rejected, RuntimeError) as exc:
                engine.halt(now, 'STARTUP_HOME_FAILED', revoke=True)
                self.startup_hold_state = 'failed'
                self.node.event('STARTUP_HOLD_FAILED', str(exc))
        elif state == 'stopping' and not engine.stopping:
            self.startup_hold_state = 'ready'
            self.node.event('STARTUP_HOLD_READY', 'POWERED_HOLD')
            self.node.get_logger().info('SNOWBOT_STARTUP_HOLD_READY: powered hold; squeeze controls following')

    def brake_before_source_expiry(self, engine):
        """Request PAUSE before a lost target producer's original TTL expires."""
        if not engine.active or engine.stopping:
            return
        sent = self.sent.get(self.envelopes.sequence)
        lead_ns = round(self.runtime_parameters['execution']['target_stream_brake_lead_s']*1e9)
        if sent and sent[0]['operation'] != 2:
            # Keep the ordinary execution path to one tick. The extra dispatch
            # is needed only when the preceding packet is about to be fenced.
            if round(self.node.now()*1e9) < sent[0]['expires_at_ns']-lead_ns:
                return
            # A service can accept a replacement between executor ticks. Dispatch
            # it through the normal feedback/lease/limit checks before fencing
            # the old packet. Only an actual new envelope changes the deadline;
            # a pending takeover, failed send or repeated tick cannot renew it.
            if engine.active.command_id != sent[2]:
                engine.tick(self.node.now())
                if not engine.active or engine.stopping:
                    return
                sent = self.sent.get(self.envelopes.sequence)
                if not sent or sent[0]['operation'] == 2:
                    return
            # The configured lead reserves transport/dispatch, preparation and
            # scheduling time before the original target deadline.
            # A stalled controller still falls under the native watchdog.
            if round(self.node.now()*1e9) >= sent[0]['expires_at_ns']-lead_ns:
                engine.halt(self.node.now(), 'COMMAND_TIMEOUT', revoke=True)

    def send(self, groups, target):
        # This adapter has one atomic publisher. Before that call, failure is
        # provably unsent; once called, even an exception leaves delivery unknown.
        self.publish_attempted = False
        try:
            return self._send_packet(groups, target)
        except RuntimeError as exc:
            if not self.publish_attempted:
                raise CommandNotSent(str(exc)) from exc
            raise

    def _send_packet(self, groups, target):
        capture = not self.sent or self.io.sending_hold
        if capture:
            if not self.sent:self.send_timing = {}
            trace = dict(points=[])
            self.send_timing['pause' if self.io.sending_hold else 'takeover'] = trace
        def mark(phase):
            if capture:
                usage = resource.getrusage(resource.RUSAGE_THREAD)
                trace['points'].append(dict(phase=phase, steady_ns=time.monotonic_ns(),
                    cpu_ns=time.thread_time_ns(), nvcsw=usage.ru_nvcsw, nivcsw=usage.ru_nivcsw))
        mark('begin')
        # Validate before minting an envelope, so rejected requests cannot renew.
        self._settle_takeover()
        self.check_ready(groups)
        mark('ready_checked')
        if set(target) != set(CONTROL_NAMES) or any(not math.isfinite(target[n]) or
                not self.io.profile.limits[n][0] <= target[n] <= self.io.profile.limits[n][1]
                for n in CONTROL_NAMES):
            raise RuntimeError('SNOWBOT_TARGET_LAYOUT_OR_LIMIT')
        identity = None
        motion = self.io.source_motion
        # PAUSE may follow revocation/clearing of the Motion. Retain the last
        # source identity in this session; stopping never invents ownership.
        identity = ((motion.command_id, motion.lease_id, motion.epoch, self.io.profile.digest)
                    if motion is not None else self.source_identity if self.io.sending_hold else None)
        if identity is None:
            raise RuntimeError('SNOWBOT_SOURCE_IDENTITY_LAYOUT')
        if self.takeover_pending is not None and not self.io.sending_hold:
            return  # Retain the latest original Motion in IO; no identity/TTL renewal.
        metadata = self.envelopes.command(self.io.source_motion, pause=self.io.sending_hold)
        if metadata is None:
            return
        metadata.update(middle_instance_id=self.health.states['middle'][1].instance_id,
                        feedback_boot_id=self.health.boot, feedback_sample=self.health.sample)
        msg = self.arm_type()
        for name, value in metadata.items():
            setattr(msg.meta, name, value)
        self.source_identity = identity
        msg.meta.reference_mode = 2
        msg.meta.source_command_id, msg.meta.lease_id, msg.meta.epoch, msg.meta.profile_hash = identity
        # Version 2 is a target recipe, never a curve. The servo owns its
        # local switching time; empty curve fields reject legacy packets.
        msg.trajectory.version = (3 if not self.io.sending_hold and
                                  getattr(self.io.source_motion,'rigid_reference',False) else 2)
        msg.trajectory.segment_id = metadata['sequence']
        prepared = getattr(self.io, 'prepared_replay', None)
        if prepared is not None and not self.io.sending_hold:
            msg.trajectory.version = 4 if prepared.inner_reference else 5
            msg.trajectory.effective_at_ns = round(prepared.effective*1e9)
            if self.io.source_motion.mode != 'trajectory_heartbeat':
                self.replay_segment_id = metadata['sequence']
                msg.trajectory = self.replay_wire
                msg.trajectory.effective_at_ns = round(prepared.effective*1e9)
            msg.trajectory.segment_id = self.replay_segment_id
        for field, getter, ceiling in (('max_velocity', self.io.profile.speed, self.trajectory_limits['trajectory_max_velocity']),
                ('max_acceleration', self.io.profile.acceleration, self.trajectory_limits['trajectory_max_acceleration']),
                ('max_jerk', self.io.profile.jerk, self.trajectory_limits.get('trajectory_max_jerk', 162.))):
            if getattr(self, 'startup_hold_state', '') == 'homing':
                ceiling = min(ceiling, TARGET_STREAM_LIMIT_DEFAULTS['trajectory_'+field])
            limits = [min(ceiling, getter(n)) for n in CONTROL_NAMES]
            if getattr(self, 'startup_hold_state', '') == 'homing' and field == 'max_velocity':
                waist_speed = parameters_for(self, 'execution')['startup_home_waist_max_velocity_rad_s']
                limits = [min(value, waist_speed) if n in GROUPS['waist'] else value
                          for n, value in zip(CONTROL_NAMES, limits)]
            setattr(msg.trajectory, field, limits)
        msg.command.l_arm_position = [float(target[n]) for n in GROUPS['left_arm']]
        msg.command.r_arm_position = [float(target[n]) for n in GROUPS['right_arm']]
        msg.waist_position = [float(target[n]) for n in GROUPS['waist']]
        instance = self.health.states['servo'][1].instance_id
        self.target_instance = instance
        if metadata['operation'] == 1:
            self.takeover_pending = (dict(metadata), instance)
            obs = self.health.observation
            self.takeover_previous_hold = (
                (obs.boot_id, obs.session, obs.command_sequence, instance, metadata['expires_at_ns'], obs.session_reason)
                if obs.session != metadata['session'] and (obs.session_reason >= 2 or
                obs.session_selected and obs.session_reason == 1 and obs.hold_valid and
                getattr(obs, 'trajectory_phase', 0) == 4 and
                getattr(obs, 'reference_valid', False) and getattr(obs, 'output_valid', False))
                else None)
        elif metadata['operation'] == 2:
            self.takeover_pending = None
        self.sent[metadata['sequence']] = (metadata, instance, msg.meta.source_command_id)
        while len(self.sent) > 32:
            del self.sent[next(iter(self.sent))]
        self.publish_attempted = True
        mark('packet_ready')
        self.publishers['arms'].publish(msg)
        mark('published')
        mapping = {k: metadata[k] for k in ('session', 'sequence', 'operation', 'source_stamp_ns', 'expires_at_ns')}
        mapping.update(lease_id=msg.meta.lease_id, epoch=msg.meta.epoch,
                       execution_mode=self.health.execution_mode, reference_mode=2,
                       target_recipe_version=msg.trajectory.version,
                       rigid_reference=msg.trajectory.version==3)
        self.node.event('TARGET_SENT', 'SNOWBOT_TARGET:'+json.dumps(mapping, separators=(',', ':')),
                        command_id=msg.meta.source_command_id)
        mark('event_published')
        if self.io.sending_hold:
            self.io.pause_request = (metadata['session'], metadata['sequence'],
                                     self.health.boot, self.health.sample)
            self.io.pause_targets = None

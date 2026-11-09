"""Independent position-session contract. No ROS, devices or implicit renewal."""
from collections import deque
import math
import time

from .protocol import GROUPS, SLOT_MASKS

ARMS = frozenset(('left_arm', 'right_arm'))
CONTROL_GROUPS = ARMS | {'waist'}
# Independent wire order; QP/resource order is mapped by joint name.
CONTROL_NAMES = (*GROUPS['left_arm'], *GROUPS['right_arm'], *GROUPS['waist'])
CONTROL_MASK = sum(SLOT_MASKS[g] for g in CONTROL_GROUPS)
# ROS delivery can lag the local PDO sampler. Keep these budgets independent.
ROS_FEEDBACK_TIMEOUT_S = .04
PDO_MAX_AGE_S = .02
SENSOR_MAX_AGE_S = .1
TARGET_STREAM_MAX_TTL_S = 1.5
SLOTS = [*(f'arm_l_joint_{i}' for i in range(1, 9)),
         *(f'arm_r_joint_{i}' for i in range(1, 9)),
         'head_joint_1', 'head_joint_2', 'body_joint_1', 'body_joint_2']


# Joint-task order; EtherCAT wire order remains CONTROL_NAMES.
REPLAY_GROUPS = ('waist', 'left_arm', 'right_arm')
REPLAY_NAMES = ('body_joint_1', 'body_joint_2',
                *(f'arm_{side}_joint_{i}' for side in ('l', 'r') for i in range(1, 8)))


def checked_execution_state(msg, now, received, steady, profile_hash, *, owned=False, epoch=0, stationary=True,
                  state_max_age=.1, feedback_max_age=.2):
    feedback_age = now-(msg.feedback_stamp.sec+msg.feedback_stamp.nanosec*1e-9)
    if (msg.runtime_mode != 'real' or msg.simulated or msg.device_backend != 'snowbot_vendor' or
            msg.profile_hash != profile_hash or msg.stop_failures or
            not 0 <= steady-received <= state_max_age or
            not 0 <= feedback_age <= feedback_max_age or
            msg.state in ('FAILED', 'STOPPING')):
        raise RuntimeError('EXECUTION_NOT_READY:state='+msg.state+';code='+msg.code+
                           ';feedback_age='+str(round(feedback_age, 4))+
                           ';state_age='+str(round(steady-received, 4))+
                           ';profile_match='+str(msg.profile_hash == profile_hash)+
                           ';stop_failures='+str(list(msg.stop_failures)))
    if (msg.lease_active != owned or (owned and
            (msg.lease_epoch != epoch or set(msg.lease_resources) != set(REPLAY_GROUPS)))):
        raise RuntimeError('EXECUTION_OWNERSHIP_CHANGED')
    if len(msg.joints.name) != len(set(msg.joints.name)) or not (
            len(msg.joints.name) == len(msg.joints.position) == len(msg.joints.velocity)):
        raise RuntimeError('MEASUREMENT_LAYOUT')
    q, v = dict(zip(msg.joints.name, msg.joints.position)), dict(zip(msg.joints.name, msg.joints.velocity))
    if any(n not in q or not all(math.isfinite(x) for x in (q[n], v[n])) for n in REPLAY_NAMES):
        raise RuntimeError('MEASUREMENT_MISSING_OR_NONFINITE')
    if stationary and not owned and any(abs(v[n]) > .01 for n in REPLAY_NAMES):
        raise RuntimeError('MEASUREMENT_NOT_STATIONARY')
    return q, v


def decode_observation(msg, now):
    if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
        raise ValueError('SNOWBOT_OBSERVATION_STAMP')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if (not msg.boot_id or not msg.pdo_sample or
            not 0 <= now-stamp <= ROS_FEEDBACK_TIMEOUT_S or
            not 0 <= msg.pdo_age <= PDO_MAX_AGE_S or
            now-stamp+msg.pdo_age > ROS_FEEDBACK_TIMEOUT_S):
        raise ValueError('SNOWBOT_PDO_STALE')
    if msg.expected_wkc <= 0 or msg.wkc != msg.expected_wkc:
        raise ValueError('SNOWBOT_PDO_WKC')
    q, v = {}, {}
    for i, name in enumerate(SLOTS):
        if msg.valid_mask & (1 << i):
            if not all(math.isfinite(x) for x in (msg.position[i], msg.speed[i], msg.current[i])):
                raise ValueError('SNOWBOT_OBSERVATION_NONFINITE')
            q[name], v[name] = msg.position[i], msg.speed[i]
    return q, v, stamp-msg.pdo_age


class PdoDeliveryWindow:
    """Timing for samples taken from the same depth-one subscription, on one executor."""
    def __init__(self):
        self.history = deque(maxlen=8)
        self.taken = None
        self.window = None
        self.deadline = 0.

    def callback(self, msg, received_ros, entered, callback_system_ns):
        # A take and its callback run serially on this executor.
        taken, self.taken = self.taken, None
        info, taken_ns = taken[1:] if taken is not None and taken[0] is msg else ({}, None)
        stamp = msg.header.stamp
        source_ns = stamp.sec*1_000_000_000 + stamp.nanosec
        record = dict(boot_id=msg.boot_id, pdo_sample=msg.pdo_sample, header_ros_ns=source_ns,
                      measurement_ros_ns=source_ns-round(msg.pdo_age*1e9) if math.isfinite(msg.pdo_age) else None,
                      dds_source_ns=info.get('source_timestamp') or None,
                      dds_received_ns=info.get('received_timestamp') or None,
                      take_system_ns=taken_ns, callback_system_ns=callback_system_ns, callback_ros=received_ros)
        self.history.append(record)
        if self.window is not None and entered < self.deadline and len(self.window['after']) < 3:
            self.window['after'].append(record)

    def trigger(self, detail, now):
        if self.window is None:
            self.window = dict(trigger=detail, trigger_steady_ns=round(now*1e9), before=list(self.history), after=[])
            self.deadline = now + .1
        elif detail.get('code') == 'SNOWBOT_PDO_NOT_ADVANCING' and self.window['trigger'].get('code') != detail['code']:
            self.window.setdefault('progress_interruption', detail)

    def finish(self, now):
        if self.window is None or (len(self.window['after']) < 3 and now < self.deadline):
            return None
        window, self.window = self.window, None
        window['end_reason'] = 'three_samples' if len(window['after']) == 3 else 'timeout'
        return window


class SessionHealth:
    def __init__(self, now, steady=time.monotonic, *, execution_mode="position", on_stale=None):
        self.execution_mode = execution_mode
        self.reference_mode = 2
        self.now, self.steady = now, steady
        self.states, self.received = {}, {}
        self.observation = None
        self.on_stale = on_stale
        self.callback_received_ros = None
        self.rejected_stale_observation = None
        self.stale_read_reported = False
        self.measurement_stamp = 0.
        self.boot = self.sample = 0
        self.progress = None
        self.progress_gap_reported = False
        self.invalid = ''
        self.messages = {'servo': {}}
        self.observation_callback_count = 0
        self.observation_callbacks = deque(maxlen=8)

    def observation_diagnostic(self):
        """Read-only timing evidence; format the bounded history only on failure."""
        now, steady = self.now(), self.steady()
        obs = self.observation
        return dict(now_ros=now, now_steady=steady, limit_ms=ROS_FEEDBACK_TIMEOUT_S*1000,
            pdo_sample=self.sample, progress_age_ms=None if self.progress is None else (steady-self.progress)*1000,
            source_age_ms=None if obs is None else (now-(obs.header.stamp.sec+obs.header.stamp.nanosec*1e-9))*1000,
            measurement_age_ms=None if obs is None else (now-self.measurement_stamp)*1000,
            pdo_age_ms=None if obs is None else obs.pdo_age*1000,
            nominal_reference_age_ms=None if obs is None or not getattr(obs, 'sample_time_ns', 0) else (now-obs.sample_time_ns*1e-9)*1000,
            reference_valid=None if obs is None else getattr(obs, 'reference_valid', None),
            output_valid=None if obs is None else getattr(obs, 'output_valid', None),
            wkc=None if obs is None else obs.wkc, expected_wkc=None if obs is None else obs.expected_wkc,
            invalid=self.invalid, callback_count=self.observation_callback_count,
            callback_receive_age_ms=None if not self.observation_callbacks else (steady-self.observation_callbacks[-1][0])*1000,
            callbacks=[dict(entered=t[0], finished=t[1], pdo_sample=t[2], source_stamp=t[3],
                pdo_age_ms=t[4]*1000, identity_ms=t[5]*1000, duration_ms=(t[1]-t[0])*1000,
                source_age_at_entry_ms=t[6]*1000, outcome=t[7],
                gap_ms=None if i==0 else (t[0]-self.observation_callbacks[i-1][0])*1000)
                for i,t in enumerate(self.observation_callbacks)])

    def _report_stale(self, stage, msg, checked_ros):
        if self.on_stale is None:
            return
        source = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.on_stale(dict(stage=stage, boot_id=msg.boot_id, pdo_sample=msg.pdo_sample,
            source_stamp_ros=source, measurement_stamp_ros=source-msg.pdo_age,
            callback_received_ros=self.callback_received_ros, checked_ros=checked_ros,
            publish_age_ms=(checked_ros-source)*1000, pdo_age_ms=msg.pdo_age*1000,
            previous_sample=self.sample, wkc=msg.wkc, expected_wkc=msg.expected_wkc))

    def observe(self, msg, *, received_ros=None):
        checked_ros = self.now()
        self.callback_received_ros = checked_ros if received_ros is None else received_ros
        self.rejected_stale_observation = None
        self.stale_read_reported = False
        try:
            result = decode_observation(msg, checked_ros)
            if self.boot and msg.boot_id != self.boot:
                raise ValueError('SNOWBOT_SERVO_RESTART_RECONNECT_REQUIRED')
            if msg.pdo_sample < self.sample:
                raise ValueError('SNOWBOT_PDO_SEQUENCE')
            if msg.pdo_sample > self.sample:
                self.progress = self.steady()
                self.progress_gap_reported = False
            self.boot, self.sample = msg.boot_id, msg.pdo_sample
            self.observation = msg
            self.measurement_stamp = result[2]
            self.invalid = ''
            self.messages['servo'] = dict(operation_enabled_mask=sum(
                1 << i for i, status in enumerate(msg.status) if status & 4))
            return result
        except ValueError as exc:
            self.invalid = str(exc)
            if self.invalid == 'SNOWBOT_PDO_STALE':
                self.rejected_stale_observation = msg
                self._report_stale('receive', msg, checked_ros)
            raise

    def state(self, layer, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        if (msg.layer != layer or msg.header.stamp.sec < 0 or
                not 0 <= msg.header.stamp.nanosec < 1_000_000_000 or
                not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(now) or now < stamp):
            raise ValueError('SNOWBOT_SESSION_STATE_INVALID:'+layer)
        previous = self.states.get(layer)
        if previous is not None and stamp <= previous[0]:
            return False
        if now-stamp > .2:
            raise ValueError('SNOWBOT_SESSION_STATE_INVALID:'+layer)
        self.states[layer] = stamp, msg
        self.received[layer] = self.steady()
        return True

    def confirmed_hold(self, request):
        """Return the adopted fixed reference, never physical-stop confirmation.

        A trajectory impedance PAUSE adopts a fixed nominal reference and keeps
        its controller. Executor confirms measured rest separately from arrival.
        """
        if self.hold_failure(request):
            return None
        return dict(zip(CONTROL_NAMES, self.observation.hold_position))

    def hold_failure(self, request):
        """Explain the same ACK gate without changing state or publishing."""
        if request is None:
            return 'PAUSE_REQUEST_MISSING'
        try:
            self.require(CONTROL_GROUPS, acquiring=True)
        except RuntimeError as exc:
            return str(exc)
        obs = self.observation
        session, sequence, boot, baseline = request
        # A disabled/interrupted session cannot acknowledge a braking PAUSE.
        # Report the device barrier before the incidental sequence mismatch.
        if obs.session_reason >= 3:
            return 'SESSION_TERMINATED:'+str(obs.session_reason)
        checks = ((obs.boot_id == boot, 'BOOT_MISMATCH'),
                  (obs.session_selected, 'SESSION_NOT_SELECTED'),
                  (obs.session == session, 'SESSION_MISMATCH'),
                  (obs.command_sequence == sequence, 'PAUSE_SEQUENCE_MISMATCH'),
                  (obs.session_reason in (1, 2), 'PAUSE_REASON_INVALID'),
                  (getattr(obs, 'hold_valid', False), 'HOLD_NOT_VALID'),
                  (baseline <= obs.hold_sample < obs.pdo_sample, 'HOLD_SAMPLE_NOT_ADVANCED'))
        checks += ((getattr(obs, 'reference_mode', 0) == 2, 'REFERENCE_MODE_MISMATCH'),
                   (getattr(obs, 'segment_id', 0) == sequence, 'SEGMENT_ID_MISMATCH'),
                   (obs.session_reason == 1, 'PAUSE_EXPIRED_OR_REVOKED'),
                   (getattr(obs, 'trajectory_phase', 0) == 4, 'TRAJECTORY_NOT_PAUSED'),
                   (getattr(obs, 'hold_basis', -1) == 0, 'HOLD_BASIS_MISMATCH'),
                   (getattr(obs, 'reference_valid', False), 'REFERENCE_NOT_VALID'),
                   (getattr(obs, 'output_valid', False), 'OUTPUT_NOT_VALID'))
        for passed, reason in checks:
            if not passed:
                return reason
        values = obs.hold_position
        if not all(math.isfinite(q) for q in values):
            return 'HOLD_POSITION_INVALID'
        return ''

    def require_observation(self):
        """Use the decoded PDO snapshot; only its freshness changes between reads."""
        if self.invalid:
            if self.rejected_stale_observation is not None and not self.stale_read_reported:
                self.stale_read_reported = True
                self._report_stale('read_rejected', self.rejected_stale_observation, self.now())
            raise RuntimeError(self.invalid)
        checked_steady = self.steady()
        if self.progress is None or not 0 <= checked_steady-self.progress <= ROS_FEEDBACK_TIMEOUT_S:
            if self.on_stale is not None and not self.progress_gap_reported:
                self.progress_gap_reported = True
                self.on_stale(dict(code='SNOWBOT_PDO_NOT_ADVANCING', stage='read_progress',
                    boot_id=self.boot or None, pdo_sample=self.sample or None,
                    measurement_stamp_ros=self.measurement_stamp if self.observation is not None else None,
                    callback_received_ros=self.callback_received_ros, last_progress_steady=self.progress,
                    checked_steady=checked_steady, checked_ros=self.now()))
            raise RuntimeError('SNOWBOT_PDO_NOT_ADVANCING')
        obs = self.observation
        checked_ros = self.now()
        if not (obs.header.stamp.sec+obs.header.stamp.nanosec*1e-9 <= checked_ros <=
                self.measurement_stamp+ROS_FEEDBACK_TIMEOUT_S):
            if not self.stale_read_reported:
                self.stale_read_reported = True
                self._report_stale('read_cached', obs, checked_ros)
            raise RuntimeError('SNOWBOT_PDO_STALE')
        return obs

    def require(self, groups, *, acquiring=False):
        obs = self.require_observation()
        if any(status & 8 for i, status in enumerate(obs.status) if CONTROL_MASK & (1 << i)):
            raise RuntimeError('SNOWBOT_DRIVE_FAULT')
        if obs.valid_mask & CONTROL_MASK != CONTROL_MASK:
            raise RuntimeError('SNOWBOT_POSITION_FEEDBACK_INVALID')
        for layer in ('upper', 'middle', 'servo'):
            stamp, state = self.states.get(layer, (0., None))
            if (state is None or not 0 <= self.now()-stamp <= .2 or
                    not 0 <= self.steady()-self.received[layer] <= .2 or not state.startup_complete or
                    not state.feedback_ready or not state.instance_id):
                raise RuntimeError('SNOWBOT_SESSION_LAYER_UNAVAILABLE:'+layer)
            if layer == 'upper' and state.control_source != 'topic':
                raise RuntimeError('SNOWBOT_CONTROL_SOURCE_NOT_TOPIC')
            if layer == 'servo' and state.stop_policy not in ('disable_no_load', 'measured_hold'):
                raise RuntimeError('SNOWBOT_STOP_POLICY_UNCONFIRMED')
            if layer == 'servo' and state.control_source != self.execution_mode:
                raise RuntimeError('SNOWBOT_EXECUTION_MODE_CONFLICT:'+self.execution_mode)
            if getattr(state, 'wire_version', 0) != 5:
                raise RuntimeError('SNOWBOT_SESSION_PROTOCOL_CONFLICT:'+layer)
            if layer == 'servo' and getattr(state, 'reference_mode', 0) != 2:
                raise RuntimeError('SNOWBOT_TRAJECTORY_PROTOCOL_CONFLICT:'+layer)
        if self.execution_mode == 'impedance':
            # The optional vendor build places force data in this exact PDO sample.
            if obs.valid_mask != 0xfffff:
                raise RuntimeError('SNOWBOT_POSITION_FEEDBACK_INVALID')
            wrench = getattr(obs, 'wrench', ())
            age = getattr(obs, 'sensor_age', ())
            # ROS delivery age is already checked by require_observation().
            if (getattr(obs, 'sensor_valid_mask', 0) != 3 or
                    not all(math.isfinite(x) for x in wrench) or
                    not all(math.isfinite(a) and 0 <= a <= SENSOR_MAX_AGE_S for a in age)):
                # The RT watchdog owns consecutive-PDO confirmation during a session.
                if acquiring or not obs.session_selected or obs.session_reason >= 3:
                    raise RuntimeError('SNOWBOT_IMPEDANCE_SENSOR_INVALID')
        if obs.session_reason >= 2 and not acquiring:
            raise RuntimeError('SNOWBOT_SESSION_STOPPED:'+str(obs.session_reason))


def stop_diagnostic(health, engine, now):
    """Read-only stop-gate evidence, separate from any acceptance decision."""
    if not engine.stopping and not engine.stop_fault:
        return None
    io, fb, obs = engine.io, engine.feedback, health.observation
    request = getattr(io, 'pause_request', None)
    session, sequence, boot, baseline = request or (None, None, None, None)
    ack_reason = health.hold_failure(request)
    names = tuple(engine.reference_names) or CONTROL_NAMES
    tolerance = engine.profile.feedback_tolerances
    position_tolerance = tolerance.get('joint_position', 1e-5)
    velocity_tolerance = tolerance.get('joint_velocity', .01)
    hold = getattr(obs, 'hold_position', ())
    targets = dict(zip(CONTROL_NAMES, hold)) if len(hold) == len(CONTROL_NAMES) else {}
    errors = {n: abs(fb.positions[n]-targets[n]) for n in names
              if n in fb.positions and n in targets and math.isfinite(fb.positions[n]) and
              math.isfinite(targets[n])}
    speeds = {n: abs(fb.joint_velocities[n]) for n in names
              if n in fb.joint_velocities and math.isfinite(fb.joint_velocities[n])}
    position_error = max(errors.values()) if errors else None
    arrival_names = names if getattr(io, 'stop_requires_arrival', True) else tuple(getattr(io, 'stop_arrival_joints', ()))
    arrival_errors = {n: errors[n] for n in arrival_names if n in errors}
    arrival_error = max(arrival_errors.values()) if arrival_errors else None
    overrides = getattr(engine.profile, 'stop_position_tolerances', {})
    arrival_tolerances = {n: overrides.get(n, position_tolerance) for n in arrival_names}
    max_speed = max(speeds.values()) if speeds else None
    fresh = fb.stamp > io.stop_requested_at and 0 <= now-fb.stamp <= .2
    steady = engine.steady()
    reason = ack_reason
    if not reason:
        if any(not engine.profile.limits[n][0] <= q <= engine.profile.limits[n][1]
               for n, q in targets.items()):
            reason = 'HOLD_OUTSIDE_LIMITS'
        elif getattr(io, 'pause_targets', None) is not None and targets != io.pause_targets:
            reason = 'HOLD_POINT_CHANGED'
        elif not fresh:
            reason = 'STOP_FEEDBACK_NOT_FRESH'
        elif len(errors) != len(names) or len(speeds) != len(names):
            reason = 'STOP_FEEDBACK_INCOMPLETE'
        elif not set(arrival_names) <= set(names):
            reason = 'STOP_REFERENCE_INVALID'
        elif any(e >= arrival_tolerances[n] for n, e in arrival_errors.items()):
            reason = 'STOP_POSITION_ERROR'
        elif max_speed > velocity_tolerance:
            reason = 'STOP_SPEED_NOT_ZERO'
        else:
            reason = 'WAIT_REST_WINDOW' if getattr(io, 'stop_confirmation_window', 0.) else 'WAIT_EXECUTOR'
    if engine.stop_fault:
        reason = engine.stop_fault
    elif now >= engine.stop_deadline or steady >= engine.stop_steady_deadline:
        reason = 'STOP_BUDGET_EXPIRED'
    window = getattr(engine, 'stop_rest_window', None)
    device_fault = bool(obs and getattr(obs, 'session_reason', 0) >= 3)
    servo = health.states.get('servo', (None, None))[1]
    return dict(reason=reason, ack_reason=ack_reason, session=session, sequence=sequence,
        recovery_required=('device_inspection' if device_fault else
                           'stop_confirmation' if ack_reason or engine.stop_fault else 'measured_rest'),
        servo_rejection={key:getattr(servo,key,None) for key in
                         ('rejected_session','rejected_sequence','reason')},
        boot_id=boot, baseline_sample=baseline,
        observed={key:(value.tolist() if hasattr(value, 'tolist') else value)
            for key in ('boot_id', 'pdo_sample', 'session',
            'command_sequence', 'session_reason', 'session_selected', 'hold_valid', 'hold_sample',
            'hold_basis', 'trajectory_phase', 'reference_valid', 'output_valid',
            'wkc', 'expected_wkc', 'valid_mask', 'pdo_age',
            'sensor_valid_mask', 'sensor_data_no', 'sensor_age')
            for value in (getattr(obs, key, None),)},
        controlled_mask=CONTROL_MASK,
        operation_enabled_mask=health.messages['servo'].get('operation_enabled_mask'),
        stop_requested_at=io.stop_requested_at, feedback_stamp=fb.stamp, feedback_fresh=fresh,
        stop_deadline=engine.stop_deadline, remaining_ros_s=engine.stop_deadline-now,
        remaining_steady_s=engine.stop_steady_deadline-steady,
        max_position_error_rad=position_error, max_speed_rad_s=max_speed,
        position_error_basis='observed_hold_position',
        arrival_required=bool(arrival_names), arrival_required_joints=list(arrival_names),
        required_reference_arrived=bool(len(arrival_errors)==len(arrival_names) and
            all(e < arrival_tolerances[n] for n, e in arrival_errors.items())),
        max_required_position_error_rad=arrival_error,
        worst_required_position_joint=max(arrival_errors, key=arrival_errors.get) if arrival_errors else None,
        reference_arrived=bool(len(errors)==len(names) and position_error < position_tolerance),
        worst_position_joint=max(errors, key=errors.get) if errors else None,
        position_tolerance_rad=position_tolerance,
        arrival_position_tolerances_rad=arrival_tolerances,
        velocity_tolerance_rad_s=velocity_tolerance,
        rest_window_required_s=getattr(io, 'stop_confirmation_window', 0.),
        rest_window_elapsed_s=max(0., steady-window[0]) if window else None,
        stop_failures=list(io.stop_failures))


def new_session_id(now, previous=0):
    """Ordered uint64 authority identity; time rollback never wraps/reuses it.

    Across a source restart ROS time must advance past the receiver watermark;
    otherwise admission fails closed. No receiver clock or TTL is rewritten.
    """
    if not math.isfinite(now) or now <= 0:
        raise ValueError('SNOWBOT_SESSION_CLOCK_INVALID')
    value = max(round(now*1e9), previous+1)
    return value


class SourceEnvelope:
    """One envelope per accepted Motion; executor ticks cannot renew a target."""
    def __init__(self, session, now):
        self.session, self.now = session, now
        # A new PAUSE authorizes only braking. Its original deadline is fixed
        # here, independently of the ordinary target's source deadline.
        self.pause_valid_for = 2.
        self.sequence = 0
        self.last_id = None
        self.takeover = True

    @staticmethod
    def validate_motion(motion, now):
        """Check the same effective source deadline before software acceptance."""
        maximum = TARGET_STREAM_MAX_TTL_S
        expires = motion.stamp + min(maximum, motion.valid_for)
        if (not all(math.isfinite(value) for value in (motion.stamp, motion.valid_for, now)) or
                motion.valid_for <= 0 or not motion.stamp <= now < expires):
            raise RuntimeError('SNOWBOT_SOURCE_COMMAND_EXPIRED')
        return expires

    def command(self, motion, *, pause=False):
        if not pause and motion is None:
            raise RuntimeError('SNOWBOT_SOURCE_MOTION_REQUIRED')
        if not pause and motion.command_id == self.last_id:
            return None
        source = self.now() if pause else motion.stamp
        expires = source+self.pause_valid_for if pause else self.validate_motion(
            motion, self.now())
        if not source <= self.now() < expires:
            raise RuntimeError('SNOWBOT_SOURCE_COMMAND_EXPIRED')
        self.sequence += 1
        operation = 2 if pause else 1 if self.takeover else 0
        self.takeover = pause
        if not pause:
            self.last_id = motion.command_id
        source_ns = round(source*1e9)
        maximum = TARGET_STREAM_MAX_TTL_S
        ttl_ns = (round(self.pause_valid_for*1e9) if pause else
                  min(round(maximum*1e9), round(motion.valid_for*1e9)))
        return dict(session=self.session, sequence=self.sequence,
                    source_stamp_ns=source_ns, expires_at_ns=source_ns+ttl_ns,
                    operation=operation)

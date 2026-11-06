"""Snowbot position/feedback boundary; no ROS or trajectory generation.

Layout follows the user supplied src.zip (2026-10-13). J8 is a gripper,
read as context only. Arm packets always carry both arms, so their ownership
must be acquired together. No base/odometry interface is inferred.
"""
import math
import time

from bindu_contracts.contracts import Feedback


class CommandNotSent(RuntimeError):
    """A single-packet adapter proved failure before calling its transport.

    Transport-call failures remain unknown and must retain a stop footprint.
    """


class GraphSnapshot:
    """Refresh endpoint discovery at 10 Hz, independently of packet frequency."""
    def __init__(self, node, steady=time.monotonic):
        self.node, self.steady = node, steady
        self.entries = {}

    def get_publishers_info_by_topic(self, topic):
        now = self.steady()
        previous = self.entries.get(topic)
        if previous is None or now-previous[0] >= .1:
            self.entries[topic] = now, self.node.get_publishers_info_by_topic(topic)
        return self.entries[topic][1]


GROUPS = {
    'waist': ['body_joint_1', 'body_joint_2'],
    'head': ['head_joint_1', 'head_joint_2'],
    'left_arm': [f'arm_l_joint_{i}' for i in range(1, 8)],
    'right_arm': [f'arm_r_joint_{i}' for i in range(1, 8)],
    'left_tool_axis': ['arm_l_joint_8'],
    'right_tool_axis': ['arm_r_joint_8'],
}
WRITABLE = frozenset(('waist', 'head', 'left_arm', 'right_arm'))
SLOT_MASKS = {'left_arm': 0x7f, 'right_arm': 0x7f00, 'head': 0x30000, 'waist': 0xc0000}
SLOT_MASKS.update(left_tool_axis=0x80, right_tool_axis=0x8000)


class SnowbotIO:
    def __init__(self, profile, send, now, *, command_enabled=False,
                 simulated=False, feedback_max_age=.2, check_ready=None, check_health=None):
        if (profile.groups != GROUPS or set(profile.capabilities) != {'joint_position'} or
                profile.name not in ('snowbot_vendor_position','snowbot_real_position')):
            raise ValueError('SNOWBOT_PROFILE_REQUIRED')
        self.profile, self.send, self.now = profile, send, now
        self.command_enabled, self.simulated = command_enabled, simulated
        self.feedback_max_age = feedback_max_age
        self.check_ready = check_ready
        self.check_health = check_health
        self.owned_groups = set()
        self.writable_groups = WRITABLE
        self.coupled_group_sets = (frozenset(('left_arm', 'right_arm')),)
        self.stop_group_sets = (('left_arm', 'right_arm'), ('waist',), ('head',))
        self.group_feedback = True
        self.last_feedback = Feedback(0., {}, 0., 0., (0., 0.), simulated)
        self.received_at = 0.
        self.invalid = ''
        self.last_source_stamp = 0.
        self.last_pdo_sample = None
        self.targets = {}
        self.reference = {}
        self.stop_failures = ()
        self.stop_targets = {}
        self.stop_requested_at = 0.
        self.sending_hold = False
        # ExecutionState expects independently dated sources. Base is absent.
        self.feedback = self
        self.source_stamps = {}

    @property
    def missing_feedback_groups(self):
        return [g for g, names in self.profile.groups.items()
                if not set(names) <= self.last_feedback.positions.keys()]

    @property
    def reference_positions(self):
        return dict(self.reference)

    def observe(self, positions, velocities, stamp, now, *, sample=None):
        names = set(self.profile.limits)
        required = set(GROUPS['left_arm']) | set(GROUPS['right_arm'])
        if (not required <= set(positions) <= names or set(velocities) != set(positions) or
                any(set(group) & positions.keys() and not set(group) <= positions.keys()
                    for group in self.profile.groups.values()) or
                not all(math.isfinite(v) for v in (*positions.values(), *velocities.values(), stamp, now))):
            self.invalid = 'SNOWBOT_FEEDBACK_INVALID'
            raise ValueError(self.invalid)
        if self.last_pdo_sample is not None:
            if sample is None or sample < self.last_pdo_sample:
                self.invalid = 'SNOWBOT_PDO_SEQUENCE'
                raise ValueError(self.invalid)
            if sample == self.last_pdo_sample:
                return False  # Re-publication never renews measurement freshness or confirms rest.
        if stamp <= self.last_source_stamp or not 0 <= now-stamp <= self.feedback_max_age:
            self.invalid = 'SNOWBOT_FEEDBACK_STALE_OR_UNORDERED'
            raise ValueError(self.invalid)
        self.last_source_stamp, self.received_at, self.invalid = stamp, now, ''
        self.last_pdo_sample = sample
        self.last_feedback = Feedback(stamp, dict(positions), 0., 0., (0., 0.),
                                      self.simulated, joint_velocities=dict(velocities))
        self.source_stamps = {g: stamp for g in self.profile.groups if g not in self.missing_feedback_groups}
        return True

    def require_feedback(self, groups=()):
        now = self.now()
        if (self.invalid or not self.last_source_stamp or
                not 0 <= now-self.last_source_stamp <= self.feedback_max_age or
                not 0 <= now-self.received_at <= self.feedback_max_age):
            raise RuntimeError(self.invalid or 'SNOWBOT_FEEDBACK_UNAVAILABLE')
        missing = set(groups) & set(self.missing_feedback_groups)
        if missing:
            raise RuntimeError('SNOWBOT_FEEDBACK_INCOMPLETE:'+','.join(sorted(missing)))
        if self.check_health is not None and (self.targets or self.stop_targets):
            self.check_health(self.owned_groups or self._groups(self.targets or self.stop_targets))

    def capture(self, groups):
        if not self.command_enabled:
            raise RuntimeError('SNOWBOT_COMMANDS_DISABLED')
        if self.stop_failures:
            raise RuntimeError('SNOWBOT_STOP_UNCONFIRMED_RESTART_REQUIRED')
        self.require_feedback(groups)
        if self.check_ready is not None:
            self.check_ready(groups)
        self.owned_groups = set(groups)
        self.targets.clear()
        self.reference.clear()
        self.stop_targets.clear()

    def _groups(self, target):
        selected = {g for g, names in self.profile.groups.items() if set(names) <= set(target)}
        if not selected or set(target) != {n for g in selected for n in self.profile.groups[g]}:
            raise RuntimeError('SNOWBOT_COMPLETE_GROUP_REQUIRED')
        if not selected <= WRITABLE:
            raise RuntimeError('SNOWBOT_GRIPPER_CONTEXT_ONLY')
        return selected

    def _send(self, target, *, hold=False):
        groups = self._groups(target)
        for coupled in self.coupled_group_sets:
            if groups & coupled:
                # Capture other coupled resources from real feedback once.
                for g in coupled:
                    for n in GROUPS[g]:
                        target.setdefault(n, self.targets.get(n, self.last_feedback.positions[n]))
                groups.update(coupled)
        if any(not math.isfinite(q) or not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n, q in target.items()):
            raise RuntimeError('SNOWBOT_JOINT_LIMIT')
        self.sending_hold = hold
        try:
            self.send(groups, target)
        finally:
            self.sending_hold = False

    def write_joints(self, target):
        if not self.command_enabled:
            raise RuntimeError('SNOWBOT_COMMANDS_DISABLED')
        self.require_feedback(self.owned_groups or self._groups(target))
        self._groups(target)
        if any(not math.isfinite(q) or not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n, q in target.items()):
            raise RuntimeError('SNOWBOT_JOINT_LIMIT')
        packet = dict(target)
        may_have_sent = True
        try:
            self._send(packet)
        except CommandNotSent:
            may_have_sent = False
            raise
        finally:
            # Preserve earlier sends, including partial or unknown delivery.
            # Only a proven pre-transport failure adds no new stop footprint.
            if may_have_sent:
                self.targets.update(packet)
        self.reference.update(packet)

    def write_base(self, velocity):
        if velocity != (0., 0.):
            raise RuntimeError('SNOWBOT_BASE_UNAVAILABLE')

    def stop(self):
        self.reference.clear()
        if not self.targets:
            if self.stop_targets:
                # A hold already sent still needs fresh feedback. A subsequent
                # fault must not erase that outstanding physical stop request.
                try:
                    self.require_feedback(self._groups(self.stop_targets))
                except RuntimeError:
                    self.stop_failures = ('vendor_hold',)
            return  # Read-only/idle shutdown never publishes a motor command.
        self.stop_requested_at = self.now()
        self.stop_targets = {}
        try:
            self.require_feedback()
        except RuntimeError:
            self.stop_failures = tuple(sorted(self._groups(self.targets)))
            self.targets.clear()
            return
        involved = self._groups(self.targets)
        failures = set(self.stop_failures)
        # Normal multi-topic writes check the whole batch before sending.
        # Stops instead try every independent endpoint, even if one is lost.
        for coupled in self.stop_group_sets:
            groups = set(coupled) & involved
            if not groups:
                continue
            try:
                self.require_feedback(groups)
                hold = {n: self.last_feedback.positions[n] for g in coupled if g in groups for n in GROUPS[g]}
                self._send(hold, hold=True)
            except RuntimeError:
                failures.update(groups)
            else:
                self.stop_targets.update(hold)
        self.stop_failures = tuple(sorted(failures))
        self.targets.clear()

    def read_feedback(self, now, dt):
        if self.invalid:
            raise RuntimeError(self.invalid)
        if self.check_health is not None and (self.targets or self.stop_targets):
            self.check_health(self.owned_groups)
        return self.last_feedback

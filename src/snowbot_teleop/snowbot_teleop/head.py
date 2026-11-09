"""Right-stick head targets, independent of the arm/waist position session."""
import math
import time

NAMES = ('head_joint_1', 'head_joint_2')  # pitch (-Y), yaw (+Z)


class HeadTeleop:
    def __init__(self, profile, *, deadzone=.05, input_max_age=.25,
                 feedback_max_age=.2, max_reference_error=.15):
        self.profile = profile
        self.deadzone, self.max_age = deadzone, input_max_age
        self.feedback_max_age = feedback_max_age
        self.max_reference_error = max_reference_error
        self.source = None
        self.seq = -1
        self.stamp = self.received = float('-inf')
        self.last_tick = None
        self.axes = self.target = None
        self.stop_at = float('-inf')
        self.reason = 'HEAD_WAITING_FOR_INPUT'

    @property
    def armed(self):
        return self.target is not None

    def inhibit(self, reason, now):
        if self.armed or self.stop_at == float('-inf'):
            self.stop_at = now
        self.axes = self.target = None
        self.reason = reason

    def ingest(self, left, right, now, clock):
        """Consume a new pair already accepted by PicoControl."""
        if len(right.stick) != 2 or any(type(x) not in (int, float) or
                not -1 <= x <= 1 for x in right.stick):
            self.inhibit('HEAD_INPUT_INVALID', now)
            return
        if left.source_id != self.source:
            self.source, self.seq, self.stamp = left.source_id, -1, float('-inf')
            self.inhibit('HEAD_SOURCE_CHANGED', now)
        stamp = min(left.stamp, right.stamp)
        if (left.seq <= self.seq or stamp <= self.stamp or
                not 0 <= now-stamp <= self.max_age):
            self.inhibit('HEAD_INPUT_STALE', now)
            return
        self.seq, self.stamp, self.received = left.seq, stamp, clock
        # OpenXR up is negative Y; head pitch is about robot -Y.
        self.axes = tuple(0. if abs(x) < self.deadzone else -x
                          for x in (right.stick[1], right.stick[0]))

    def stopped(self, q, v, stamp, now):
        return (0 <= now-stamp <= self.feedback_max_age and stamp > self.stop_at and
                all(math.isfinite(q.get(n, math.nan)) and
                    math.isfinite(v.get(n, math.nan)) and
                    abs(v[n]) <= self.profile.feedback_tolerances['joint_velocity'] and
                    (self.target is None or abs(q[n]-self.target[i]) <=
                     self.profile.feedback_tolerances['joint_position']) for i, n in enumerate(NAMES)))

    def tick(self, now, clock, q, v, feedback_stamp, *, allowed):
        dt = 0. if self.last_tick is None else clock-self.last_tick
        self.last_tick = clock
        reason = ('HEAD_NOT_PERMITTED' if not allowed else
                  'HEAD_TICK_GAP' if not 0 < dt <= self.profile.max_tick_gap else
                  'HEAD_INPUT_TIMEOUT' if not (0 <= now-self.stamp <= self.max_age and
                      0 <= clock-self.received <= self.max_age) else
                  'HEAD_FEEDBACK_UNAVAILABLE' if not (0 <= now-feedback_stamp <= self.feedback_max_age and
                      all(math.isfinite(q.get(n, math.nan)) and math.isfinite(v.get(n, math.nan)) and
                          self.profile.limits[n][0]-self.profile.feedback_tolerances['joint_position'] <= q[n] <=
                          self.profile.limits[n][1]+self.profile.feedback_tolerances['joint_position'] for n in NAMES)) else '')
        if reason:
            self.inhibit(reason, now)
            return None
        if not self.armed:
            if self.axes != (0., 0.) or not self.stopped(q, v, feedback_stamp, now):
                self.reason = 'HEAD_CENTRE_STICK_AND_WAIT_FOR_STOP'
                return None
            self.target = tuple(min(self.profile.limits[n][1], max(self.profile.limits[n][0], q[n])) for n in NAMES)
        if any(abs(q[n]-self.target[i]) > self.max_reference_error for i, n in enumerate(NAMES)):
            self.inhibit('HEAD_FOLLOWING_ERROR', now)
            return None
        # Native smoothstep owns interpolation (peak slope 1.5). A delayed tick
        # cannot accumulate a large target jump or catch up missed time.
        self.target = tuple(min(self.profile.limits[n][1], max(self.profile.limits[n][0],
            value + axis*self.profile.speed(n)/1.5*min(dt, self.profile.control_period)))
            for n, value, axis in zip(NAMES, self.target, self.axes))
        self.reason = 'HEAD_READY' if self.axes == (0., 0.) else 'HEAD_ACTIVE'
        return self.target


class TeleopHead:
    def __init__(self, node):
        from std_msgs.msg import Float64MultiArray
        from snowbot_hardware.protocol import GraphSnapshot
        self.node, self.message_type = node, Float64MultiArray
        self.core = HeadTeleop(node.profile,
                               feedback_max_age=node.tool_cfg['feedback_max_age'],
                               max_reference_error=node.tool_cfg['max_reference_error'])
        self.writable = bool(node.get_parameter('command_enabled').value and not node.simulation)
        self.publisher = node.create_publisher(Float64MultiArray, '/bindu/head/target', 1) if self.writable else None
        self.graph = GraphSnapshot(node)
        self.instance = None
        self.fault = ''
        self.last_sent = float('-inf')
        self.sent_active = False

    def endpoints(self):
        n = self.node
        pubs = self.graph.get_publishers_info_by_topic('/bindu/head/target')
        receivers = self.graph.get_subscriptions_info_by_topic('/bindu/head/target')
        return (not self.graph.get_publishers_info_by_topic('/head/motor_cmd') and len(pubs) == 1 and pubs[0].node_name == n.get_name() and
                pubs[0].node_namespace == n.get_namespace() and
                any(info.node_namespace.rstrip('/')+'/'+info.node_name == '/servo_node' and
                    info.topic_type == 'std_msgs/msg/Float64MultiArray' for info in receivers))

    def stopped(self):
        s = self.node.state
        return bool(s and self.core.stopped(dict(zip(s.joints.name, s.joints.position)),
            dict(zip(s.joints.name, s.joints.velocity)),
            s.feedback_stamp.sec+s.feedback_stamp.nanosec*1e-9, self.node.now()))

    def tick(self, control, healthy):
        n, clock = self.node, time.monotonic()
        permitted = (healthy and not control.pending and self.core.source == control.source and
                     self.core.seq == control.seq and control.phase in ('standby', 'tool_standby', 'following'))
        # Permission loss bypasses normal target pacing.
        if (self.core.armed and permitted and 0 <= n.now()-self.core.stamp <= self.core.max_age and
                clock-self.last_sent < n.profile.control_period):
            return
        s = n.state
        q, v, stamp = {}, {}, float('-inf')
        if s:
            q, v = dict(zip(s.joints.name, s.joints.position)), dict(zip(s.joints.name, s.joints.velocity))
            stamp = s.feedback_stamp.sec+s.feedback_stamp.nanosec*1e-9
            if self.instance is None:
                self.instance = s.instance_id
            elif self.instance != s.instance_id:
                self.fault = 'HEAD_EXECUTOR_CHANGED_RESTART_REQUIRED'
        allowed = self.writable and not self.fault and self.endpoints() and permitted
        target = self.core.tick(n.now(), time.monotonic(), q, v, stamp, allowed=allowed)
        if target is not None:
            # Keep the original VR deadline through the native position port.
            self.publisher.publish(self.message_type(data=[*target, self.core.stamp, self.core.max_age, self.node.now(), 0.]))
            self.sent_active = True
            self.last_sent = time.monotonic()
        else:
            self.stop()

    def stop(self):
        if self.publisher and self.sent_active:
            self.publisher.publish(self.message_type(data=[0., 0., self.node.now(), self.core.max_age, self.node.now(), 1.]))
            self.sent_active = False

    def snapshot(self):
        return dict(writable=self.writable, armed=self.core.armed, reason=self.fault or self.core.reason,
                    target=self.core.target, stop_confirmed=self.stopped() if self.writable else None)

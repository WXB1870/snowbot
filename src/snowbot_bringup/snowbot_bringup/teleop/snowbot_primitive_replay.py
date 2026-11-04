"""Operator-run replay of certified Snowbot atomic joint targets.

Preview is offline. --execute uses an already running session_v3 executor;
Prepare is an empty-hand entry to the side-scoop ready pose. No driver
startup, motor enabling, encoder homing, or base commands are performed.
Prepare opens/closes the grippers only after separate operator Enter confirmations.
"""
import argparse
import bisect
from dataclasses import dataclass, replace
import hashlib
import gzip
import json
import math
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

from .snowbot_waist_test import GROUPS, NAMES, checked_state


def kinematic_digest(path):
    joints = []
    for j in ET.parse(path).findall('joint'):
        origin, axis = j.find('origin'), j.find('axis')
        def vector(element, key, default):
            return [float(v) for v in (element.get(key, default) if element is not None else default).split()]
        joints.append([j.get('name'), j.get('type'), j.find('parent').get('link'),
                       j.find('child').get('link'), vector(origin, 'xyz', '0 0 0'),
                       vector(origin, 'rpy', '0 0 0'), vector(axis, 'xyz', '1 0 0')])
    return hashlib.sha256(json.dumps(sorted(joints), separators=(',', ':')).encode()).hexdigest()


@dataclass(frozen=True)
class Replay:
    times: tuple
    points: tuple
    names: tuple
    tool_id: str
    kinematics: str
    speed: float
    waist_lead_s: float = 0.
    backend_ready: bool = False
    reference_margin_rad: float = 0.
    empty_prepare: bool = False
    measured_start: tuple = ()
    start_reference_source: str = 'measured'
    reverse_prepare: bool = False

    @property
    def duration(self):
        return self.times[-1] / self.speed + self.waist_lead_s

    def _at(self, elapsed):
        t = min(max(0., elapsed * self.speed), self.times[-1])
        i = min(bisect.bisect_right(self.times, t)-1, len(self.times)-2)
        u = (t-self.times[i])/(self.times[i+1]-self.times[i])
        return tuple(a+(b-a)*u for a, b in zip(self.points[i], self.points[i+1]))

    def sample(self, elapsed):
        waist = self._at(elapsed)
        arms = self._at(elapsed-self.waist_lead_s)
        return waist[:2]+arms[2:]

    def validate_profile(self, profile):
        if self.reverse_prepare:
            # Apply the same hard limits and margin rule in the original direction.
            replace(self, times=tuple(self.times[-1]-t for t in reversed(self.times)),
                    points=tuple(reversed(self.points)), empty_prepare=True,
                    reverse_prepare=False).validate_profile(profile)
            return
        if tuple(n for g in GROUPS for n in profile.groups.get(g, ())) != self.names:
            raise ValueError('REPLAY_GROUP_LAYOUT')
        for row in self.points:
            for n, q in zip(self.names, row):
                margin = self.reference_margin_rad if n.startswith('arm_') else 0.
                lo, hi = profile.limits[n]
                low, high = lo+margin, hi-margin
                if self.empty_prepare:
                    start = self.points[0][self.names.index(n)]
                    low, high = min(low, start), max(high, start)
                if not lo <= q <= hi:
                    raise ValueError('REPLAY_POSITION_LIMIT:'+n)
                if not low <= q <= high:
                    raise ValueError(('PREPARE_REFERENCE_MARGIN_ENTRY:' if self.empty_prepare else 'REPLAY_POSITION_LIMIT:')+n)
        if self.empty_prepare:
            for i, n in enumerate(self.names):
                margin = self.reference_margin_rad if n.startswith('arm_') else 0.
                low, high = profile.limits[n][0]+margin, profile.limits[n][1]-margin
                if not low <= self.points[-1][i] <= high:
                    raise ValueError('PREPARE_END_REFERENCE_MARGIN:'+n)
                for a,b in zip(self.points,self.points[1:]):
                    if ((a[i]<low and b[i]<a[i]-1e-12) or
                            (a[i]>high and b[i]>a[i]+1e-12) or
                            (low<=a[i]<=high and not low<=b[i]<=high)):
                        raise ValueError('PREPARE_REFERENCE_MARGIN_ENTRY:'+n)
        for a, b, t0, t1 in zip(self.points, self.points[1:], self.times, self.times[1:]):
            for n, x, y in zip(self.names, a, b):
                if abs(y-x)/(t1-t0)*self.speed > profile.speed(n)+1e-7:
                    raise ValueError('REPLAY_VELOCITY_LIMIT:'+n)


def load_replay(path, action, speed=1., waist_lead_s=None):
    if not math.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('REPLAY_SPEED_REQUIRES_0_TO_1')
    path = Path(path).resolve()
    manifest = json.loads(path.read_text())
    if manifest.get('schema_version') != 1 or manifest.get('validation_passed') is not True:
        raise ValueError('REPLAY_UNVALIDATED_LIBRARY')
    lead = manifest.get('waist_lead_s', 0.) if waist_lead_s is None else waist_lead_s
    if type(lead) not in (float, int) or not math.isfinite(lead) or not 0 <= lead <= 1.5:
        raise ValueError('REPLAY_WAIST_LEAD')
    margin = manifest.get('posture_reference_margin_rad', 0.)
    if type(margin) not in (float, int) or not math.isfinite(margin) or not 0 <= margin < .2:
        raise ValueError('REPLAY_REFERENCE_MARGIN')
    item = (manifest.get('transfers', {}) | manifest['actions'])[action]
    target = (path.parent/item['targets']).resolve()
    if not target.is_relative_to(path.parent):
        raise ValueError('REPLAY_ASSET_PATH')
    payload = target.read_bytes()
    if hashlib.sha256(payload).hexdigest() != item['sha256']:
        raise ValueError('REPLAY_ASSET_IDENTITY')
    data = json.loads(gzip.decompress(payload) if target.suffix == '.gz' else payload)
    if (data.get('schema_version') != 1 or data.get('mode') != 'joint_target' or
            data.get('runtime_mode') != 'real' or tuple(data['joint_names']) != NAMES or
            tuple(data['resource_groups']) != GROUPS or data.get('period_s') != .02 or
            data.get('valid_for_s') != .25):
        raise ValueError('REPLAY_TARGET_CONTRACT')
    samples = data['samples']
    if len(samples) < 2:
        raise ValueError('REPLAY_EMPTY')
    times, points = tuple(s['t'] for s in samples), tuple(tuple(s['q']) for s in samples)
    if (times[0] != 0 or any(type(t) not in (int, float) or not math.isfinite(t) for t in times) or
            any(not 0 < b-a <= .020001 for a, b in zip(times, times[1:])) or
            any(len(row) != len(NAMES) or any(type(q) not in (float, int) or not math.isfinite(q)
                                             for q in row) for row in points) or
            tuple(data['start_q_rad']) != points[0]):
        raise ValueError('REPLAY_TIMELINE_OR_VALUES')
    return Replay(times, points, NAMES, manifest['tool_id'],
                  manifest['kinematic_sha256'], speed, lead, manifest.get('backend_ready') is True, margin)



@dataclass(frozen=True)
class ReplaySequence:
    segments: tuple
    route: tuple
    hold_s: float = 3.

    @property
    def points(self): return (self.segments[0].points[0], self.segments[-1].points[-1])
    @property
    def duration(self): return sum(s.duration for s in self.segments)+self.hold_s*(len(self.segments)-1)
    @property
    def names(self): return self.segments[0].names
    @property
    def tool_id(self): return self.segments[0].tool_id
    @property
    def kinematics(self): return self.segments[0].kinematics
    @property
    def speed(self): return self.segments[0].speed
    @property
    def waist_lead_s(self): return self.segments[0].waist_lead_s
    @property
    def backend_ready(self): return all(s.backend_ready for s in self.segments)
    @property
    def reference_margin_rad(self): return self.segments[0].reference_margin_rad

    def sample(self, elapsed):
        for segment in self.segments[:-1]:
            if elapsed <= segment.duration+self.hold_s:
                return segment.sample(min(elapsed, segment.duration))
            elapsed -= segment.duration+self.hold_s
        return self.segments[-1].sample(elapsed)

    def validate_profile(self, profile):
        for segment in self.segments: segment.validate_profile(profile)


def plan_from_current(path, action, measured, speed=1., waist_lead_s=None, tolerance=.01):
    """Join only certified grasp-preserving edges; unknown starts never interpolate."""
    import heapq
    manifest = json.loads(Path(path).read_text())
    if manifest.get('transition_validation_passed') is not True:
        raise ValueError('REPLAY_TRANSITIONS_UNVALIDATED')
    if not 0 < tolerance <= math.radians(2) or not set(NAMES) <= set(measured):
        raise ValueError('REPLAY_TRANSITION_START_LAYOUT')
    if any(not math.isfinite(measured[n]) for n in NAMES):
        raise ValueError('REPLAY_TRANSITION_START_LAYOUT')
    nodes = manifest['nodes']
    errors = {key: max(abs(measured[n]-q) for n, q in zip(NAMES, values))
              for key, values in nodes.items() if len(values) == len(NAMES)}
    if not errors or min(errors.values()) > tolerance:
        raise RuntimeError('REPLAY_UNKNOWN_START: use a certified action start/end pose')
    source = min(errors, key=errors.get)
    target = manifest['home_node'] if action == 'Home' else manifest['actions'][action]['start_node']
    edges, plans = {}, {}
    for edge in manifest['edges']:
        key, a, b = edge['segment'], edge['from'], edge['to']
        segment = load_replay(path, key, speed, waist_lead_s)
        if (max(abs(x-y) for x, y in zip(segment.points[0], nodes[a])) > 1e-8 or
                max(abs(x-y) for x, y in zip(segment.points[-1], nodes[b])) > 1e-8):
            raise ValueError('REPLAY_TRANSITION_ENDPOINT:'+key)
        plans[key] = segment
        edges.setdefault(a, []).append((b, key, segment.duration+3.))
    queue, costs = [(0., source, ())], {source: 0.}
    selected = None
    while queue:
        cost, node, route = heapq.heappop(queue)
        if cost != costs[node]: continue
        if node == target:
            selected = route
            break
        for following, key, duration in edges.get(node, ()):
            candidate = cost+duration
            if candidate < costs.get(following, math.inf):
                costs[following] = candidate
                heapq.heappush(queue, (candidate, following, route+(key,)))
    if selected is None: raise RuntimeError('REPLAY_TRANSITION_UNREACHABLE')
    if action != 'Home':
        selected += (action,)
    if not selected:
        # A at home still confirms the measured pose and stop; no artificial path.
        home = load_replay(path, manifest['home_node'].split(':')[0], speed, waist_lead_s)
        selected_plans = (Replay((0., .02), (tuple(nodes[target]),)*2, home.names,
                                home.tool_id, home.kinematics, speed, 0.,
                                home.backend_ready, home.reference_margin_rad),)
    else:
        selected_plans = tuple(plans[k] if k in plans else load_replay(path, k, speed, waist_lead_s)
                               for k in selected)
    for before, after in zip(selected_plans, selected_plans[1:]):
        if (before.tool_id != after.tool_id or before.kinematics != after.kinematics or
                before.reference_margin_rad != after.reference_margin_rad or
                max(abs(x-y) for x, y in zip(before.points[-1], after.points[0])) > 1e-8):
            raise ValueError('REPLAY_TRANSITION_JOIN')
    return ReplaySequence(selected_plans, selected)


class AxialGraspGuard:
    """Check only the four constrained directions of the two released grips."""
    def __init__(self, urdf, specification):
        import numpy as np
        from bindu_kinematics.model import ArmModel
        root = ET.parse(urdf)
        locked = {j.get('name'): 0. for j in root.findall('joint')
                  if j.get('type') != 'fixed' and j.get('name') not in NAMES}
        self.np, self.spec = np, specification
        self.model = ArmModel(dict(urdf=str(urdf), joint_names=NAMES, locked_joints=locked,
            tool_transform=np.eye(4).ravel(), ee_link=specification['left_frame']))

    def check(self, positions):
        np, s = self.np, self.spec
        frames = self.model.frames([positions[n] for n in NAMES], None,
                                   [s['left_frame'], s['right_frame']])
        points, axes = [], []
        for side in ('left', 'right'):
            F = frames[s[side+'_frame']]
            points.append(F[:3, 3]+F[:3, :3]@np.array(s[side+'_grip_in_flange_m']))
            axes.append(F[:3, :3]@np.array(s[side+'_axis_in_flange']))
        error = points[1]-points[0]-s['shaft_separation_m']*axes[1]-s['bar_separation_m']*axes[0]
        position = float(np.linalg.norm(error))
        angle = math.asin(float(np.clip(axes[0]@axes[1], -1., 1.)))
        if position > s['max_position_error_m'] or abs(angle) > s['max_axis_error_rad']:
            raise RuntimeError(f'REPLAY_GRASP_ERROR:position_m={position:.6f},axis_rad={angle:.6f}')
        return position, angle



def load_prepare_return(path, prepare_log, speed=1.):
    """One-time empty-hand reversal of the old zero-to-Prepare target stream."""
    home = load_replay(path, 'SideScoop', speed, waist_lead_s=0.)
    rows = [json.loads(line) for line in Path(prepare_log).read_text().splitlines()]
    metadata = next(r for r in rows if r['kind'] == 'plan')
    targets = [r for r in rows if r['kind'] == 'target']
    if (metadata['action'] != 'Prepare' or metadata['tool_present'] or not targets or
            targets[-1]['source_stamp']-targets[0]['source_stamp'] < metadata['duration_s']-1e-6):
        raise ValueError('RETURN_ZERO_REQUIRES_COMPLETE_PREPARE')
    end = targets[-1]['source_stamp']
    return replace(home, times=tuple(end-r['source_stamp'] for r in reversed(targets)),
                   points=tuple(tuple(r['q']) for r in reversed(targets)), reverse_prepare=True)


def plan_prepare(path, robot, measured, speed=1., *, reference=None):
    """Empty-hand entry from measured joints; validate before acquiring a lease."""
    import numpy as np
    from bindu_execution.interpolation import Curve, State
    from bindu_kinematics.tool_geometry import make_collision_checker
    from bindu_kinematics.bimanual_worker import prepare_limit_reference
    from snowbot_hardware.parameters import load_mode_config

    home = load_replay(path, 'SideScoop', speed, waist_lead_s=0.)
    profile = robot.profile
    tool = json.loads(robot.tool_config.read_text())
    tolerance = float(tool.get('measured_joint_limit_tolerance_rad', 0.))
    if not math.isfinite(tolerance) or not 0 <= tolerance <= math.pi:
        raise ValueError('BIMANUAL_MEASURED_LIMIT_TOLERANCE_CONFIG')
    context = tuple(n for group in profile.groups.values() for n in group)
    for name in context:
        if name not in measured or not math.isfinite(measured[name]):
            raise ValueError('PREPARE_MEASUREMENT_REQUIRED:'+name)
        if not profile.limits[name][0]-tolerance <= measured[name] <= profile.limits[name][1]+tolerance:
            raise ValueError('PREPARE_START_JOINT_LIMIT:'+name)
    after = home.points[0]
    config = load_mode_config(robot.tool_config.with_name('teleop.json'), 'real')
    recovering = any(not profile.limits[n][0] <= measured[n] <= profile.limits[n][1] for n in NAMES)
    start = measured
    reference_source = 'measured'
    if recovering:
        if all(max(profile.limits[n][0]-measured[n], measured[n]-profile.limits[n][1], 0.)
               <= profile.feedback_tolerances['joint_position'] for n in NAMES):
            start = {n: min(profile.limits[n][1], max(profile.limits[n][0], measured[n])) for n in NAMES}
            reference_source = 'boundary_roundoff'
        else:
            if reference is None:
                raise ValueError('PREPARE_HELD_REFERENCE_REQUIRED')
            start = prepare_limit_reference(profile, NAMES, measured, reference, dict(zip(NAMES, after)),
                                            tolerance, float(config['prepare_position_tolerance_rad']))
            reference_source = 'confirmed_pause'
    before = tuple(start[n] for n in NAMES)
    duration = float(config['prepare_duration'])
    for name, a, b in zip(NAMES, before, after):
        shared = name.startswith('body_')
        velocity = min(profile.speed(name)*.8, .025) if shared else profile.speed(name)*.8
        acceleration = min(profile.acceleration(name)*.8, .12) if shared else profile.acceleration(name)*.8
        jerk = min(profile.jerk(name)*.8, .8) if shared else profile.jerk(name)*.8
        distance = abs(b-a)
        duration = max(duration, 1.875*distance/velocity,
                       math.sqrt(10/math.sqrt(3)*distance/acceleration),
                       (60*distance/jerk)**(1/3))
    duration = math.ceil(duration/.02)*.02
    curve = Curve.between(0., duration, State.rest(before), State.rest(after))
    if not curve.valid(profile, NAMES):
        raise ValueError('PREPARE_MOTION_LIMITS')
    times = tuple(k*.02 for k in range(round(duration/.02)+1))
    plan = Replay(times, tuple(curve.sample(t).q for t in times), NAMES,
                  home.tool_id, home.kinematics, speed, 0., home.backend_ready,
                  home.reference_margin_rad, empty_prepare=True,
                  measured_start=tuple(measured[n] for n in NAMES) if recovering else (),
                  start_reference_source=reference_source)
    plan.validate_profile(profile)
    recipe = dict(tool['demo'])
    recipe['left_grip_in_tool_m'] = (np.linalg.inv(np.array(tool['left_to_tool']).reshape(4,4)) @
        np.r_[recipe['grip_offset_m'], 1.])[:3].tolist()
    checker = make_collision_checker(robot.display_urdf, recipe, conservative_convex=True,
                                     tool_present=False, validation_only=True)
    # A rest-to-rest quintic shares one scalar progress across every joint.
    samples = []
    waypoints = ([plan.measured_start] if recovering else [])+[before, after]
    for leg, (begin, end) in enumerate(zip(waypoints, waypoints[1:])):
        count = max(1, math.ceil(max(abs(a-b) for a,b in zip(begin,end))/.005))
        for k in range(count+1):
            if samples and k == 0:
                continue
            u = k/count
            q = dict(measured)
            q.update({n: (1-u)*a+u*b for n,a,b in zip(NAMES,begin,end)})
            samples.append(dict(t=leg+u, q=q, tool=np.eye(4)))
    checker(samples)
    return plan



def run_replay(plan, port, *, settle_timeout=5., execution_mode='position', velocity_tolerance=.01):
    """Port owns ROS/feedback; this loop owns source time and playback progress."""
    nominal = execution_mode == 'impedance'
    attempted, acquired, sent, arrived, stopped, failure = False, False, 0, False, False, None
    try:
        attempted = True
        port.acquire()
        acquired = True
        start = due = renewed = port.steady()
        ros_start = port.ros()
        rest_since = None
        measured_at_goal = False
        while True:
            port.wait_until(due)
            now = port.steady()
            if now-due > .04:
                raise RuntimeError('REPLAY_STREAM_LATE')
            if abs(port.ros()-(ros_start+now-start)) > .02:
                raise RuntimeError('REPLAY_CLOCK_CHANGED')
            q, v = port.check(True)
            elapsed = due-start
            goal = plan.sample(elapsed)
            if now-renewed >= .5:
                port.renew()
                renewed = port.steady()
            if port.steady()-due > .04:
                raise RuntimeError('REPLAY_STREAM_LATE')
            # The source date is the scheduled tick, never the later service time.
            port.send(goal, ros_start+elapsed)
            sent += 1
            if elapsed >= plan.duration:
                arrival = port.nominal_reference() if nominal else q
                measured_at_goal = all(abs(q[n]-x) <= port.arrival_tolerance(n)
                                       for n,x in zip(plan.names,plan.points[-1]))
                resting = (arrival is not None and all(abs(arrival[n]-x) <= port.arrival_tolerance(n)
                               for n,x in zip(plan.names,plan.points[-1])) and
                           all(abs(v[n]) <= velocity_tolerance for n in plan.names))
                rest_since = now if resting and rest_since is None else rest_since if resting else None
                if rest_since is not None and now-rest_since >= .2:
                    arrived = True
                    break
                if elapsed-plan.duration >= settle_timeout:
                    raise RuntimeError('REPLAY_ARRIVAL_UNCONFIRMED')
            due += .02
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        if attempted:
            try:
                stopped = port.release_and_confirm_stop() is True
            except (Exception, KeyboardInterrupt) as exc:
                failure = (failure+'; ' if failure else '')+'STOP_UNCONFIRMED:'+str(exc)
    result = dict(targets_sent=sent, measured_arrival_confirmed=arrived and measured_at_goal,
                  nominal_arrival_confirmed=arrived and nominal,
                  measured_end_error_rad={n:q[n]-x for n,x in zip(plan.names,plan.points[-1])} if arrived else None,
                  stop_confirmed=stopped, completed=arrived and failure is None,
                  error=failure, driver_started=False, snow_removal_verified=False)
    if failure:
        raise RuntimeError(json.dumps(result))
    return result


def confirm_prepare_grippers(port, operation):
    message = ('Prepare 已到位；按 Enter 打开双爪以装夹雪铲：' if operation == 'open' else
               '装夹雪铲后按 Enter 关闭双爪：')
    if input(message).strip():
        raise RuntimeError('PREPARE_OPERATOR_CANCELED')
    port.grippers(operation)


def execute(args, plan):
    # Deliberately absent from offline preview and unit tests.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from bindu_interfaces.msg import ExecutionState, MotionCommand, RuntimeEvent, GripperState
    from bindu_interfaces.srv import Lease, SubmitMotion, GripperControl
    from std_srvs.srv import Trigger
    from bindu_runtime.stop_evidence import sampled_nominal_reference
    from snowbot_ctl_msgs.msg import PositionObservation
    from bindu_runtime.robot_setup import RobotSetup
    from bindu_runtime.common import stamp

    if args.robot_config is None:
        from bindu_contracts.resources import package_share
        args.robot_config = package_share('snowbot_bringup')/'config/robots/snowbot/robot_snowbot_real.json'
    robot = RobotSetup.load(args.robot_config)
    plan.validate_profile(robot.profile)
    tool_config = json.loads(robot.tool_config.read_text())
    from snowbot_hardware.parameters import load_mode_config
    gripper_config = load_mode_config(robot.tool_config.with_name('grippers.json'), 'real')
    configured_margin = tool_config.get('posture_reference_margin_rad', 0.)
    if abs(configured_margin-plan.reference_margin_rad) > 1e-10:
        raise ValueError('REPLAY_REFERENCE_MARGIN_CONFIG_MISMATCH')
    if kinematic_digest(robot.urdf) != plan.kinematics:
        raise ValueError('REPLAY_KINEMATICS_MISMATCH')
    empty_handed = args.action in ('Prepare', 'ReturnZero')
    if not empty_handed and args.tool_id != plan.tool_id:
        raise ValueError('REPLAY_TOOL_ID_MISMATCH')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.open('x')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('snowbot_primitive_replay_'+uuid.uuid4().hex[:8])
    ns = '/'+args.namespace.strip('/')
    topic = '/'+args.vendor_namespace.strip('/') if args.vendor_namespace.strip('/') else ''
    latest, task = {}, 'primitive-'+uuid.uuid4().hex

    def record(kind, **data):
        log.write(json.dumps(dict(kind=kind, steady=time.monotonic(), **data), allow_nan=False)+'\n')

    def receive(kind):
        def callback(msg):
            latest[kind] = (msg, time.monotonic())
        return callback

    node.create_subscription(ExecutionState, ns+'/execution/state', receive('state'), 10)
    node.create_subscription(PositionObservation, topic+'/bindu/position/observation',
                             receive('pdo'), qos_profile_sensor_data)
    commands = set()
    def event(msg):
        if msg.task_id == task or msg.command_id in commands:
            record('event', state=msg.state, code=msg.code)
            if msg.state in ('TARGET_REJECTED', 'REJECTED', 'FAILED'):
                latest.setdefault('error', msg.code)
    node.create_subscription(RuntimeEvent, ns+'/events', event, 100)
    leases = node.create_client(Lease, ns+'/execution/lease')
    submit = node.create_client(SubmitMotion, ns+'/execution/submit')
    holds = node.create_client(Trigger, ns+'/execution/held_reference')
    gripper_leases = node.create_client(Lease, ns+'/gripper/lease')
    gripper_control = node.create_client(GripperControl, ns+'/gripper/control')
    node.create_subscription(GripperState, ns+'/gripper/state',
        lambda msg: latest.update({'gripper_'+msg.side: (msg, time.monotonic())}), 10)

    def wait(predicate, timeout):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('REPLAY_WAIT_TIMEOUT')
            rclpy.spin_once(node, timeout_sec=.005)

    def call(client, request):
        if not client.service_is_ready():
            raise RuntimeError('REPLAY_SERVICE_UNAVAILABLE')
        future = client.call_async(request)
        wait(future.done, .25)
        reply = future.result()
        if not (reply.ok if hasattr(reply, 'ok') else reply.accepted):
            raise RuntimeError(reply.code)
        return reply

    class Port:
        lease = None
        boot = None
        instance = None
        @staticmethod
        def steady(): return time.monotonic()
        @staticmethod
        def ros(): return node.get_clock().now().nanoseconds*1e-9

        def grippers(self, operation):
            # Reuse the adapter's configured endpoints, leases and measured completion.
            confirmed_at = self.ros()
            wait(lambda: latest['state'][0].stamp.sec+latest['state'][0].stamp.nanosec*1e-9 >= confirmed_at and
                 latest['pdo'][0].header.stamp.sec+latest['pdo'][0].header.stamp.nanosec*1e-9 >= confirmed_at, 1.)
            self.check(False)
            lease = call(gripper_leases, Lease.Request(operation='acquire', owner=task,
                         resources=['left_tool_axis', 'right_tool_axis']))
            sequence = 0
            def command(operation, side=''):
                nonlocal sequence
                sequence += 1
                call(gripper_control, GripperControl.Request(operation=operation, side=side,
                    lease_id=lease.lease_id, epoch=lease.epoch, sequence=sequence,
                    stamp=stamp(self.ros()), valid_for=gripper_config['command_ttl'], load_state='unknown'))
            try:
                command('authorize_direct')
                for side in ('left', 'right'):
                    command(operation, side)
                renewed = self.steady()
                end = renewed+gripper_config['motion_timeout']+gripper_config['stop_timeout']
                while True:
                    self.check(False)
                    if all('gripper_'+side in latest for side in ('left', 'right')):
                        states = [latest['gripper_'+side][0] for side in ('left', 'right')]
                        current = all(m.epoch == lease.epoch and m.control_sequence == sequence for m in states)
                        if current and any(m.state == 'FAILED' or
                                m.state == 'CANCELED' and m.code != 'MANUAL_ENDPOINT_STOPPED' for m in states):
                            raise RuntimeError('REPLAY_GRIPPER_FAILED:'+','.join(m.code for m in states))
                        if current and all(m.state == 'SUCCEEDED' or
                                m.state == 'CANCELED' and m.code == 'MANUAL_ENDPOINT_STOPPED' for m in states):
                            record('gripper_stopped', operation=operation,
                                   codes={m.side:m.code for m in states})
                            break
                    if self.steady() >= end:
                        raise RuntimeError('REPLAY_GRIPPER_'+operation.upper()+'_TIMEOUT')
                    if self.steady()-renewed >= .5:
                        call(gripper_leases, Lease.Request(operation='renew', lease_id=lease.lease_id, epoch=lease.epoch))
                        renewed = self.steady()
                    rclpy.spin_once(node, timeout_sec=.005)
            finally:
                released = call(gripper_leases, Lease.Request(operation='release', lease_id=lease.lease_id, epoch=lease.epoch))
                record('gripper_release', operation=operation, epoch=released.epoch)
                wait(lambda: all('gripper_'+side in latest and
                     latest['gripper_'+side][0].epoch == released.epoch and
                     not latest['gripper_'+side][0].lease_active and
                     latest['gripper_'+side][0].state not in ('MOVING', 'STOPPING', 'FAILED')
                     for side in ('left', 'right')), gripper_config['stop_timeout'])
            q, v = self.check(False)
            record('grippers', operation=operation, measured_q_rad={n:q[n] for n in ('arm_l_joint_8', 'arm_r_joint_8')})
            if operation == 'close':
                print('双爪关爪已停止，请现场确认雪铲已夹牢。')
        @staticmethod
        def wait_until(due): wait(lambda: time.monotonic() >= due, max(.1, due-time.monotonic()+.1))
        def check(self, owned):
            if 'state' not in latest or 'pdo' not in latest:
                raise RuntimeError('REPLAY_FEEDBACK_MISSING')
            s, received = latest['state']
            now, steady = self.ros(), self.steady()
            q, v = checked_state(s, now, received, steady, robot.profile.digest,
                                 owned=owned, epoch=self.lease.epoch if self.lease else 0,
                                 stationary=False, state_max_age=.2, feedback_max_age=.3)
            if self.instance is None: self.instance = s.instance_id
            if not self.instance or s.instance_id != self.instance:
                raise RuntimeError('REPLAY_EXECUTOR_CHANGED')
            p, _ = latest['pdo']
            if self.boot is None: self.boot = p.boot_id
            if not self.boot: raise RuntimeError('REPLAY_BOOT_MISSING')
            if p.execution_mode != (1 if args.execution_mode == 'impedance' else 0):
                raise RuntimeError('REPLAY_EXECUTION_MODE_MISMATCH:'+args.execution_mode)
            if latest.get('error'):
                raise RuntimeError(latest['error'])
            if guard is not None: guard.check(q)
            return q, v
        def nominal_reference(self):
            state = latest['state'][0]
            if self.lease and getattr(self, 'commanded', False):
                sample = sampled_nominal_reference(state, NAMES, self.lease.epoch,
                    self.ros(), tool_config['feedback_max_age'],
                    wait_until=getattr(self,'reference_ready_by',0.), steady_now=self.steady())
                if sample is None: return None
                self.reference_ready_by = 0.
                return dict(zip(NAMES, sample[0]))
            if not holds.service_is_ready():
                raise RuntimeError('REPLAY_HELD_REFERENCE_UNAVAILABLE')
            future = holds.call_async(Trigger.Request())
            wait(future.done, .25)
            reply = future.result()
            if not reply.success: raise RuntimeError(reply.message)
            data = json.loads(reply.message)
            state = latest['state'][0]
            expected = self.lease.lease_id if self.lease else ''
            epoch = self.lease.epoch if self.lease else state.lease_epoch
            if (data['instance_id'] != self.instance or data['profile_hash'] != robot.profile.digest or
                    data['lease_id'] != expected or data['epoch'] != epoch or data['revision'] != state.revision or
                    data['boot_id'] != self.boot or not 0 <= self.ros()-data['stamp'] <= tool_config['feedback_max_age'] or
                    set(data['positions']) != set(NAMES) or
                    any(not math.isfinite(q) or not robot.profile.limits[n][0]<=q<=robot.profile.limits[n][1]
                        for n,q in data['positions'].items())):
                raise RuntimeError('REPLAY_HELD_REFERENCE_IDENTITY')
            identity = (data['boot_id'], data['session'], data['sequence'], data['positions'])
            if getattr(self,'held_identity',None) is not None and identity != self.held_identity:
                raise RuntimeError('REPLAY_HELD_REFERENCE_CHANGED')
            self.held_identity = identity
            return data['positions']

        def acquire(self):
            self.lease = call(leases, Lease.Request(operation='acquire', owner=task, resources=list(GROUPS)))
            if self.lease.profile_hash != robot.profile.digest:
                # The acquired lease remains known so the outer cleanup can revoke it.
                self.release_and_confirm_stop()
                raise RuntimeError('REPLAY_LEASE_PROFILE_MISMATCH')
            wait(lambda: latest['state'][0].lease_active, .25)
        def renew(self):
            call(leases, Lease.Request(operation='renew', lease_id=self.lease.lease_id, epoch=self.lease.epoch))
        def send(self, goal, source):
            msg = MotionCommand(schema_version=2, runtime_mode='real', command_id=uuid.uuid4().hex,
                task_id=task, observation_id=str(latest['pdo'][0].pdo_sample),
                profile_hash=robot.profile.digest, lease_id=self.lease.lease_id, epoch=self.lease.epoch,
                mode='joint_target', resource_groups=list(GROUPS), joint_names=list(NAMES),
                positions=list(goal), stamp=stamp(source), valid_for=.25)
            commands.add(msg.command_id)
            call(submit, SubmitMotion.Request(command=msg))
            if not getattr(self,'commanded',False):
                self.reference_ready_by=self.steady()+tool_config['feedback_max_age']
            self.commanded = True
            record('target', source_stamp=source, q=list(goal))
        @staticmethod
        def arrival_tolerance(name):
            return math.radians(2)
        def release_and_confirm_stop(self):
            if not self.lease: return False
            call(leases, Lease.Request(operation='release', lease_id=self.lease.lease_id, epoch=self.lease.epoch))
            released = self.steady()
            wait(lambda: latest['state'][1] > released and
                 not latest['state'][0].lease_active and latest['state'][0].state != 'STOPPING',
                 robot.profile.stop_timeout+.2)
            self.check(False)
            self.lease = None
            return True

    manifest = json.loads(args.library.read_text())
    guard = AxialGraspGuard(robot.urdf, manifest['grasp']) if 'grasp' in manifest and not empty_handed else None
    port = Port()
    try:
        wait(lambda: 'state' in latest and 'pdo' in latest and leases.service_is_ready() and submit.service_is_ready(), 5.)
        q, v = port.check(False)
        if args.action == 'Prepare':
            initial = dict(q)
            record('prepare_start', measured_q_rad=initial)
            reference = port.nominal_reference() if any(
                max(robot.profile.limits[n][0]-initial[n], initial[n]-robot.profile.limits[n][1], 0.)
                > robot.profile.feedback_tolerances['joint_position'] for n in NAMES) else None
            plan = plan_prepare(args.library, robot, initial, args.speed, reference=reference)
            record('prepare_reference', source=plan.start_reference_source,
                   reference_q_rad=dict(zip(NAMES, plan.points[0])))
            # Validation blocks callbacks; receipt time alone can admit queued old samples.
            checked_at = port.ros()
            wait(lambda: latest['state'][0].stamp.sec+latest['state'][0].stamp.nanosec*1e-9 >= checked_at and
                 latest['pdo'][0].header.stamp.sec+latest['pdo'][0].header.stamp.nanosec*1e-9 >= checked_at, 1.)
            port.check(False)
        elif args.action != 'ReturnZero':
            if manifest.get('transition_validation_passed'):
                anchor = port.nominal_reference() if args.execution_mode=='impedance' else q
                plan = plan_from_current(args.library, args.action, anchor, args.speed, args.waist_lead, args.start_tolerance)
                plan.validate_profile(robot.profile)
        record('plan', execution_mode=args.execution_mode, tool_present=not empty_handed, route=list(getattr(plan, 'route', (args.action,))), action=args.action, duration_s=plan.duration, speed=plan.speed,
               profile_hash=robot.profile.digest, tool_id=plan.tool_id, waist_lead_s=plan.waist_lead_s,
               posture_reference_margin_rad=plan.reference_margin_rad)
        result = run_replay(plan, port, execution_mode=args.execution_mode,
                            velocity_tolerance=robot.profile.feedback_tolerances['joint_velocity'])
        if args.action == 'Prepare':
            confirm_prepare_grippers(port, 'open')
            confirm_prepare_grippers(port, 'close')
        record('result', **result)
        print(json.dumps(result))
    except (Exception, KeyboardInterrupt) as exc:
        record('result', completed=False, error=str(exc) or type(exc).__name__)
        raise
    finally:
        # Also cover acquisition that succeeded before its state-ACK wait failed.
        if port.lease:
            try: port.release_and_confirm_stop()
            except Exception as exc: record('cleanup_stop_unconfirmed', error=str(exc))
        node.destroy_node()
        rclpy.shutdown()
        log.close()


def replay_action(args, parser):
    manifest = json.loads(args.library.read_text())
    if args.action == 'Prepare':
        plan = load_replay(args.library, 'SideScoop', args.speed, waist_lead_s=0.)
        if not args.execute:
            print(json.dumps(dict(action='Prepare', execution_mode=args.execution_mode, empty_handed=True,
                start_source='fresh measured joints on execute; full path checked before lease',
                goal_q_rad=dict(zip(plan.names,plan.points[0])),
                posture_reference_margin_rad=plan.reference_margin_rad,hardware_commands=False)))
            return
    elif args.action == 'ReturnZero':
        plan = load_prepare_return(args.library, args.prepare_log, args.speed)
    elif manifest.get('transition_validation_passed'):
        start = args.from_node or manifest['home_node']
        plan = plan_from_current(args.library, args.action, dict(zip(NAMES, manifest['nodes'][start])),
                                 args.speed, args.waist_lead, args.start_tolerance)
    else:
        plan = load_replay(args.library, args.action, args.speed, args.waist_lead)
    if args.execute:
        if not args.output or (args.action not in ('Prepare', 'ReturnZero') and not args.tool_id):
            parser.error('--execute requires --output, and --tool-id except for empty-handed Prepare/ReturnZero')
        if not plan.backend_ready:
            parser.error('REPLAY_VENDOR_LIMIT_ALIGNMENT_REQUIRED: backend source approval/build is pending')
        execute(args, plan)
    else:
        print(json.dumps(dict(execution_mode=args.execution_mode, route=list(getattr(plan, 'route', (args.action,))), action=args.action, duration_s=plan.duration, speed=args.speed,
                              waist_lead_s=plan.waist_lead_s, posture_reference_margin_rad=plan.reference_margin_rad,
                              start_q_rad=dict(zip(plan.names, plan.points[0])),
                              end_q_rad=dict(zip(plan.names, plan.points[-1])),
                              tool_id=plan.tool_id, hardware_commands=False)))


def interactive_replay(args, parser):
    actions = {'1': 'Push', '2': 'SideScoop', '3': 'Flip'}
    while True:
        print('\n1 前推    2 侧前铲    3 翻倒    q 退出')
        try:
            choice = input('输入编号并按 Enter 执行：').strip()
        except (EOFError, KeyboardInterrupt):
            return
        if choice.lower() == 'q':
            return
        if choice not in actions:
            print('请输入 1、2、3 或 q。')
            continue
        selected = argparse.Namespace(**vars(args))
        selected.action = actions[choice]
        selected.output = args.output_dir/(time.strftime('%Y%m%d-%H%M%S')+'-'+
                                          selected.action+'-'+uuid.uuid4().hex[:8]+'.jsonl')
        replay_action(selected, parser)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--action', choices=('Push', 'SideScoop', 'Flip', 'Home', 'Prepare', 'ReturnZero'))
    parser.add_argument('--prepare-log', type=Path, help='ReturnZero: old empty-hand Prepare JSONL')
    selection.add_argument('--interactive', action='store_true', help='Select 1=Push, 2=SideScoop, 3=Flip; Enter executes')
    parser.add_argument('--from-node', help='Offline route preview from a certified node; default home node')
    parser.add_argument('--speed', type=float, default=1., help='0 < speed <= 1; scales certified timing')
    parser.add_argument('--waist-lead', type=float, help='Waist command lead in real seconds; default from validated library')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--execution-mode', choices=('position','impedance'), default='position',
                        help='Must match the already running backend; never switches its mode')
    parser.add_argument('--robot-config', type=Path)
    parser.add_argument('--tool-id')
    parser.add_argument('--namespace', default='bindu_snowbot')
    parser.add_argument('--vendor-namespace', default='')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--output-dir', type=Path, help='Interactive mode: one new JSONL log per selected action')
    parser.add_argument('--start-tolerance', type=float, default=math.radians(2),
                        help='Start-node matching tolerance in rad; default/max 2 deg')
    args = parser.parse_args(argv)
    if args.action == 'ReturnZero' and not args.prepare_log:
        parser.error('ReturnZero requires --prepare-log')
    if args.interactive:
        if not args.execute or not args.tool_id or not args.output_dir or args.output:
            parser.error('--interactive requires --execute --tool-id --output-dir; omit --output')
        return interactive_replay(args, parser)
    if args.output_dir:
        parser.error('--output-dir requires --interactive')
    return replay_action(args, parser)


if __name__ == '__main__':
    main()

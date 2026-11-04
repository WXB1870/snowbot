"""Operator-run replay of certified Snowbot atomic joint targets.

Preview is offline. --execute uses an already running session_v3 executor;
Prepare is an empty-hand entry to the side-scoop ready pose. No driver
startup, motor enabling, encoder homing, or gripper/base commands are performed.
"""
import argparse
import bisect
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import gzip
import json
import math
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

from .snowbot_waist_test import GROUPS, NAMES, checked_state, checked_pdo, checked_pdo_source


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


@lru_cache(maxsize=8)
def load_library(path):
    """One immutable asset manifest per process; restart to select a new release."""
    return json.loads(Path(path).read_text())


@lru_cache(maxsize=128)
def load_replay(path, action, speed=1., waist_lead_s=None):
    if not math.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('REPLAY_SPEED_REQUIRES_0_TO_1')
    path = Path(path).resolve()
    manifest = load_library(path)
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


@lru_cache(maxsize=16)
def replay_graph(path, speed, waist_lead_s):
    manifest = load_library(Path(path).resolve())
    if manifest.get('transition_validation_passed') is not True:
        raise ValueError('REPLAY_TRANSITIONS_UNVALIDATED')
    nodes = manifest['nodes']
    edges, plans = {}, {}
    for edge in manifest['edges']:
        key, a, b = edge['segment'], edge['from'], edge['to']
        segment = load_replay(path, key, speed, waist_lead_s)
        if (max(abs(x-y) for x, y in zip(segment.points[0], nodes[a])) > 1e-8 or
                max(abs(x-y) for x, y in zip(segment.points[-1], nodes[b])) > 1e-8):
            raise ValueError('REPLAY_TRANSITION_ENDPOINT:'+key)
        plans[key] = segment
        edges.setdefault(a, []).append((b, key, segment.duration+3.))
    return manifest, edges, plans


def plan_from_current(path, action, measured, speed=1., waist_lead_s=None, tolerance=.01):
    """Select a route from current measurements over the cached certified graph."""
    import heapq
    manifest, edges, plans = replay_graph(path, speed, waist_lead_s)
    if not 0 < tolerance <= .0175 or not set(NAMES) <= set(measured):
        raise ValueError('REPLAY_TRANSITION_START_LAYOUT')
    if any(not math.isfinite(measured[n]) for n in NAMES):
        raise ValueError('REPLAY_TRANSITION_START_LAYOUT')
    nodes = manifest['nodes']
    errors = {key: max(abs(measured[n]-q) for n, q in zip(NAMES, values))
              for key, values in nodes.items()}
    if not errors or min(errors.values()) > tolerance:
        raise RuntimeError('REPLAY_UNKNOWN_START: use a certified action start/end pose')
    source = min(errors, key=errors.get)
    target = manifest['home_node'] if action == 'Home' else manifest['actions'][action]['start_node']
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



def plan_prepare(path, robot, measured, speed=1.):
    """Empty-hand entry from measured joints; validate before acquiring a lease."""
    import numpy as np
    from bindu_execution.interpolation import Curve, State
    from bindu_kinematics.tool_geometry import make_collision_checker
    from snowbot_hardware.parameters import load_mode_config

    manifest = load_library(Path(path).resolve())
    home = load_replay(path, manifest['home_node'].split(':')[0], speed, waist_lead_s=0.)
    profile = robot.profile
    context = tuple(n for group in profile.groups.values() for n in group)
    for name in context:
        if name not in measured or not math.isfinite(measured[name]):
            raise ValueError('PREPARE_MEASUREMENT_REQUIRED:'+name)
        if not profile.limits[name][0] <= measured[name] <= profile.limits[name][1]:
            raise ValueError('PREPARE_START_JOINT_LIMIT:'+name)
    before = tuple(measured[n] for n in NAMES)
    after = tuple(manifest['nodes'][manifest['home_node']])
    config = load_mode_config(robot.tool_config.with_name('teleop.json'), 'real')
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
                  home.reference_margin_rad, empty_prepare=True)
    plan.validate_profile(profile)
    tool = json.loads(robot.tool_config.read_text())
    recipe = dict(tool['demo'])
    recipe['left_grip_in_tool_m'] = (np.linalg.inv(np.array(tool['left_to_tool']).reshape(4,4)) @
        np.r_[recipe['grip_offset_m'], 1.])[:3].tolist()
    checker = make_collision_checker(robot.display_urdf, recipe, conservative_convex=True,
                                     tool_present=False, validation_only=True)
    # A rest-to-rest quintic shares one scalar progress across every joint.
    count = max(1, math.ceil(max(abs(a-b) for a,b in zip(before,after))/.005))
    samples = []
    for k in range(count+1):
        u = k/count
        q = dict(measured)
        q.update({n: (1-u)*a+u*b for n,a,b in zip(NAMES,before,after)})
        samples.append(dict(t=u, q=q, tool=np.eye(4)))
    checker(samples)
    return plan


def run_replay(plan, port, *, start_tolerance=.01, follow_tolerance=.10, settle_timeout=5.,
               velocity_tolerance=.005, settle_time=.2, command_ttl=.25):
    """Port owns ROS/feedback; this loop owns source time and playback progress."""
    if not 0 < start_tolerance <= .0175 or not 0 < follow_tolerance <= .15:
        raise ValueError('REPLAY_TOLERANCE')
    measured, velocity = port.check(False)
    if (max(abs(measured[n]-q) for n, q in zip(plan.names, plan.points[0])) > start_tolerance or
            max(abs(velocity[n]) for n in plan.names) > velocity_tolerance):
        raise RuntimeError('REPLAY_START_POSE_MISMATCH: prepare the exported start pose')
    attempted, acquired, sent, arrived, stopped, failure = False, False, 0, False, False, None
    try:
        attempted = True
        port.acquire()
        acquired = True
        initial, velocity = port.check(True)
        if max(abs(initial[n]-q) for n, q in zip(plan.names, plan.points[0])) > start_tolerance:
            raise RuntimeError('REPLAY_START_MOVED')
        start = due = renewed = port.steady()
        ros_start = port.ros()
        rest_since = None
        while True:
            port.wait_until(due)
            now = port.steady()
            if now-due >= command_ttl:
                raise RuntimeError('REPLAY_STREAM_LATE')
            if abs(port.ros()-(ros_start+now-start)) > command_ttl:
                raise RuntimeError('REPLAY_CLOCK_CHANGED')
            q, v = port.check(True)
            elapsed = due-start
            goal = plan.sample(elapsed)
            if max(abs(q[n]-x) for n, x in zip(plan.names, goal)) > follow_tolerance:
                raise RuntimeError('REPLAY_TRACKING_ERROR')
            if now-renewed >= .5:
                port.renew()
                renewed = port.steady()
            if port.steady()-due >= command_ttl:
                raise RuntimeError('REPLAY_STREAM_LATE')
            # The source date is the scheduled tick, never the later service time.
            port.send(goal, ros_start+elapsed)
            sent += 1
            if elapsed >= plan.duration:
                resting = (all(abs(q[n]-x) <= port.arrival_tolerance(n)
                               for n, x in zip(plan.names, plan.points[-1])) and
                           all(abs(v[n]) <= velocity_tolerance for n in plan.names))
                rest_since = now if resting and rest_since is None else rest_since if resting else None
                if rest_since is not None and now-rest_since >= settle_time:
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
    result = dict(targets_sent=sent, measured_arrival_confirmed=arrived,
                  stop_confirmed=stopped, completed=arrived and failure is None,
                  error=failure, driver_started=False, snow_removal_verified=False)
    if failure:
        raise RuntimeError(json.dumps(result))
    return result


def execute(args, plan):
    # Deliberately absent from offline preview and unit tests.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from bindu_interfaces.msg import ExecutionState, MotionCommand, RuntimeEvent
    from bindu_interfaces.srv import Lease, SubmitMotion
    from snowbot_ctl_msgs.msg import PositionObservation
    from bindu_runtime.robot_setup import RobotSetup
    from bindu_runtime.common import stamp

    if args.robot_config is None:
        from bindu_contracts.resources import package_share
        args.robot_config = package_share('snowbot_bringup')/'config/robots/snowbot/robot_snowbot_real.json'
    robot = RobotSetup.load(args.robot_config)
    from snowbot_hardware.parameters import load_parameters, load_mode_config
    settings = load_parameters()
    velocity_tolerance = settings['execution']['motion_arrival_velocity_tolerance_rad_s']
    settle_time = settings['execution']['stop_confirmation_window_s']
    settle_timeout = settings['pico']['motion_completion_grace_s']
    prepare_tolerance = load_mode_config(robot.tool_config.with_name('teleop.json'), 'real')['prepare_position_tolerance_rad']
    plan.validate_profile(robot.profile)
    configured_margin = json.loads(robot.tool_config.read_text()).get('posture_reference_margin_rad', 0.)
    if abs(configured_margin-plan.reference_margin_rad) > 1e-10:
        raise ValueError('REPLAY_REFERENCE_MARGIN_CONFIG_MISMATCH')
    if kinematic_digest(robot.urdf) != plan.kinematics:
        raise ValueError('REPLAY_KINEMATICS_MISMATCH')
    if args.action != 'Prepare' and args.tool_id != plan.tool_id:
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

    prepare_context = None

    class Port:
        lease = None
        boot = None
        sample = None
        progress = None
        instance = None
        @staticmethod
        def steady(): return time.monotonic()
        @staticmethod
        def ros(): return node.get_clock().now().nanoseconds*1e-9
        @staticmethod
        def wait_until(due): wait(lambda: time.monotonic() >= due, max(.1, due-time.monotonic()+.1))
        def check(self, owned):
            if 'state' not in latest or 'pdo' not in latest:
                raise RuntimeError('REPLAY_FEEDBACK_MISSING')
            s, received = latest['state']
            now, steady = self.ros(), self.steady()
            q, v = checked_state(s, now, received, steady, robot.profile.digest,
                                 owned=owned, epoch=self.lease.epoch if self.lease else 0,
                                 stationary=False)
            if self.instance is None: self.instance = s.instance_id
            if not self.instance or s.instance_id != self.instance:
                raise RuntimeError('REPLAY_EXECUTOR_CHANGED')
            p, received = latest['pdo']
            if self.boot is None: self.boot = p.boot_id
            if not self.boot: raise RuntimeError('REPLAY_BOOT_MISSING')
            checked_pdo(p, now, received, steady, self.boot)
            checked_pdo_source(node.get_publishers_info_by_topic(topic+'/bindu/position/observation'))
            if not p.session_selected or p.reference_mode != 2:
                raise RuntimeError('REPLAY_REQUIRES_SESSION_V3_TARGET_STREAM')
            if self.sample is not None and p.pdo_sample < self.sample:
                raise RuntimeError('REPLAY_PDO_REVERSED')
            if p.pdo_sample != self.sample: self.sample, self.progress = p.pdo_sample, steady
            if steady-self.progress > .05 or latest.get('error'):
                raise RuntimeError(latest.get('error', 'REPLAY_PDO_NOT_ADVANCING'))
            if prepare_context is not None and any(
                    n not in q or not math.isfinite(q[n]) or abs(q[n]-x)>.001 for n,x in prepare_context.items()):
                raise RuntimeError('PREPARE_CONTEXT_CHANGED')
            if guard is not None: guard.check(q)
            return q, v
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
            record('target', source_stamp=source, q=list(goal))
        @staticmethod
        def arrival_tolerance(name):
            if args.action == 'Prepare': return prepare_tolerance
            return robot.profile.stop_position_tolerances.get(
                name, robot.profile.feedback_tolerances['joint_position'])
        def release_and_confirm_stop(self):
            if not self.lease: return False
            call(leases, Lease.Request(operation='release', lease_id=self.lease.lease_id, epoch=self.lease.epoch))
            released = self.steady()
            wait(lambda: latest['state'][1] > released and
                 not latest['state'][0].lease_active and latest['state'][0].state != 'STOPPING',
                 robot.profile.stop_timeout+.2)
            rest_since = None
            end = self.steady()+.5
            while self.steady() < end:
                q, v = self.check(False)
                now = self.steady()
                resting = all(abs(v[n]) <= velocity_tolerance for n in NAMES)
                rest_since = now if resting and rest_since is None else rest_since if resting else None
                if rest_since is not None and now-rest_since >= .1:
                    self.lease = None
                    return True
                rclpy.spin_once(node, timeout_sec=.005)
            raise RuntimeError('REPLAY_MEASURED_STOP_NOT_CONFIRMED')

    manifest = load_library(args.library.resolve())
    guard = AxialGraspGuard(robot.urdf, manifest['grasp']) if 'grasp' in manifest and args.action != 'Prepare' else None
    port = Port()
    try:
        wait(lambda: 'state' in latest and 'pdo' in latest and leases.service_is_ready() and submit.service_is_ready(), 5.)
        rest_since = None
        end = time.monotonic()+settle_timeout
        while True:
            q, v = port.check(False)
            now = time.monotonic()
            resting = all(abs(v[n]) <= velocity_tolerance for n in NAMES)
            rest_since = now if resting and rest_since is None else rest_since if resting else None
            if rest_since is not None and now-rest_since >= settle_time: break
            if now >= end: raise RuntimeError('REPLAY_START_NOT_SETTLED')
            rclpy.spin_once(node, timeout_sec=.005)
        if args.action == 'Prepare':
            initial = dict(q)
            plan = plan_prepare(args.library, robot, initial, args.speed)
            prepare_context = {n:x for n,x in initial.items() if n not in NAMES}
            checked_at = time.monotonic()
            wait(lambda: latest['state'][1]>checked_at and latest['pdo'][1]>checked_at, 1.)
            current, _ = port.check(False)
            if any(n not in current or abs(current[n]-x)>.001 for n,x in initial.items()):
                raise RuntimeError('PREPARE_START_CHANGED_AFTER_VALIDATION')
        elif manifest.get('transition_validation_passed'):
            plan = plan_from_current(args.library, args.action, q, args.speed, args.waist_lead, args.start_tolerance)
            plan.validate_profile(robot.profile)
        record('plan', tool_present=args.action!='Prepare', route=list(getattr(plan, 'route', (args.action,))), action=args.action, duration_s=plan.duration, speed=plan.speed,
               profile_hash=robot.profile.digest, tool_id=plan.tool_id, waist_lead_s=plan.waist_lead_s,
               posture_reference_margin_rad=plan.reference_margin_rad)
        result = run_replay(plan, port, start_tolerance=min(args.start_tolerance,.001) if args.action=='Prepare' else args.start_tolerance,
                            follow_tolerance=args.follow_tolerance, velocity_tolerance=velocity_tolerance,
                            settle_time=settle_time, settle_timeout=settle_timeout)
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--action', choices=('Push', 'SideScoop', 'Flip', 'Home', 'Prepare'), required=True)
    parser.add_argument('--from-node', help='Offline route preview from a certified node; default home node')
    parser.add_argument('--speed', type=float, default=1., help='0 < speed <= 1; scales certified timing')
    parser.add_argument('--waist-lead', type=float, help='Waist command lead in real seconds; default from validated library')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--robot-config', type=Path)
    parser.add_argument('--tool-id')
    parser.add_argument('--namespace', default='bindu_snowbot')
    parser.add_argument('--vendor-namespace', default='')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--start-tolerance', type=float, default=.01)
    parser.add_argument('--follow-tolerance', type=float, default=.10)
    args = parser.parse_args(argv)
    manifest = load_library(args.library.resolve())
    if args.action == 'Prepare':
        plan = load_replay(args.library, manifest['home_node'].split(':')[0], args.speed, waist_lead_s=0.)
        if not args.execute:
            print(json.dumps(dict(action='Prepare', empty_handed=True,
                start_source='fresh measured joints on execute; full path checked before lease',
                goal_q_rad=dict(zip(plan.names,manifest['nodes'][manifest['home_node']])),
                posture_reference_margin_rad=plan.reference_margin_rad,hardware_commands=False)))
            return
    elif manifest.get('transition_validation_passed'):
        start = args.from_node or manifest['home_node']
        plan = plan_from_current(args.library, args.action, dict(zip(NAMES, manifest['nodes'][start])), args.speed, args.waist_lead)
    else:
        plan = load_replay(args.library, args.action, args.speed, args.waist_lead)
    if args.execute:
        if not args.output or (args.action != 'Prepare' and not args.tool_id):
            parser.error('--execute requires --output, and --tool-id except for empty-handed Prepare')
        if not plan.backend_ready:
            parser.error('REPLAY_VENDOR_LIMIT_ALIGNMENT_REQUIRED: backend source approval/build is pending')
        execute(args, plan)
    else:
        print(json.dumps(dict(route=list(getattr(plan, 'route', (args.action,))), action=args.action, duration_s=plan.duration, speed=args.speed,
                              waist_lead_s=plan.waist_lead_s, posture_reference_margin_rad=plan.reference_margin_rad,
                              start_q_rad=dict(zip(plan.names, plan.points[0])),
                              end_q_rad=dict(zip(plan.names, plan.points[-1])),
                              tool_id=plan.tool_id, hardware_commands=False)))


if __name__ == '__main__':
    main()

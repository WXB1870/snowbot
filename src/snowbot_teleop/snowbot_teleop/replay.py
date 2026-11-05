"""Snowbot primitive planning and port-based replay; no ROS or device startup."""
import bisect
from dataclasses import dataclass, replace
from functools import lru_cache
import hashlib
import gzip
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from snowbot_hardware.session import REPLAY_GROUPS as GROUPS, REPLAY_NAMES as NAMES


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
    reference_margin_rad: float = 0.
    empty_prepare: bool = False
    measured_start: tuple = ()
    start_reference_source: str = 'measured'
    reverse_prepare: bool = False
    velocities: tuple = ()
    accelerations: tuple = ()
    curves: tuple = ()

    @property
    def duration(self):
        return self.times[-1] / self.speed + self.waist_lead_s

    def _at(self, elapsed):
        t = min(max(0., elapsed * self.speed), self.times[-1])
        if self.curves:
            return self.state(elapsed)[0]
        i = min(bisect.bisect_right(self.times, t)-1, len(self.times)-2)
        u = (t-self.times[i])/(self.times[i+1]-self.times[i])
        return tuple(a+(b-a)*u for a, b in zip(self.points[i], self.points[i+1]))

    def state(self, elapsed):
        """q/v/a from the same accepted Bernstein polynomial."""
        if not self.curves:
            return self._at(elapsed), (), ()
        t = min(max(0., elapsed*self.speed), self.times[-1])
        index = min(bisect.bisect_right([p[0] for p in self.curves], t)-1, len(self.curves)-1)
        start, duration, source, offset, axes = self.curves[index]
        u = (offset+min(duration, max(0., t-start)))/source
        def value(controls):
            values = list(controls)
            while len(values) > 1:
                values = [(1-u)*a+u*b for a,b in zip(values, values[1:])]
            return values[0]
        q, v, a = [], [], []
        for controls in axes:
            degree = len(controls)-1
            first = [degree*(y-x)/source for x,y in zip(controls,controls[1:])]
            second = [(degree-1)*(y-x)/source for x,y in zip(first,first[1:])]
            q.append(value(controls)); v.append(value(first)*self.speed); a.append(value(second)*self.speed**2)
        return tuple(q), tuple(v), tuple(a)

    def trajectory_knots(self):
        if not self.curves or self.waist_lead_s:
            raise ValueError('REPLAY_PREPARED_CURVE_REQUIRED')
        times = [p[0]/self.speed for p in self.curves]+[self.duration]
        return tuple((t, *self.state(t)) for t in times)

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
    if not 0 <= lead <= 1.5:
        raise ValueError('REPLAY_WAIST_LEAD')
    margin = manifest.get('posture_reference_margin_rad', 0.)
    item = (manifest.get('transfers', {}) | manifest['actions'])[action]
    target = (path.parent/item['targets']).resolve()
    payload = target.read_bytes()
    data = json.loads(gzip.decompress(payload) if target.suffix == '.gz' else payload)
    if tuple(data['joint_names']) != NAMES:
        raise ValueError('REPLAY_TARGET_CONTRACT')
    samples = data['samples']
    times, points = tuple(s['t'] for s in samples), tuple(tuple(s['q']) for s in samples)
    return Replay(times, points, NAMES, manifest['tool_id'],
                  manifest['kinematic_sha256'], speed, lead, reference_margin_rad=margin,
                  velocities=tuple(tuple(s['qd']) for s in samples) if 'qd' in samples[0] else (),
                  accelerations=tuple(tuple(s['qdd']) for s in samples) if 'qdd' in samples[0] else (),
                  curves=tuple(data.get('curves', ())))



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
    def reference_margin_rad(self): return self.segments[0].reference_margin_rad

    def sample(self, elapsed):
        for segment in self.segments[:-1]:
            if elapsed <= segment.duration+self.hold_s:
                return segment.sample(min(elapsed, segment.duration))
            elapsed -= segment.duration+self.hold_s
        return self.segments[-1].sample(elapsed)

    def state(self, elapsed):
        for segment in self.segments[:-1]:
            if elapsed <= segment.duration+self.hold_s:
                return segment.state(min(elapsed, segment.duration))
            elapsed -= segment.duration+self.hold_s
        return self.segments[-1].state(elapsed)

    def validate_profile(self, profile):
        for segment in self.segments: segment.validate_profile(profile)

    def trajectory_knots(self):
        knots, offset = [], 0.
        for i, segment in enumerate(self.segments):
            current = segment.trajectory_knots()
            if knots and max(abs(a-b) for a,b in zip(knots[-1][1],current[0][1])) > 1e-8:
                raise ValueError('REPLAY_SEGMENT_DISCONTINUITY')
            knots.extend((t+offset,q,v,a) for t,q,v,a in current[bool(knots):])
            offset += segment.duration
            if i < len(self.segments)-1 and self.hold_s:
                offset += self.hold_s
                knots.append((offset, *current[-1][1:]))
        return tuple(knots)


@lru_cache(maxsize=16)
def replay_graph(path, speed, waist_lead_s, hold_s):
    manifest = load_library(Path(path).resolve())
    if manifest.get('transition_validation_passed') is not True:
        raise ValueError('REPLAY_TRANSITIONS_UNVALIDATED')
    edges, plans = {}, {}
    for edge in manifest['edges']:
        key, a, b = edge['segment'], edge['from'], edge['to']
        segment = load_replay(path, key, speed, waist_lead_s)
        plans[key] = segment
        edges.setdefault(a, []).append((b, key, segment.duration+hold_s))
    return manifest, edges, plans


def plan_from_current(path, action, measured, speed=1., waist_lead_s=None, tolerance=.01, *, hold_s=None):
    """Select a route from current measurements over the cached certified graph."""
    import heapq
    manifest = load_library(Path(path).resolve())
    hold = float(manifest.get('transition_hold_s', 3.) if hold_s is None else hold_s)
    manifest, edges, plans = replay_graph(path, speed, waist_lead_s, hold)
    if not set(NAMES) <= set(measured):
        raise ValueError('REPLAY_TRANSITION_START_LAYOUT')
    if any(not math.isfinite(measured[n]) for n in NAMES):
        raise ValueError('REPLAY_TRANSITION_START_LAYOUT')
    nodes = manifest['nodes']
    errors = {key: max(abs(measured[n]-q) for n, q in zip(NAMES, values))
              for key, values in nodes.items()}
    if not errors or min(errors.values()) > tolerance:
        raise RuntimeError('REPLAY_UNKNOWN_START: use a certified action start/end pose')
    source = min(errors, key=errors.get)
    if action == 'Flip' and source != manifest['actions']['Flip']['start_node']:
        raise RuntimeError('REPLAY_FLIP_REQUIRES_COMPLETED_SIDE_SCOOP')
    items = manifest.get('transfers', {}) | manifest['actions']
    target = manifest['home_node'] if action == 'Home' else items[action]['start_node']
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
                                home.tool_id, home.kinematics, speed,
                                reference_margin_rad=home.reference_margin_rad),)
    else:
        selected_plans = tuple(plans[k] if k in plans else load_replay(path, k, speed, waist_lead_s)
                               for k in selected)
    return ReplaySequence(selected_plans, selected, hold_s=hold)





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
    """Empty-hand entry from measured joints or an authenticated held reference."""
    import numpy as np
    from bindu_execution.interpolation import Curve, State
    from bindu_kinematics.tool_geometry import make_collision_checker
    from bindu_kinematics.bimanual_worker import prepare_limit_reference
    from snowbot_hardware.parameters import load_mode_config

    manifest = load_library(Path(path).resolve())
    home = load_replay(path, manifest['home_node'].split(':')[0], speed, waist_lead_s=0.)
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
    after = tuple(manifest['nodes'][manifest['preparation']['goal_node']])
    config = load_mode_config(robot.tool_config.with_name('teleop.json'), 'real')
    recovering = any(not profile.limits[n][0] <= measured[n] <= profile.limits[n][1] for n in NAMES)
    start = measured
    reference_source = 'measured'
    if reference is not None:
        start = prepare_limit_reference(profile, NAMES, measured, reference, dict(zip(NAMES, after)),
                                        tolerance, float(config['prepare_position_tolerance_rad']))
        reference_source = 'confirmed_pause'
    elif recovering:
        if all(max(profile.limits[n][0]-measured[n], measured[n]-profile.limits[n][1], 0.)
               <= profile.feedback_tolerances['joint_position'] for n in NAMES):
            start = {n: min(profile.limits[n][1], max(profile.limits[n][0], measured[n])) for n in NAMES}
            reference_source = 'boundary_roundoff'
        else:
            raise ValueError('PREPARE_HELD_REFERENCE_REQUIRED')
    before = tuple(start[n] for n in NAMES)
    # Preset entry has its own floor; VR retains teleop.prepare_duration.
    duration = float(manifest['preparation'].get('minimum_duration_s', 2.))
    for name, a, b in zip(NAMES, before, after):
        velocity = profile.speed(name)*.8
        acceleration = profile.acceleration(name)*.8
        jerk = profile.jerk(name)*.8
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
                  home.tool_id, home.kinematics, speed,
                  reference_margin_rad=home.reference_margin_rad, empty_prepare=True,
                  measured_start=tuple(measured[n] for n in NAMES) if recovering or reference is not None else (),
                  start_reference_source=reference_source,
                  velocities=tuple(curve.sample(t).v for t in times),
                  accelerations=tuple(curve.sample(t).a for t in times),
                  curves=((0., duration, duration, 0., tuple((a,a,a,b,b,b) for a,b in zip(before,after))),))
    plan.validate_profile(profile)
    recipe = dict(tool['demo'])
    recipe['left_grip_in_tool_m'] = (np.linalg.inv(np.array(tool['left_to_tool']).reshape(4,4)) @
        np.r_[recipe['grip_offset_m'], 1.])[:3].tolist()
    checker = make_collision_checker(robot.display_urdf, recipe, conservative_convex=True,
                                     tool_present=False, validation_only=True)
    # A rest-to-rest quintic shares one scalar progress across every joint.
    samples = []
    waypoints = ([plan.measured_start] if plan.measured_start else [])+[before, after]
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





def replay_follow_limits(config, mode):
    from bindu_runtime.teleop_deployment import apply_execution_tracking
    from bindu_contracts.teleoperation import joint_reference_limits
    selected = apply_execution_tracking(config, mode)
    default = selected.get('max_reference_error', .15) if mode == 'impedance' else .10
    overrides = selected.get('joint_reference_error_limits', {}) if mode == 'impedance' else {}
    return dict(zip(NAMES, joint_reference_limits(NAMES, default, overrides)))




def run_replay(plan, port, *, settle_timeout=5., execution_mode='position', velocity_tolerance=.005, settle_time=.2, command_ttl=.25):
    """Port owns ROS/feedback; this loop owns source time and playback progress."""
    nominal = execution_mode == 'impedance'
    attempted, acquired, sent, arrived, stopped, failure = False, False, 0, False, False, None
    try:
        # Startup READY is historical: a later settling swing can still fail
        # the servo's takeover gate. Wait without owning or commanding joints.
        print(f'WAIT_REST 连续停稳 {settle_time:g} 秒后接管；速度门限 {velocity_tolerance:g} rad/s，最多等待 {settle_timeout:g} 秒', flush=True)
        deadline = port.steady()+settle_timeout
        rest_since = previous_sample = None
        while True:
            _, velocity = port.check(False)
            sample, now = port.feedback_stamp(), port.steady()
            worst_speed, worst_joint = max((abs(velocity[n]), n) for n in plan.names)
            if worst_speed > velocity_tolerance:
                rest_since = None
            if previous_sample is None or sample > previous_sample:
                previous_sample = sample
                if worst_speed <= velocity_tolerance:
                    if rest_since is None:
                        rest_since = now
                    elif now-rest_since >= settle_time:
                        break
            if now >= deadline:
                raise RuntimeError(f'REPLAY_NOT_SETTLED:axis={worst_joint};speed={worst_speed:g};limit={velocity_tolerance:g}')
            port.wait_until(now+.02)
        print('REST_CONFIRMED 开始接管', flush=True)
        attempted = True
        port.acquire()
        acquired = True
        start = due = renewed = port.steady()
        ros_start = port.ros()
        if getattr(port, 'prepared', None) is not None:
            # One dated trajectory, followed by TTL heartbeats. The lead covers
            # admission, never changes the trajectory's recorded timing.
            port.begin(plan, ros_start+.12)
            start += .12
            due = start
            ros_start += .12
        rest_since = None
        measured_at_goal = False
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
            if now-renewed >= .5:
                port.renew()
                renewed = port.steady()
            if port.steady()-due >= command_ttl:
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
    result = dict(targets_sent=sent, measured_arrival_confirmed=arrived and measured_at_goal,
                  nominal_arrival_confirmed=arrived and nominal,
                  measured_end_error_rad={n:q[n]-x for n,x in zip(plan.names,plan.points[-1])} if arrived else None,
                  stop_confirmed=stopped, completed=arrived and failure is None,
                  error=failure, driver_started=False, snow_removal_verified=False)
    if failure:
        raise RuntimeError(json.dumps(result))
    return result

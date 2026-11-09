"""Operator-run replay of certified Snowbot atomic joint targets.

Preview is offline. --execute uses an already running session_v3 executor;
Prepare is an empty-hand entry to the side-scoop ready pose. No driver
startup, motor enabling, encoder homing, or base commands are performed.
Prepare opens/closes the grippers only after separate operator Enter confirmations.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid

from snowbot_hardware.session import (
    REPLAY_GROUPS as GROUPS, REPLAY_NAMES as NAMES,
)
from snowbot_teleop.replay import (
    kinematic_digest, load_library, load_replay,
    plan_from_current, plan_prepare, run_replay,
)


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
    from bindu_interfaces.msg import ExecutionState, MotionCommand, RuntimeEvent
    from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution, GripperControl
    from std_srvs.srv import Trigger
    from bindu_runtime.robot_setup import RobotSetup
    from bindu_runtime.common import stamp

    if args.robot_config is None:
        from bindu_contracts.resources import package_share
        args.robot_config = package_share('snowbot_bringup')/'config/robots/snowbot/robot_snowbot_real.json'
    robot = RobotSetup.load(args.robot_config)
    from snowbot_hardware.parameters import load_parameters, load_mode_config
    settings = load_parameters()
    from bindu_contracts.teleoperation import load_tool_config
    tool_config = load_tool_config(robot.tool_config)
    gripper_config = load_mode_config(robot.tool_config.with_name('grippers.json'), 'real')
    configured_margin = tool_config.get('posture_reference_margin_rad', 0.)
    if abs(configured_margin-plan.reference_margin_rad) > 1e-10:
        raise ValueError('REPLAY_REFERENCE_MARGIN_CONFIG_MISMATCH')
    if kinematic_digest(robot.urdf) != plan.kinematics:
        raise ValueError('REPLAY_KINEMATICS_MISMATCH')
    empty_handed = args.action == 'Prepare'
    if not empty_handed and args.tool_id != plan.tool_id:
        raise ValueError('REPLAY_TOOL_ID_MISMATCH')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    from snowbot_bringup.visualization.snowbot_diagnostics import DiagnosticWriter
    args.output.touch(exist_ok=False)
    log = DiagnosticWriter(args.output, sampling=False)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('snowbot_primitive_replay_'+uuid.uuid4().hex[:8])
    ns = '/'+args.namespace.strip('/')
    latest, task = {}, 'primitive-'+uuid.uuid4().hex

    def record(kind, **data):
        value = dict(kind=kind, steady=time.monotonic(), **data)
        if kind == 'trajectory_sample':
            # Bulk JSON encoding precedes the lease; disk I/O uses the existing writer.
            log.write_text('diagnostics', json.dumps(value, allow_nan=False)+'\n')
        else:
            log.write_json_line('diagnostics', value)

    def receive(kind):
        def callback(msg):
            latest[kind] = (msg, time.monotonic())
            if kind == 'state':
                record('diagnostic', stamp=msg.stamp.sec+msg.stamp.nanosec*1e-9,
                       feedback_stamp=msg.feedback_stamp.sec+msg.feedback_stamp.nanosec*1e-9,
                       reference_stamp=msg.reference.header.stamp.sec+msg.reference.header.stamp.nanosec*1e-9,
                       joint_names=list(msg.joints.name), measured_q=list(msg.joints.position),
                       measured_qd=list(msg.joints.velocity), reference_names=list(msg.reference.name),
                       reference_q=list(msg.reference.position), reference_qd=list(msg.reference.velocity),
                       reference_qdd=list(msg.reference_accelerations))
        return callback

    node.create_subscription(ExecutionState, ns+'/execution/state', receive('state'), 10)
    def event(msg):
        if msg.task_id == task:
            record('event', state=msg.state, code=msg.code)
            if msg.state == 'REPLAY_RESULT':
                latest['result'] = json.loads(msg.code)
    node.create_subscription(RuntimeEvent, ns+'/events', event, 100)
    leases = node.create_client(Lease, ns+'/execution/lease')
    submit = node.create_client(SubmitMotion, ns+'/execution/submit')
    control = node.create_client(ControlExecution, ns+'/execution/control')
    holds = node.create_client(Trigger, ns+'/execution/held_reference')
    gripper_leases = node.create_client(Lease, ns+'/gripper/lease')
    gripper_control = node.create_client(GripperControl, ns+'/gripper/control')

    def wait(predicate, timeout):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('REPLAY_WAIT_TIMEOUT')
            rclpy.spin_once(node, timeout_sec=.005)

    def call(client, request, timeout=.25):
        if not client.service_is_ready():
            raise RuntimeError('REPLAY_SERVICE_UNAVAILABLE')
        future = client.call_async(request)
        wait(future.done, timeout)
        reply = future.result()
        if not (reply.ok if hasattr(reply, 'ok') else reply.accepted):
            raise RuntimeError(reply.code)
        return reply

    class Port:
        lease = None
        def __init__(self):
            self.receipts = []

        @staticmethod
        def steady(): return time.monotonic()
        @staticmethod
        def ros(): return node.get_clock().now().nanoseconds*1e-9

        def grippers(self, operation):
            # Reuse configured endpoints and leases; the operator confirms the grasp.
            confirmed_at, confirmed_steady = self.ros(), self.steady()
            wait(lambda: latest['state'][0].stamp.sec+latest['state'][0].stamp.nanosec*1e-9 >= confirmed_at and
                 latest['state'][1] >= confirmed_steady, 1.)
            self.state()
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
                while self.steady() < end:
                    self.state()
                    if self.steady()-renewed >= .5:
                        call(gripper_leases, Lease.Request(operation='renew', lease_id=lease.lease_id, epoch=lease.epoch))
                        renewed = self.steady()
                    rclpy.spin_once(node, timeout_sec=.005)
            finally:
                released = call(gripper_leases, Lease.Request(operation='release', lease_id=lease.lease_id, epoch=lease.epoch))
                record('gripper_release', operation=operation, epoch=released.epoch)
            q, v = self.state()
            record('grippers', operation=operation, measured_q_rad={n:q[n] for n in ('arm_l_joint_8', 'arm_r_joint_8')})
            if operation == 'close':
                print('双爪关爪指令已完成，请现场确认雪铲已夹牢。')
        @staticmethod
        def wait_until(due): wait(lambda: time.monotonic() >= due, max(.1, due-time.monotonic()+.1))
        def state(self):
            state, received = latest['state']
            source = state.stamp.sec+state.stamp.nanosec*1e-9
            if not (0 <= self.steady()-received <= timing['state_max_age_s'] and
                    0 <= self.ros()-source <= timing['state_max_age_s']):
                raise RuntimeError('REPLAY_STATE_TIMEOUT')
            return dict(zip(state.joints.name, state.joints.position)), dict(zip(state.joints.name, state.joints.velocity))

        def nominal_reference(self):
            future = holds.call_async(Trigger.Request())
            wait(future.done, .25)
            reply = future.result()
            if not reply.success:
                raise RuntimeError(reply.message)
            data = json.loads(reply.message)
            self.planning_revision = data['revision']
            self.planning_context = data['instance_id']
            return data['positions']

        def prepare(self, plan):
            from trajectory_msgs.msg import JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            self.replay_identity = uuid.uuid4().hex
            # Only the final planned trajectory is serialized and certified.
            points = []
            for t,q,v,a in plan.trajectory_knots():
                at = stamp(t)
                points.append(JointTrajectoryPoint(positions=list(q), velocities=list(v),
                    accelerations=list(a), time_from_start=Duration(sec=at.sec, nanosec=at.nanosec)))
            record('trajectory', identity=self.replay_identity, joint_names=list(NAMES),
                   sample_period_s=timing['period_s'], stage='prepared_reference')
            for k in range(math.ceil(plan.duration/timing['period_s'])+1):
                t = min(k*timing['period_s'], plan.duration)
                q,v,a = plan.state(t)
                record('trajectory_sample', t=t, q=q, qd=v, qdd=a)
            # Slow preparation cannot turn queued observations into current data.
            prepared_at = self.ros()
            wait(lambda: latest['state'][0].stamp.sec+latest['state'][0].stamp.nanosec*1e-9 >= prepared_at, 1.)
            msg = MotionCommand(schema_version=2, runtime_mode='real', command_id=self.replay_identity,
                task_id=task, observation_id=getattr(self, 'planning_context', ''),
                profile_hash=robot.profile.digest,
                mode=('prepared_entry' if empty_handed else 'prepared_trajectory')+('_impedance' if args.execution_mode == 'impedance' else ''),
                resource_groups=list(GROUPS), joint_names=list(NAMES), points=points,
                expected_revision=self.planning_revision, stamp=stamp(self.ros()), valid_for=timing['command_ttl_s'])
            call(submit, SubmitMotion.Request(command=msg), settings['pico']['validation_worker_timeout_s'])

        def acquire(self):
            request = Lease.Request(operation='acquire_replay', owner=task, resources=list(GROUPS))
            while True:
                future = leases.call_async(request)
                wait(future.done, .25)
                reply = future.result()
                if reply.ok:
                    self.lease = reply
                    return
                if not reply.code.startswith('WAIT_REST'):
                    raise RuntimeError(reply.code)
                if not getattr(self, 'waiting', False):
                    print(reply.code, flush=True)
                self.waiting = True
                self.wait_until(self.steady()+timing['period_s'])

        def renew(self):
            self.receipts.append((self.steady(), 'renew', leases.call_async(Lease.Request(
                operation='renew', lease_id=self.lease.lease_id, epoch=self.lease.epoch))))

        def poll(self):
            remaining = []
            for started, operation, future in self.receipts:
                if not future.done():
                    if self.steady()-started >= timing['rpc_timeout_s']:
                        raise RuntimeError('REPLAY_RPC_TIMEOUT:'+operation)
                    remaining.append((started, operation, future))
                    continue
                reply = future.result()
                if not (reply.ok if operation=='renew' else reply.accepted):
                    raise RuntimeError(reply.code)
            self.receipts = remaining

        def send(self, source):
            msg = MotionCommand(schema_version=2, runtime_mode='real', command_id=uuid.uuid4().hex,
                task_id=task, observation_id=self.replay_identity, profile_hash=robot.profile.digest,
                lease_id=self.lease.lease_id, epoch=self.lease.epoch, mode='trajectory_heartbeat',
                resource_groups=list(GROUPS), joint_names=list(NAMES), stamp=stamp(source), valid_for=timing['command_ttl_s'])
            self.receipts.append((self.steady(), 'submit', submit.call_async(SubmitMotion.Request(command=msg))))
            record('heartbeat', source_stamp=source, phase='sent')

        def outcome(self):
            if latest.get('result') is None:
                self.state()
            return latest.get('result')

        def finish(self):
            if self.lease is None:
                return None
            reply = call(control, ControlExecution.Request(operation='replay_stop',
                lease_id=self.lease.lease_id, epoch=self.lease.epoch))
            result = json.loads(reply.code)
            end = self.steady()+robot.profile.stop_timeout+.2
            while not result['final']:
                if self.steady() >= end:
                    raise RuntimeError('REPLAY_STOP_RESULT_TIMEOUT')
                self.wait_until(self.steady()+timing['period_s'])
                result = json.loads(call(control, ControlExecution.Request(operation='replay_result',
                    lease_id=self.lease.lease_id, epoch=self.lease.epoch)).code)
            self.lease = None
            return result

    manifest = load_library(args.library.resolve())
    port = Port()
    try:
        wait(lambda: 'state' in latest and leases.service_is_ready() and submit.service_is_ready() and control.service_is_ready(), 5.)
        timing = json.loads(call(leases, Lease.Request(operation='ready', resources=list(GROUPS))).code)
        q, v = port.state()
        port.planning_revision = latest['state'][0].revision
        if args.action == 'Prepare':
            initial = dict(q)
            record('prepare_start', measured_q_rad=initial)
            reference = port.nominal_reference() if args.execution_mode == 'impedance' or any(
                max(robot.profile.limits[n][0]-initial[n], initial[n]-robot.profile.limits[n][1], 0.)
                > robot.profile.feedback_tolerances['joint_position'] for n in NAMES) else None
            plan = plan_prepare(args.library, robot, initial, args.speed, reference=reference)
            record('prepare_reference', source=plan.start_reference_source,
                   reference_q_rad=dict(zip(NAMES, plan.points[0])))
        else:
            if manifest.get('transition_validation_passed'):
                anchor = port.nominal_reference() if args.execution_mode=='impedance' else q
                plan = plan_from_current(args.library, args.action, anchor, args.speed, args.waist_lead,
                                         args.start_tolerance, hold_s=getattr(args, 'transition_hold_s', None))
                plan.validate_profile(robot.profile)
        record('plan', execution_mode=args.execution_mode, tool_present=not empty_handed, route=list(getattr(plan, 'route', (args.action,))), action=args.action, duration_s=plan.duration, speed=plan.speed,
               profile_hash=robot.profile.digest, tool_id=plan.tool_id, waist_lead_s=plan.waist_lead_s,
               posture_reference_margin_rad=plan.reference_margin_rad)
        result = run_replay(plan, port, timing=timing)
        if args.action == 'Prepare':
            confirm_prepare_grippers(port, 'open')
            confirm_prepare_grippers(port, 'close')
        record('result', **result)
        print(json.dumps(result))
    except (Exception, KeyboardInterrupt) as exc:
        record('result', completed=False, error=str(exc) or type(exc).__name__)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log.close()


def replay_action(args, parser):
    manifest = load_library(args.library.resolve())
    if args.action == 'Prepare':
        plan = load_replay(args.library, manifest['home_node'].split(':')[0], args.speed, waist_lead_s=0.)
        if not args.execute:
            print(json.dumps(dict(action='Prepare', execution_mode=args.execution_mode, empty_handed=True,
                start_source='fresh measured joints on execute; full path checked before lease',
                goal_q_rad=dict(zip(plan.names, manifest['nodes'][manifest['preparation']['goal_node']])),
                posture_reference_margin_rad=plan.reference_margin_rad,hardware_commands=False)))
            return
    elif args.execute and manifest.get('transition_validation_passed'):
        # Execution selects the route once, from the executor's current snapshot.
        plan = load_replay(args.library, manifest['home_node'].split(':')[0], args.speed, args.waist_lead)
    elif manifest.get('transition_validation_passed'):
        start = args.from_node or manifest['home_node']
        plan = plan_from_current(args.library, args.action, dict(zip(NAMES, manifest['nodes'][start])),
                                 args.speed, args.waist_lead, args.start_tolerance,
                                 hold_s=getattr(args, 'transition_hold_s', None))
    else:
        plan = load_replay(args.library, args.action, args.speed, args.waist_lead)
    if args.execute:
        if not args.output or (args.action != 'Prepare' and not args.tool_id):
            parser.error('--execute requires --output, and --tool-id except for empty-handed Prepare')
        execute(args, plan)
    else:
        print(json.dumps(dict(execution_mode=args.execution_mode, route=list(getattr(plan, 'route', (args.action,))), action=args.action, duration_s=plan.duration, speed=args.speed,
                              waist_lead_s=plan.waist_lead_s, posture_reference_margin_rad=plan.reference_margin_rad,
                              start_q_rad=dict(zip(plan.names, plan.points[0])),
                              end_q_rad=dict(zip(plan.names, plan.points[-1])),
                              tool_id=plan.tool_id, hardware_commands=False)))


def interactive_replay(args, parser):
    manifest = json.loads(args.library.read_text())
    items = manifest.get('transfers', {}) | manifest['actions']
    state, node = 'ready', args.from_node or manifest['preparation']['goal_node']
    commands = {
        'ready': {'1': ('Push', 'pushed'), '2': ('SideScoop', 'scooped')},
        'pushed': {'': ('PushReverse', 'ready')},
        'scooped': {'3': ('Flip', 'dumped'), '': ('Home', 'ready')},
        'dumped': {'': ('Home', 'ready')},
    }
    prompts = {
        'ready': '输入 1 前推或 2 侧前铲，按 Enter 执行；q 退出',
        'pushed': '前推完成，直接按 Enter 拉回；q 退出',
        'scooped': '侧前铲完成，输入 3 后按 Enter 翻倒，或直接按 Enter 回到侧前铲初始位；q 退出',
        'dumped': '翻倒完成，直接按 Enter 连续退回侧前铲初始位；q 退出',
    }
    while True:
        try:
            choice = input('\n等待输入（'+prompts[state]+'）：').strip()
        except (EOFError, KeyboardInterrupt):
            return
        if choice.lower() == 'q':
            return
        if choice not in commands[state]:
            print('当前状态不能执行该输入。')
            continue
        action, following = commands[state][choice]
        selected = argparse.Namespace(**vars(args))
        selected.action, selected.from_node = action, node
        selected.transition_hold_s = 0.
        selected.output = args.output_dir/(time.strftime('%Y%m%d-%H%M%S')+'-'+
                                          selected.action+'-'+uuid.uuid4().hex[:8]+'.jsonl')
        replay_action(selected, parser)
        state = following
        node = manifest['home_node'] if action == 'Home' else items[action]['end_node']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--action', choices=('Push', 'SideScoop', 'Flip', 'Home', 'Prepare'))
    selection.add_argument('--interactive', action='store_true', help='Staged 1/2/3 actions; empty Enter returns after each action')
    parser.add_argument('--from-node', help='Offline route preview from a certified node; default home node')
    parser.add_argument('--speed', type=float, default=1., help='0 < speed <= 1; scales certified timing')
    parser.add_argument('--waist-lead', type=float, help='Waist command lead in real seconds; default from validated library')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--execution-mode', choices=('position','impedance'), default='position',
                        help='Must match the already running backend; never switches its mode')
    parser.add_argument('--robot-config', type=Path)
    parser.add_argument('--tool-id')
    parser.add_argument('--namespace', default='bindu_snowbot')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--output-dir', type=Path, help='Interactive mode: one new JSONL log per selected action')
    parser.add_argument('--start-tolerance', type=float, default=math.radians(2),
                        help='Start-node matching tolerance in rad; default/max 2 deg')
    args = parser.parse_args(argv)
    if args.interactive:
        if not args.execute or not args.tool_id or not args.output_dir or args.output:
            parser.error('--interactive requires --execute --tool-id --output-dir; omit --output')
        return interactive_replay(args, parser)
    if args.output_dir:
        parser.error('--output-dir requires --interactive')
    return replay_action(args, parser)


if __name__ == '__main__':
    main()

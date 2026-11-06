"""Offline takeover and preset launch regression checks; no ROS or hardware."""
import json
import ast
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest

from snowbot_teleop.replay import NAMES, Replay, run_replay
from tools.deployment.deploy import executor_command


TIMING = dict(period_s=.02, command_ttl_s=.25, renewal_period_s=.5)


class Port:
    def __init__(self):
        self.now, self.acquired_at, self.sent, self.stops = 0., None, [], 0
        self.result = None
        self.delay = 0.
        self.error = None
    def steady(self): return self.now
    def ros(self): return 100.+self.now
    def wait_until(self, due): self.now = max(self.now, due)+self.delay
    def prepare(self, plan): self.now += .147  # Bulk logs and serialization precede control.
    def acquire(self): self.acquired_at = self.now
    def renew(self): pass
    def send(self, source):
        if self.error: raise RuntimeError(self.error)
        self.sent.append(source)
        if len(self.sent) == 3:
            self.result = dict(completed=True, final=True, stop_confirmed=True, error=None,
                nominal_arrival_confirmed=False, measured_arrival_confirmed=True,
                measured_end_error_rad={}, details=[])
    def outcome(self): return self.result
    def finish(self):
        self.stops += 1
        return self.result or dict(completed=False, final=True, stop_confirmed=True, error=None,
            nominal_arrival_confirmed=False, measured_arrival_confirmed=False,
            measured_end_error_rad=None, details=[])


class ReplayTakeoverTests(unittest.TestCase):
    def replay(self, port):
        return run_replay(Replay((0., .02), ((0.,)*16, (0.,)*16), NAMES, 'tool', 'hash', 1.),
                          port, timing=TIMING)

    def test_bulk_preparation_cannot_consume_first_heartbeat_budget(self):
        port = Port()
        result = self.replay(port)
        self.assertTrue(result['completed'])
        self.assertAlmostEqual(port.acquired_at, .147)
        self.assertEqual(port.sent, [100.147, 100.167, 100.187])
        self.assertEqual(port.stops, 1)

    def test_late_client_cannot_redate_old_command_and_finishes_once(self):
        port = Port(); port.delay = .3
        with self.assertRaisesRegex(RuntimeError, 'REPLAY_STREAM_LATE'):
            self.replay(port)
        self.assertEqual(port.sent, [])
        self.assertEqual(port.stops, 1)

    def test_command_timeout_survives_invalid_lease_and_stop_failure_details(self):
        port = Port(); port.error = 'INVALID_LEASE'
        port.finish = lambda: dict(completed=False, final=True, stop_confirmed=False,
            nominal_arrival_confirmed=False, measured_arrival_confirmed=False,
            measured_end_error_rad=None, error='COMMAND_TIMEOUT;STOP_UNCONFIRMED:vendor_hold',
            details=['vendor_hold'])
        with self.assertRaises(RuntimeError) as caught:
            self.replay(port)
        result = json.loads(str(caught.exception))
        self.assertTrue(result['error'].startswith('COMMAND_TIMEOUT'))
        self.assertIn('INVALID_LEASE', result['details'])
        self.assertFalse(result['completed'] or result['stop_confirmed'])

    def test_local_cancel_cause_and_external_cancel_are_not_replaced_by_invalid_lease(self):
        for cause in ('KeyboardInterrupt', 'INVALID_LEASE'):
            port = Port(); port.error = cause
            port.finish = lambda: dict(completed=False, final=True, stop_confirmed=True,
                nominal_arrival_confirmed=False, measured_arrival_confirmed=False,
                measured_end_error_rad=None, error='CANCELED', details=[])
            with self.assertRaises(RuntimeError) as caught:
                self.replay(port)
            result = json.loads(str(caught.exception))
            self.assertEqual(result['error'], 'KeyboardInterrupt' if cause == 'KeyboardInterrupt' else 'CANCELED')
            self.assertIn('CANCELED' if cause == 'KeyboardInterrupt' else 'INVALID_LEASE', result['details'])
            self.assertTrue(result['stop_confirmed'])

    def test_current_preset_entry_passes_selected_dynamics(self):
        for mode in ('position', 'impedance'):
            command = executor_command(Path('/config'), Path('/runs'), mode, 'bindu_snowbot')
            for argument in ('trajectory_max_velocity:=1.25',
                             'trajectory_max_acceleration:=4.5', 'trajectory_max_jerk:=259.2'):
                self.assertIn(argument, command)


class GripperOnlyTests(unittest.TestCase):
    def setUp(self):
        from bindu_contracts.profile import Profile
        from snowbot_hardware.parameters import load_mode_config
        from snowbot_hardware.gripper import GripperController, SIDES, GROUPS, JOINTS
        root = Path(__file__).resolve().parents[2]
        cfg = root/'src/snowbot_bringup/config/robots/snowbot'
        self.profile = Profile.load(cfg/'snowbot_real_position.json')
        config = load_mode_config(cfg/'grippers.json', 'real')
        self.clock, self.sent, self.goals = 10., [], {}
        self.q = {JOINTS[s]: config['grippers'][s]['closed'] for s in SIDES}
        self.health = dict(sample=1, operation_enabled_mask=0xfffff)
        def send(side, position, hold, sequence):
            self.sent.append((side, position, hold))
            self.goals[side] = position
        self.core = GripperController(self.profile, config, send, lambda:self.health,
            lambda:self.clock, lambda:True, enabled=True, steady=lambda:self.clock,
            require_driver_ack=False)
        self.observe()
        self.node = S(grippers=S(core=self.core), snowbot_devices=S(health=S(progress=10.)),
                      now=lambda:self.clock)
        source = root/'src/snowbot_hardware/snowbot_hardware/gripper_node.py'
        tree = ast.parse(source.read_text())
        runner = next(n for n in tree.body if getattr(n, 'name', '') == 'run_gripper_open')
        ns = dict(time=S(monotonic=lambda:self.clock), GROUPS=GROUPS, JOINTS=JOINTS, SIDES=SIDES)
        exec(compile(ast.Module(body=[runner], type_ignores=[]), str(source), 'exec'), ns)
        run_open = ns['run_gripper_open']
        self.run_open = lambda spin:run_open(self.node, spin, lambda:self.clock)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        idle = next(n for n in cls.body if getattr(n, 'name', '') == 'arms_idle')
        ns = dict(math=math, __package__='snowbot_hardware')
        exec(compile(ast.Module(body=[idle], type_ignores=[]), str(source), 'exec'), ns)
        self.arms_idle = ns['arms_idle']

    def observe(self):
        self.core.observe(self.q, dict.fromkeys(self.q, 0.), dict(left=0., right=0.), self.clock)

    def step(self, track=True):
        from snowbot_hardware.gripper import JOINTS
        self.clock += .01
        self.health['sample'] += 1
        if track:
            self.q.update({JOINTS[s]:q for s,q in self.goals.items()})
        self.observe()
        self.core.tick()

    def test_open_confirms_measured_endpoints_and_releases_without_closing(self):
        measured = self.run_open(self.step)
        self.assertEqual(measured, {s:self.core.cfg['grippers'][s]['open'] for s in ('left', 'right')})
        self.assertFalse(self.core.lease_id)
        self.assertTrue(all(c['state'] != 'STOPPING' for c in self.core.channels.values()))
        self.assertTrue(all(q == self.core.cfg['grippers'][s]['open'] for s,q,hold in self.sent))

    def test_manual_endpoint_timeout_does_not_report_success(self):
        with self.assertRaisesRegex(RuntimeError, 'MANUAL_ENDPOINT_STOPPED'):
            self.run_open(lambda:self.step(track=False))
        self.assertFalse(self.core.lease_id)

    def test_cancel_releases_lease_and_confirms_gripper_stop(self):
        first = [True]
        def spin():
            if first:
                first.pop()
                raise KeyboardInterrupt()
            self.step()
        with self.assertRaises(KeyboardInterrupt):
            self.run_open(spin)
        self.assertFalse(self.core.lease_id)
        self.assertFalse(any(c['state'] == 'STOPPING' for c in self.core.channels.values()))

    def test_stale_feedback_cannot_be_reported_as_open(self):
        def spin():
            self.clock += .01
            self.core.tick()
        with self.assertRaisesRegex(RuntimeError, 'GRIPPER_STOP_UNCONFIRMED'):
            self.run_open(spin)
        self.assertFalse(self.core.lease_id)

    def test_gripper_only_requires_valid_stationary_arms_without_arm_lease(self):
        from snowbot_hardware.session import CONTROL_NAMES, CONTROL_MASK
        velocities = dict.fromkeys(CONTROL_NAMES, 0.)
        obs = S(valid_mask=CONTROL_MASK)
        engine = S(stop_fault='', lease_id='', active=None, stopping=None)
        devices = S(health=S(require_observation=lambda:obs),
                    io=S(last_feedback=S(joint_velocities=velocities)))
        runtime = S(node=S(engine=engine, gripper_only=True, profile=self.profile), devices=devices)
        self.assertTrue(self.arms_idle(runtime))
        velocities[CONTROL_NAMES[0]] = self.profile.feedback_tolerances['joint_velocity']*2
        self.assertFalse(self.arms_idle(runtime))
        velocities[CONTROL_NAMES[0]] = 0.
        obs.valid_mask = 0
        self.assertFalse(self.arms_idle(runtime))
        obs.valid_mask = CONTROL_MASK
        engine.active = object()
        self.assertFalse(self.arms_idle(runtime))


if __name__ == '__main__':
    unittest.main()

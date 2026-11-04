"""Offline takeover and preset launch regression checks; no ROS or hardware."""
import json
from pathlib import Path
import unittest

from snowbot_bringup.teleop.snowbot_primitive_replay import NAMES, Replay, run_replay
from tools.deployment.deploy import executor_command


class Port:
    def __init__(self, speed):
        self.now = 0.
        self.speed = speed
        self.acquired_at = None
        self.sent = []
        self.released = False
        self.frozen = False
        self.fail_feedback = False
        self.reject_target = False

    def steady(self): return self.now
    def ros(self): return 100. + self.now
    def wait_until(self, due): self.now = max(self.now, due)
    def feedback_stamp(self): return 100. if self.frozen else self.ros()
    def check(self, owned):
        if self.fail_feedback:
            raise RuntimeError('STALE_FEEDBACK')
        if self.reject_target and self.sent:
            raise RuntimeError('trajectory_takeover_seed')
        velocity = dict.fromkeys(NAMES, 0.)
        velocity['body_joint_2'] = self.speed(self.now)
        return dict.fromkeys(NAMES, 0.), velocity
    def acquire(self): self.acquired_at = self.now
    def renew(self): pass
    def send(self, goal, source): self.sent.append((self.now, source))
    def release_and_confirm_stop(self):
        self.released = True
        return True
    def arrival_tolerance(self, name): return .002


class ReplayTakeoverTests(unittest.TestCase):
    def replay(self, port):
        plan = Replay((0., .02), ((0.,)*16, (0.,)*16), NAMES, 'tool', 'hash', 1.)
        return run_replay(plan, port, settle_timeout=.8, velocity_tolerance=.01)

    def test_waits_through_a_second_swing_before_acquiring(self):
        # One low-speed sample / READY must not authorize a later moving takeover.
        port = Port(lambda t: .013 if .08 <= t < .3 else .009 if t < .3 else 0.)
        result = self.replay(port)
        self.assertTrue(result['completed'])
        self.assertGreaterEqual(port.acquired_at, .5)
        self.assertEqual(port.sent[0][1], 100. + port.acquired_at)

    def test_rest_timeout_never_acquires_sends_or_releases(self):
        port = Port(lambda t: -.013)
        with self.assertRaisesRegex(RuntimeError, 'REPLAY_NOT_SETTLED.*body_joint_2'):
            self.replay(port)
        self.assertIsNone(port.acquired_at)
        self.assertFalse(port.sent or port.released)
        self.assertLess(port.now, .83)

    def test_repeated_feedback_cannot_complete_rest_window(self):
        port = Port(lambda t: 0.)
        port.frozen = True
        with self.assertRaisesRegex(RuntimeError, 'REPLAY_NOT_SETTLED'):
            self.replay(port)
        self.assertIsNone(port.acquired_at)

    def test_feedback_error_before_acquire_does_not_request_stop(self):
        port = Port(lambda t: 0.)
        port.fail_feedback = True
        with self.assertRaisesRegex(RuntimeError, 'STALE_FEEDBACK'):
            self.replay(port)
        self.assertIsNone(port.acquired_at)
        self.assertFalse(port.sent or port.released)

    def test_rejection_after_rest_still_stops_without_retry(self):
        port = Port(lambda t: 0.)
        port.reject_target = True
        with self.assertRaises(RuntimeError) as caught:
            self.replay(port)
        result = json.loads(str(caught.exception))
        self.assertIn('trajectory_takeover_seed', result['error'])
        self.assertTrue(result['stop_confirmed'] and port.released)
        self.assertEqual(len(port.sent), 1)

    def test_current_preset_entry_passes_selected_dynamics(self):
        for mode in ('position', 'impedance'):
            command = executor_command(Path('/config'), Path('/runs'), mode, 'bindu_snowbot')
            for argument in ('trajectory_max_velocity:=1.25',
                             'trajectory_max_acceleration:=4.5', 'trajectory_max_jerk:=259.2'):
                self.assertIn(argument, command)


if __name__ == '__main__':
    unittest.main()

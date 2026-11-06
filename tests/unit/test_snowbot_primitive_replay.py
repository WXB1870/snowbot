"""Offline takeover and preset launch regression checks; no ROS or hardware."""
import json
from pathlib import Path
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


if __name__ == '__main__':
    unittest.main()

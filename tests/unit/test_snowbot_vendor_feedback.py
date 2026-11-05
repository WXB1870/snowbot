"""Measured PDO feedback contract; all native checks run without ROS or devices."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
from tools.deployment import prepare_snowbot_vendor as vendor


class PositionEndpointTests(unittest.TestCase):
    def setUp(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        from snowbot_hardware.position_session import PositionSessionDevices
        self.S = S
        self.device = d = PositionSessionDevices.__new__(PositionSessionDevices)
        d.node = Mock()
        d.graph = d.node
        d.node.now.return_value = 10.
        d.health = Mock(observation=None, reference_mode=0, states={})
        d.health.observe.return_value = ({'joint':.1}, {'joint':0.}, 10.)
        d.io = Mock(invalid='')
        d.io.observe.return_value = False
        d.grippers = None
        d.receivers = dict(head='/servo_node', arms='/robot_upperMonitor')
        d.feedback_topic = '/bindu/position/observation'
        self.kind = 'snowbot_ctl_msgs/msg/PositionObservation'
        self.endpoint = S(node_namespace='/', node_name='servo_node', topic_type=self.kind)
        self.unknown = S(node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                         node_name='_NODE_NAME_UNKNOWN_', topic_type=self.kind)

    def test_startup_discards_unresolved_samples_then_accepts_the_resolved_endpoint(self):
        d = self.device
        msg = self.S(session_selected=False, pdo_sample=1)
        for infos in ([], [self.unknown]):
            d.node.get_publishers_info_by_topic.return_value = infos
            d.receive(msg)
            self.assertEqual(d.io.invalid, '')
            d.health.observe.assert_not_called()
            d.io.observe.assert_not_called()
            d.node.get_logger.assert_not_called()
        d.node.get_publishers_info_by_topic.return_value = [self.endpoint]
        d.receive(msg)
        d.io.observe.assert_called_once_with({'joint':.1}, {'joint':0.}, 10., 10., sample=1)
        self.assertEqual(d.io.invalid, '')

    def test_startup_state_waits_for_discovery_and_endpoint_errors_show_the_actual_publisher(self):
        d = self.device
        kind = 'snowbot_ctl_msgs/msg/PositionSessionState'
        self.unknown.topic_type = kind
        d.node.get_publishers_info_by_topic.return_value = [self.unknown]
        msg = self.S()
        d.receive_state('upper', '/bindu/position/upper_state', 'arms', msg)
        d.health.state.assert_not_called()
        d.node.get_logger.assert_not_called()
        d.node.get_publishers_info_by_topic.return_value = [
            self.S(node_namespace='/', node_name='robot_upperMonitor', topic_type=kind)]
        d.receive_state('upper', '/bindu/position/upper_state', 'arms', msg)
        d.health.state.assert_called_once_with('upper', msg)
        self.endpoint.node_name = 'other_servo'
        d.node.get_publishers_info_by_topic.return_value = [self.endpoint]
        d.receive(self.S())
        self.assertIn('expected=/servo_node', d.io.invalid)
        self.assertIn('other_servo', d.io.invalid)

    def test_runtime_loss_duplicates_wrong_type_and_command_endpoint_checks_still_fail(self):
        d = self.device
        for infos in ([self.endpoint, self.endpoint],
                      [self.S(node_namespace='/', node_name='servo_node', topic_type='wrong/type')]):
            d.node.get_publishers_info_by_topic.return_value = infos
            with self.assertRaisesRegex(RuntimeError, 'SNOWBOT_SESSION_ENDPOINT:'):
                d.identity(d.feedback_topic, '/servo_node', self.kind, discovering=True)
        d.node.get_publishers_info_by_topic.return_value = [self.unknown]
        with self.assertRaisesRegex(RuntimeError, 'SNOWBOT_SESSION_ENDPOINT:'):
            d.identity(d.feedback_topic, '/servo_node', self.kind)
        d.health.observation = self.S()
        for infos in ([], [self.unknown]):
            d.node.get_publishers_info_by_topic.return_value = infos
            d.receive(self.S())
            self.assertIn('SNOWBOT_SESSION_ENDPOINT:', d.io.invalid)


class VendorFeedbackTests(unittest.TestCase):
    def test_native_sampling_contract(self):
        compiler = shutil.which('c++')
        if not compiler:
            self.skipTest('C++ compiler unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp)/'feedback')
            subprocess.run([compiler, '-std=c++17', '-O2', '-pthread',
                            '-I'+str(vendor.ASSETS),
                            str(ROOT/'tests/native/test_snowbot_feedback.cpp'), '-o', binary],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=20)
            self.assertIn('snapshot checks passed', result.stdout)

    def test_combined_preparation_pins_feedback_source_and_leaves_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source, assets = root/'original', root/'assets'
            source.mkdir(); assets.mkdir()
            for package in ('servo_control', 'dual_arm_control'):
                (source/package/'include/servo_control').mkdir(parents=True)
            (source/'value').write_text('original\n')
            (source/'feedback').write_text('legacy\n')
            (assets/'position_118.patch').write_text(
                '--- a/value\n+++ b/value\n@@ -1 +1 @@\n-original\n+position\n')
            (assets/'feedback_118.patch').write_text(
                '--- a/feedback\n+++ b/feedback\n@@ -1 +1 @@\n-legacy\n+measured\n')
            (assets/'position_contract.h').write_text('// position\n')
            (assets/'feedback_snapshot.h').write_text('// feedback\n')
            with patch.object(vendor, 'ASSETS', assets), patch.object(
                    vendor, 'SITE_118_FILES', {'value': vendor.digest(source/'value')}), patch.object(
                    vendor, 'SITE_118_FEEDBACK_FILES', {'feedback': vendor.digest(source/'feedback')}):
                prepared = vendor.prepare_118(source, root/'prepared', feedback=True)
                (source/'feedback').write_text('drift\n')
                with self.assertRaisesRegex(ValueError, 'SOURCE_VERSION_MISMATCH:feedback'):
                    vendor.prepare_118(source, root/'rejected', feedback=True)
            self.assertEqual((prepared/'src/value').read_text(), 'position\n')
            self.assertEqual((prepared/'src/feedback').read_text(), 'measured\n')
            self.assertEqual((source/'value').read_text(), 'original\n')
            manifest = json.loads((prepared/'bindu_vendor_build.json').read_text())
            self.assertEqual(manifest['feedback_helper_sha256'],
                             hashlib.sha256(b'// feedback\n').hexdigest())
            self.assertFalse(manifest['hardware_started'])

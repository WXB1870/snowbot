"""Unified position-only release preparation; no device or network access."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from tools.deployment import prepare_position_session as vendor
ROOT=Path(__file__).resolve().parents[2]

class PositionReleaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline=Path(os.environ.get('SNOWBOT_POSITION_BASELINE',ROOT/'artifacts/position-session-20261021/baseline'))
        if not cls.baseline.is_dir():raise unittest.SkipTest('Local reviewed baseline is not distributed in Git')
        cls.tmp=tempfile.TemporaryDirectory();cls.work=Path(cls.tmp.name)
        cls.release=vendor.prepare(cls.baseline,cls.work/'release')
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_legacy_abi_components_and_shared_sources(self):
        m=json.loads((self.release/'position_release.json').read_text())
        self.assertEqual(set(m['components']),set(vendor.COMPONENTS))
        self.assertFalse(m['driver_started']);self.assertFalse(m['hardware_commands_sent'])
        self.assertFalse(m['latest_colleague_source_verified']);self.assertEqual(m['stop_policy'],'unconfirmed')
        for name in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
            rel='ctl_msgs/msg/'+name+'.msg';self.assertEqual((self.baseline/rel).read_bytes(),(self.release/'src'/rel).read_bytes())
        for name in m['shared_helpers']:
            for package in ('robot_control','dual_arm_control','servo_control'):
                if name=='position_session_ros.h' and package=='servo_control':continue
                self.assertEqual((vendor.ASSETS/name).read_bytes(),(self.release/'src'/package/'include'/name).read_bytes())
    def test_output_overlap_existing_and_missing_rejected(self):
        for source,output,error in ((self.baseline,self.baseline/'bad','OVERLAP'),(self.baseline,self.release,'EXISTS'),(self.work/'absent',self.work/'bad','MISSING')):
            with self.assertRaisesRegex(ValueError,error):vendor.prepare(source,output)
    def test_changed_baseline_rejected_without_output(self):
        copy=self.work/'changed';shutil.copytree(self.baseline,copy)
        (copy/'robot_control/src/robot_control/msgPublisher.cpp').write_text('colleague modification\n')
        output=self.work/'bad-baseline'
        with self.assertRaisesRegex(ValueError,'BASELINE_MISMATCH'):vendor.prepare(copy,output)
        self.assertFalse(output.exists())
    def test_symlink_rejected(self):
        copy=self.work/'links';shutil.copytree(self.baseline,copy);(copy/'link').symlink_to('robot_control')
        with self.assertRaisesRegex(ValueError,'SYMLINK'):vendor.prepare(copy,self.work/'bad-link')
    def test_unlisted_colleague_assets_preserved(self):
        copy=self.work/'extra';shutil.copytree(self.baseline,copy);(copy/'colleague.cfg').write_text('preserve this\n')
        output=vendor.prepare(copy,self.work/'with-extra',observation_only=True)
        self.assertEqual((output/'src/colleague.cfg').read_text(),'preserve this\n')
        self.assertEqual(set(json.loads((output/'position_release.json').read_text())['components']),{'observation'})
        self.assertFalse((output/'src/robot_control/include/position_session_contract.h').exists())
    def test_original_paths_and_buses_invariant(self):
        subprocess.run(['python3',str(ROOT/'tests/integration/validate_position_invariants.py'),'--baseline',str(self.baseline),'--source',str(self.release/'src')],check=True,capture_output=True,text=True)
    def test_actual_production_native_contracts(self):
        subprocess.run(['python3',str(ROOT/'tests/integration/validate_position_native.py'),'--source',str(self.release/'src')],check=True,capture_output=True,text=True)
        subprocess.run(['python3',str(ROOT/'tests/integration/validate_position_model.py'),'--source',str(self.release/'src')],check=True,capture_output=True,text=True)

if __name__=='__main__':unittest.main()

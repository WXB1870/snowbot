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
        self.assertEqual(set(m['components']),{*vendor.COMPONENTS,'pause_hold','drive_maintenance','runtime_timing'})
        self.assertFalse(m['driver_started']);self.assertFalse(m['hardware_commands_sent'])
        self.assertFalse(m['latest_colleague_source_verified']);self.assertEqual(m['stop_policy'],'unconfirmed')
        for name in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
            rel='ctl_msgs/msg/'+name+'.msg';self.assertEqual((self.baseline/rel).read_bytes(),(self.release/'src'/rel).read_bytes())
        for name in m['shared_helpers']:
            for package in ('robot_control','dual_arm_control','servo_control'):
                if name=='position_session_ros.h' and package=='servo_control':continue
                self.assertEqual((vendor.ASSETS/name).read_bytes(),(self.release/'src'/package/'include'/name).read_bytes())
    def test_optional_impedance_is_explicit_and_stays_disabled(self):
        output=vendor.prepare(self.baseline,self.work/'with-impedance',with_impedance=True)
        manifest=json.loads((output/'position_release.json').read_text())
        self.assertEqual(set(manifest['components']),{*(set(vendor.COMPONENTS)-{'target'}),'impedance'})
        self.assertFalse(manifest['target_stream_available'])
        self.assertNotIn('target_stream', (output/'src/servo_control/src/ros_controller/rosLoop.cpp').read_text())
        self.assertEqual(manifest['execution_mode'],'position')
        self.assertTrue(manifest['impedance_available'])
        self.assertFalse(manifest['impedance_calibration_confirmed'])
        self.assertEqual(manifest['stop_policy'],'unconfirmed')
        default=json.loads((self.release/'position_release.json').read_text())
        self.assertFalse(default['impedance_available'])
        self.assertTrue(default['target_stream_available'])
        self.assertEqual((default['target_wire_version'],default['target_reference_mode']),(5,2))
        self.assertEqual(default['session_wire_version'],3)
        self.assertEqual(default['target_prepare_ms'],10.)
        self.assertEqual((self.release/'src/servo_control/src/impedance/cartesian_impedance_controller.cpp').read_bytes(),
                         (self.baseline/'servo_control/src/impedance/cartesian_impedance_controller.cpp').read_bytes())
        for name in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
            rel='ctl_msgs/msg/'+name+'.msg'
            self.assertEqual((self.baseline/rel).read_bytes(),(output/'src'/rel).read_bytes())
        with self.assertRaisesRegex(ValueError,'IMPEDANCE_REQUIRES_FULL_SESSION'):
            vendor.prepare(self.baseline,self.work/'invalid-impedance',with_impedance=True,observation_only=True)

    def test_rigid_model_is_explicit_hashed_and_incompatible_modes_rejected(self):
        bindu=ROOT.parent/'galbot/bindu'
        urdf=bindu/'src/hardware/bindu_description/urdf/snowbot_control.urdf'
        tool=bindu/'src/integration/bindu_runtime/config/robots/snowbot/bimanual_snowbot_real.json'
        if not urdf.is_file():self.skipTest('Canonical Bindu kinematic model unavailable')
        output=vendor.prepare(self.baseline,self.work/'rigid',prediction_urdf=urdf,prediction_tool=tool)
        manifest=json.loads((output/'position_release.json').read_text())
        self.assertTrue(manifest['rigid_prediction']['enabled'])
        self.assertEqual(manifest['rigid_prediction']['recipe_version'],3)
        self.assertEqual(manifest['rigid_prediction']['urdf_sha256'],vendor.digest(urdf))
        self.assertEqual(len(manifest['rigid_prediction']['names']),16)
        self.assertIn('prediction',manifest['components'])
        self.assertTrue((output/'src/servo_control/include/bindu_execution_native/prediction_model.hpp').is_file())
        self.assertIn('TARGET_PREDICTION ',(output/'src/servo_control/src/ros_controller/rosLoop.cpp').read_text())
        self.assertFalse(list((output/'src').rglob('*.orig')))
        self.assertFalse(json.loads((self.release/'position_release.json').read_text())['rigid_prediction']['enabled'])
        for options in ({'observation_only':True},{'with_impedance':True}):
            with self.assertRaisesRegex(ValueError,'REQUIRES_TARGET_STREAM'):
                vendor.prepare(self.baseline,self.work/'bad-rigid',prediction_urdf=urdf,prediction_tool=tool,**options)
        with self.assertRaisesRegex(ValueError,'MODEL_AND_TOOL_REQUIRED'):
            vendor.prepare(self.baseline,self.work/'bad-model',prediction_urdf=urdf)

    def test_waist_units_are_explicit_preserve_positions_and_use_actual_pdo_writer(self):
        from tests.integration.validate_position_native import write_pdo_check, write_drive_cycle_check
        compiler=shutil.which('c++')
        if not compiler:self.skipTest('C++ compiler unavailable')
        for index,gain in enumerate((0.,.05)):
            output=vendor.prepare(self.baseline,self.work/f'waist-{index}',
                waist_velocity_ratio=(25.25,25.25),waist_feedforward_gain=gain)
            manifest=json.loads((output/'position_release.json').read_text())
            self.assertIn('waist_units',manifest['components'])
            self.assertFalse(manifest['waist_units']['position_scale_changed'])
            self.assertFalse(manifest['waist_units']['physical_units_verified'])
            self.assertEqual(manifest['waist_units']['motor_per_position_velocity'],[25.25,25.25])
            cpp=write_pdo_check(output/'src',output)
            driver=(output/'src/servo_control/src/servo_control/ethercat_driver.cpp').read_text()
            mapping=driver[driver.index('void EtherCATDriver::cspPDOConfig('):driver.index('// ========== 传感器 EEPROM')]
            mapping=mapping.replace('EtherCATDriver::cspPDOConfig','MappingProbe::cspPDOConfig')
            mock=r'''
#include <cstdio>
#include <cstring>
constexpr bool FALSE=false; constexpr int EC_TIMEOUTSAFE=1;
struct Slave {uint16_t aliasadr=0;}; Slave ec_slave[32];
uint32_t velocity_mapping[32]{};
int ec_SDOwrite(int slave,unsigned index,unsigned sub,bool,int size,void* value,int){
 if(index==0x1600 && sub==3){assert(size==4);std::memcpy(&velocity_mapping[slave],value,4);}return 1;
}
struct MappingProbe {void cspPDOConfig(int);};
'''
            main=r'''
int main(){check_position_pdo_output();MappingProbe probe;
 for(unsigned alias=17;alias<=20;++alias){ec_slave[alias].aliasadr=alias;probe.cspPDOConfig(alias);
  const unsigned expected=alias>=19 && position_session::waist_units::feedforward_gain>0 ? 0x60b1 : 0x60ff;
  assert(velocity_mapping[alias]==(expected<<16|0x20));}
}
'''
            cpp.write_text(cpp.read_text()+mock+mapping+main)
            include=output/'src/servo_control/include'
            binary=output/'pdo-memory-check'
            subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(include),
                str(cpp),'-o',str(binary)],check=True,capture_output=True,text=True)
            subprocess.run([str(binary)],check=True,capture_output=True,text=True)
            cpp=write_drive_cycle_check(output/'src',output,ROOT/'tests/native')
            binary=output/'drive-memory-check'
            subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-variable','-I',str(include),
                '-I',str(output),str(cpp),'-o',str(binary)],check=True,capture_output=True,text=True)
            subprocess.run([str(binary)],check=True,capture_output=True,text=True)

    def test_waist_units_reject_unknown_ratios_or_incompatible_modes(self):
        invalid=({'waist_feedforward_gain':.05}, {'waist_velocity_ratio':(25.25,)},
            {'waist_velocity_ratio':(float('nan'),25.25)},
            {'waist_velocity_ratio':(25.25,25.25),'waist_feedforward_gain':.2},
            {'waist_velocity_ratio':(25.25,25.25),'observation_only':True},
            {'waist_velocity_ratio':(25.25,25.25),'with_impedance':True})
        for options in invalid:
            with self.subTest(options=options),self.assertRaisesRegex(ValueError,'WAIST_'):
                vendor.prepare(self.baseline,self.work/'invalid-waist',**options)

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

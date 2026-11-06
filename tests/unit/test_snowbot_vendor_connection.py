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
        cls.baseline=Path(os.environ.get('SNOWBOT_POSITION_BASELINE',ROOT/'vendor/snowbot/original'))
        if not cls.baseline.is_dir():raise RuntimeError('Reviewed vendor baseline is missing')
        cls.tmp=tempfile.TemporaryDirectory();cls.work=Path(cls.tmp.name)
        cls.release=vendor.prepare(cls.baseline,cls.work/'release')
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_named_release_outputs_reject_ambiguous_or_escaping_names(self):
        self.assertEqual(vendor.release_output('20261026-encoder-6dc74af'),
                         ROOT/'releases/20261026-encoder-6dc74af')
        for name in ('', '.', '..', '../escape', '/tmp/escape', 'a/b', 'latest', 'CURRENT', 'final', 'x'*97):
            with self.subTest(name=name),self.assertRaisesRegex(ValueError,'RELEASE_ID_INVALID'):
                vendor.release_output(name)

    def test_legacy_abi_components_and_shared_sources(self):
        m=json.loads((self.release/'position_release.json').read_text())
        self.assertEqual(set(m['components']),{*vendor.COMPONENTS,'drive_maintenance','runtime_timing','waist_feedback','deployed_vendor','shutdown'})
        self.assertFalse(m['driver_started']);self.assertFalse(m['hardware_commands_sent'])
        self.assertEqual(m['preparation']['artifact_role'],'generated_candidate')
        self.assertFalse(m['preparation']['deployment_verified'])
        self.assertFalse(m['preparation']['physical_acceptance'])
        self.assertEqual(m['preparation']['tool_sha256'],vendor.digest(Path(vendor.__file__)))
        self.assertEqual(m['preparation']['snowbot']['repository'],str(ROOT))
        self.assertEqual(m['feedback_publication']['mode'],'latest_only_worker')
        self.assertEqual(m['feedback_publication']['max_sample_age_ms'],20)
        self.assertEqual(m['feedback_publication']['timestamp'],'acquisition_start')
        self.assertFalse(m['latest_colleague_source_verified']);self.assertEqual(m['stop_policy'],'unconfirmed')
        for name in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
            rel='ctl_msgs/msg/'+name+'.msg';self.assertEqual((self.baseline/rel).read_bytes(),(self.release/'src'/rel).read_bytes())
        for name in m['shared_helpers']:
            for package in ('robot_control','dual_arm_control','servo_control'):
                if name=='position_session_ros.h' and package=='servo_control':continue
                self.assertEqual((vendor.ASSETS/name).read_bytes(),(self.release/'src'/package/'include'/name).read_bytes())
    def test_committed_source_pair_and_generated_build_dependencies(self):
        paired=json.loads((ROOT/'dependencies.repos').read_text())['repositories']['bindu']
        self.assertEqual(paired['type'],'git')
        self.assertEqual(paired['url'],'https://github.com/WXB1870/bindu.git')
        self.assertRegex(paired['version'],r'^[0-9a-f]{40}$')
        manifest=json.loads((self.release/'position_release.json').read_text())
        self.assertEqual(manifest['preparation']['recommended_bindu_source'],paired)
        self.assertEqual(manifest['preparation']['source_pairing_sha256'],vendor.digest(ROOT/'dependencies.repos'))
        self.assertEqual(manifest['preparation']['deployment_reference']['following_guard_rad'],.25)
        self.assertEqual(manifest['preparation']['default_policy']['following_guard_rad'],.5)
        self.assertFalse(manifest['waist_units']['enabled'])
        self.assertEqual(manifest['csp_encoder']['position_counts_per_rev'],262144)
        # The preserved original requires this external message; the generated
        # full session already removes its unused include and build dependency.
        for name in ('CMakeLists.txt','package.xml','include/ros_controller/rosLoop.h'):
            self.assertIn('kinematic_pose_msgs',(self.baseline/'servo_control'/name).read_text())
            self.assertNotIn('kinematic_pose_msgs',(self.release/'src/servo_control'/name).read_text())
        for name in vendor.NATIVE_FILES:
            folder='include' if name.endswith('.hpp') else 'src'
            copied=self.release/'src/servo_control'/folder/'bindu_execution_native'/name
            self.assertEqual(vendor.digest(copied),manifest['native_execution'][name])

    def test_optional_impedance_is_explicit_and_stays_disabled(self):
        output=vendor.prepare(self.baseline,self.work/'with-impedance',with_impedance=True)
        manifest=json.loads((output/'position_release.json').read_text())
        self.assertEqual(set(manifest['components']),{*vendor.COMPONENTS,'drive_maintenance','runtime_timing','waist_feedback','deployed_vendor','joint_impedance_118','impedance','shutdown','shutdown_impedance'})
        self.assertTrue(manifest['target_stream_available'])
        self.assertTrue(manifest['feedback_publication']['isolated'])
        self.assertEqual(manifest['feedback_publication']['mode'],'latest_only_worker')
        identity=json.loads((ROOT/'vendor/snowbot/impedance_118.json').read_text())
        self.assertEqual(manifest['impedance_source'],identity)
        self.assertEqual(vendor.digest(output/'src/ctl_msgs/config/impedance_params.yaml'),
                         identity['files']['ctl_msgs/config/impedance_params.yaml']['source_sha256'])
        self.assertEqual(manifest['execution_mode'],'position')
        self.assertTrue(manifest['impedance_available'])
        self.assertNotIn('impedance_calibration_confirmed',manifest)
        servo=(output/'src/servo_control/src/ros_controller/rosLoop.cpp').read_text()
        self.assertNotIn('impedance_calibration_confirmed',servo)
        self.assertEqual(manifest['stop_policy'],'unconfirmed')
        default=json.loads((self.release/'position_release.json').read_text())
        self.assertFalse(default['impedance_available'])
        self.assertTrue(default['target_stream_available'])
        self.assertEqual((default['target_wire_version'],default['target_reference_mode']),(5,2))
        self.assertEqual(default['session_wire_version'],5)
        self.assertEqual(default['target_prepare_ms'],10.)
        for rel,hashes in identity['files'].items():
            self.assertEqual(vendor.digest(self.release/'src'/rel),hashes['input_sha256'])
        for name in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
            rel='ctl_msgs/msg/'+name+'.msg'
            self.assertEqual((self.baseline/rel).read_bytes(),(output/'src'/rel).read_bytes())
        with self.assertRaisesRegex(ValueError,'IMPEDANCE_REQUIRES_FULL_SESSION'):
            vendor.prepare(self.baseline,self.work/'invalid-impedance',with_impedance=True,observation_only=True)

    def test_rigid_model_is_explicit_hashed_and_incompatible_modes_rejected(self):
        urdf=ROOT/'src/snowbot_description/urdf/snowbot_control.urdf'
        tool=ROOT/'src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_real.json'
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

    def test_feedback_ratio_read_and_actual_pdo_writer(self):
        from tests.integration.validate_position_native import write_pdo_check, write_drive_cycle_check
        compiler=shutil.which('c++')
        if not compiler:self.skipTest('C++ compiler unavailable')
        output=self.release;include=output/'src/servo_control/include'
        manifest=json.loads((output/'position_release.json').read_text())
        self.assertTrue(manifest['waist_feedback']['enabled'])
        self.assertFalse(manifest['waist_feedback']['velocity_command_scaled'])
        self.assertEqual(manifest['waist_feedback']['provisional_ratio'],[101,101])
        driver=(output/'src/servo_control/src/servo_control/ethercat_driver.cpp').read_text()
        self.assertLess(driver.index('ec_SDOread(slave,0x200E,2,'),driver.index('// === 2. 电机 PDO 配置 (PREOP) ==='))
        cpp=write_pdo_check(output/'src',output)
        cpp.write_text(cpp.read_text()+'\nint main(){check_position_pdo_output();}\n')
        for name,source in (('pdo',cpp),('cycle',write_drive_cycle_check(output/'src',output,ROOT/'tests/native'))):
            binary=output/('feedback-'+name)
            subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-variable','-pthread','-I',str(include),
                '-I',str(output),str(source),'-o',str(binary)],check=True,capture_output=True,text=True)
            subprocess.run([str(binary)],check=True,capture_output=True,text=True)

    def test_waist_units_are_explicit_and_use_actual_pdo_writer(self):
        from tests.integration.validate_position_native import write_pdo_check, write_drive_cycle_check
        compiler=shutil.which('c++')
        if not compiler:self.skipTest('C++ compiler unavailable')
        for index,gain in enumerate((0.,.05)):
            output=vendor.prepare(self.baseline,self.work/f'waist-{index}',
                waist_velocity_ratio=(25.25,25.25),waist_feedforward_gain=gain)
            manifest=json.loads((output/'position_release.json').read_text())
            self.assertIn('waist_units',manifest['components'])
            self.assertTrue(manifest['waist_units']['position_scale_changed'])
            self.assertEqual(manifest['csp_encoder']['position_counts_per_rev'],262144)
            self.assertEqual(manifest['csp_encoder']['speed_counts_per_rev'],65536)
            self.assertFalse(manifest['waist_units']['physical_units_verified'])
            self.assertEqual(manifest['waist_units']['motor_per_position_velocity'],[25.25,25.25])
            self.assertTrue(manifest['waist_units']['velocity_command_scaled'])
            self.assertEqual(manifest['waist_units']['velocity_object'],'0x60B1' if gain else '0x60FF')
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
    def test_actual_gripper_elapsed_clock_matches_command_interval(self):
        compiler=shutil.which('c++')
        if not compiler:self.skipTest('C++ compiler unavailable')
        def method(source,name):
            start=source.index('void ServoNode::'+name+'(')
            opening=source.index('{',start);depth=1;end=opening+1
            while depth:
                depth+=(source[end]=='{')-(source[end]=='}');end+=1
            return source[start:end]
        prefix=r'''#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <vector>
namespace std_msgs {namespace msg {struct Float64MultiArray {
 using SharedPtr=std::shared_ptr<Float64MultiArray>;std::vector<double> data;
};}}
static bool EtherCAT_driveResetBusy(){return false;}
static void EtherCAT_setMotorEnable(bool){}
static void EtherCAT_setSingleMotorEnable(int,bool){}
static double getGripperFbPos(int){return 0.;}
static void setLeftGripperCmd(double,double,double,double,double){}
static void setRightGripperCmd(double,double,double,double,double){}
struct Clock {int64_t ns;double seconds()const{return ns*1e-9;}int64_t nanoseconds()const{return ns;}};
struct ServoNode {bool shutdown_mode_=false;int64_t clock_ns=1000000000;
 Clock now()const{return {clock_ns};}
 void gripper_cmd_apply(const std_msgs::msg::Float64MultiArray::SharedPtr,bool);
 void gripper_runin();
'''
        for side in ('left','right'):
            prefix+=f' bool g_{side}_inited=false,g_{side}_ifRunin=false;int g_{side}_winlen=0,g_{side}_step=0;\n'
            prefix+=' double '+','.join('g_'+side+'_'+name+'=0' for name in
                ('pos_target','pos_start','pos_curr','time_last','spd','tor','kp','kd'))+';\n'
        prefix+='};\n'
        checks=r'''
int main(){for(bool left:{false,true}){
 ServoNode n;auto msg=std::make_shared<std_msgs::msg::Float64MultiArray>();
 msg->data={.1,0.,0.,100.,30.};n.gripper_cmd_apply(msg,left);
 for(int i=0;i<20;++i)n.gripper_runin();
 n.clock_ns+=20000000;msg->data[0]=.2;n.gripper_cmd_apply(msg,left);
 if((left?n.g_left_winlen:n.g_right_winlen)!=20)return 1;
 for(int i=0;i<10;++i)n.gripper_runin();
 if(std::abs((left?n.g_left_pos_curr:n.g_right_pos_curr)-.15)>1e-9)return 2;
 for(int i=0;i<10;++i)n.gripper_runin();
 if(std::abs((left?n.g_left_pos_curr:n.g_right_pos_curr)-.2)>1e-9)return 3;
 n.clock_ns+=2000000000;msg->data[0]=.3;n.gripper_cmd_apply(msg,left);
 if((left?n.g_left_winlen:n.g_right_winlen)!=500)return 4;
 }return 0;}
'''
        for label,folder,expected in [('baseline',self.baseline,1),('candidate',self.release/'src',0)]:
            source=(folder/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
            cpp=self.work/(label+'-gripper-clock.cpp');binary=self.work/(label+'-gripper-clock')
            cpp.write_text(prefix+method(source,'gripper_cmd_apply')+'\n'+method(source,'gripper_runin')+checks)
            subprocess.run([compiler,'-std=c++17',str(cpp),'-o',str(binary)],check=True,capture_output=True,text=True)
            result=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,expected,result.stdout+result.stderr)

    def test_actual_production_native_contracts(self):
        subprocess.run(['python3',str(ROOT/'tests/integration/validate_position_native.py'),'--source',str(self.release/'src')],check=True,capture_output=True,text=True)
        subprocess.run(['python3',str(ROOT/'tests/integration/validate_position_model.py'),'--source',str(self.release/'src')],check=True,capture_output=True,text=True)



class DeploymentEntryTests(unittest.TestCase):
    def test_home_is_opt_in_and_terminal_two_only(self):
        from unittest.mock import patch
        from tools.deployment import deploy
        with patch.object(deploy, 'locked_bindu', return_value=Path('/bindu')), \
                patch.object(deploy, 'operator', return_value=0) as operator:
            for extra, enabled in (([], False), (['--home'], True)):
                self.assertEqual(deploy.main(['operator', '--frontend', 'vr', *extra]), 0)
                self.assertEqual(operator.call_args.kwargs, {'home': enabled})
            self.assertEqual(deploy.main(['services', '--home']), 2)

    def test_home_waits_for_powered_hold_and_measured_and_nominal_arrival(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        from bindu_contracts.contracts import Motion
        from snowbot_hardware.position_session import PositionSessionDevices
        from snowbot_hardware.protocol import GROUPS
        names = tuple(n for g in ('waist', 'left_arm', 'right_arm') for n in GROUPS[g])
        devices = PositionSessionDevices.__new__(PositionSessionDevices)
        devices.node = S(now=Mock(return_value=10.), closing=False, event=Mock(), get_logger=Mock())
        devices.io = S(stop_failures=(), profile=S(max_transition_duration=30.,
            feedback_tolerances={'joint_position': .01}, stop_position_tolerances={names[0]: .02}),
            stop_arrival_joints=GROUPS['waist'], replay_position_tolerances=dict.fromkeys(names, .035),
            sampled_reference=Mock(return_value=(10., {}, {}, {})))
        devices.startup_hold_state = 'holding'; devices.target_fault = ''; devices.takeover_pending = None
        devices.startup_motion = Motion('hold', 'lease', 7, 'profile', 'startup_hold', '', 'joint_target', '',
            10., .2, names=names, positions=(.2,)*16, schema_version=2,
            resource_groups=('waist', 'left_arm', 'right_arm'), runtime_mode='real')
        engine = Mock(stop_fault='', stopping=True)
        devices.advance_startup_hold(engine)
        engine.submit.assert_not_called()
        engine.stopping = None
        devices.advance_startup_hold(engine)
        self.assertEqual(devices.startup_hold_state, 'homing')
        engine.references_arrived.return_value = False
        devices.advance_startup_hold(engine)
        motion = engine.submit.call_args.args[0]
        self.assertEqual((motion.lease_id, motion.epoch, motion.positions), ('lease', 7, (0.,)*16))
        self.assertEqual(motion.names, names)
        arrival = engine.references_arrived.call_args.kwargs['position_tolerances']
        self.assertEqual((arrival[names[0]], arrival[names[1]], arrival[names[2]]), (.02, .01, .035))
        engine.references_arrived.return_value = True
        devices.node.now.return_value = 10.02
        devices.advance_startup_hold(engine)
        engine.halt.assert_not_called()  # A measured zero alone cannot finish an unfinished reference.
        devices.io.sampled_reference.return_value = (10.04, dict.fromkeys(names, 0.), {}, {})
        devices.node.now.return_value = 10.04
        devices.advance_startup_hold(engine)
        self.assertEqual(devices.startup_hold_state, 'stopping')
        engine.halt.assert_called_once_with(10.04, revoke=True)
        engine.stopping = True
        devices.advance_startup_hold(engine)
        self.assertEqual(devices.startup_hold_state, 'stopping')
        engine.stopping = None
        devices.advance_startup_hold(engine)
        self.assertEqual(devices.startup_hold_state, 'ready')
        devices.startup_hold_state = 'homing'; devices.node.now.return_value = 41.
        devices.advance_startup_hold(engine)
        self.assertEqual(devices.startup_hold_state, 'failed')
        engine.halt.assert_called_with(41., 'STARTUP_HOME_FAILED', revoke=True)
        devices.startup_hold_state = 'homing'; devices.node.closing = True
        count = engine.submit.call_count
        devices.advance_startup_hold(engine)
        self.assertEqual(engine.submit.call_count, count)

    def test_vendor_build_uses_selected_ros_without_starting_nodes(self):
        from unittest.mock import patch
        from tools.deployment import deploy
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp); vendor = workspace/'vendor'
            for package, executable in zip(deploy.TARGETS, ('servo_node', 'dual_arm_control_node', 'robot_control_node')):
                binary = vendor/'install'/package/'lib'/package/executable
                binary.parent.mkdir(parents=True); binary.touch()
            with patch.object(deploy.subprocess, 'run') as run:
                deploy.vendor_build(vendor, workspace, 'humble')
                command = run.call_args.args[0]
                self.assertIn('source /opt/ros/humble/setup.bash', command[4])
                self.assertIn('build', command)
                self.assertNotIn('ros2', command)


    def test_native_platforms_select_matching_ros_and_pinocchio(self):
        from unittest.mock import patch
        from tools.deployment import deploy
        for machine, release, distro, pinocchio in (
                ('x86_64', '24.04', 'jazzy', '4.1.0'),
                ('aarch64', '22.04', 'humble', '3.9.0')):
            with self.subTest(machine=machine), \
                    patch('platform.system', return_value='Linux'), \
                    patch('platform.machine', return_value=machine), \
                    patch('platform.freedesktop_os_release', return_value=dict(ID='ubuntu', VERSION_ID=release), create=True), \
                    patch.object(Path, 'is_file', autospec=True, return_value=True) as exists:
                self.assertEqual(deploy.require_platform(), distro)
                checked = [str(call.args[0]) for call in exists.call_args_list]
                self.assertIn(f'/opt/ros/{distro}/setup.bash', checked)
                self.assertIn(f'/opt/openrobots/lib/libpinocchio_default.so.{pinocchio}', checked)
                for missing in ('/usr/include/nlohmann/json.hpp',
                                '/usr/lib/python3/dist-packages/colcon_ros/task/ament_python/build.py'):
                    exists.side_effect = lambda path: str(path) != missing
                    with self.assertRaisesRegex(ValueError, 'prerequisites'):
                        deploy.require_platform()
        with patch('platform.system', return_value='Linux'), \
                patch('platform.machine', return_value='aarch64'), \
                patch('platform.freedesktop_os_release', return_value=dict(ID='ubuntu', VERSION_ID='24.04'), create=True):
            with self.assertRaisesRegex(ValueError, 'requires'):
                deploy.require_platform()


    def primitive_menu(self, choices, replay):
        from unittest.mock import patch
        from contextlib import redirect_stdout
        from io import StringIO
        from types import SimpleNamespace as S
        import snowbot_bringup.runtime.snowbot_primitive_replay as module
        library = ROOT/'src/snowbot_bringup/config/robots/snowbot/primitive-library/manifest.json'
        args = S(library=library, from_node=None, output_dir=Path('/unused'), execution_mode='impedance')
        with patch('builtins.input', side_effect=choices) as choose, \
                patch.object(module, 'replay_action', side_effect=replay) as run, \
                redirect_stdout(StringIO()):
            module.interactive_replay(args, None)
        return choose, run

    def test_primitive_menu_requires_explicit_returns_and_blocks_wrong_stage_keys(self):
        calls = []
        self.primitive_menu(['3','1','2','3','','3','2','1','3','1','2','','q'],
                            lambda args, _:calls.append((args.action, args.from_node, args.execution_mode, args.transition_hold_s)))
        self.assertEqual([(action, node) for action,node,_,_ in calls], [
            ('Push','Push:start'), ('PushReverse','Push:end'), ('SideScoop','Push:start'),
            ('Flip','Flip:start'), ('Home','Dump:end')])
        self.assertTrue(all(mode=='impedance' and hold==0. for _,_,mode,hold in calls))

    def test_primitive_menu_scoop_can_return_without_flip_and_failure_never_advances(self):
        calls = []
        self.primitive_menu(['2','','1','','q'], lambda args, _:calls.append(args.action))
        self.assertEqual(calls, ['SideScoop','Home','Push','PushReverse'])
        def fail(args, parser):
            raise RuntimeError('STOP_UNCONFIRMED')
        with self.assertRaisesRegex(RuntimeError, 'STOP_UNCONFIRMED'):
            self.primitive_menu(['1','2'], fail)

    def test_primitive_menu_routes_reuse_exact_segments_with_no_artificial_pause(self):
        from snowbot_teleop.replay import NAMES, plan_from_current
        library = ROOT/'src/snowbot_bringup/config/robots/snowbot/primitive-library/manifest.json'
        m = json.loads(library.read_text())
        self.assertEqual(m['preparation']['goal_node'], 'Push:start')
        self.assertEqual(m['actions']['SideScoop']['end_node'], 'Flip:start')
        cases = [
            ('Push:start','Push',('Push',),7.8),
            ('Push:end','PushReverse',('PushReverse',),7.82),
            ('Push:start','SideScoop',('BridgeForward','SideScoop'),12.5),
            ('Flip:start','Flip',('Flip',),8.12),
            ('Flip:start','Home',('ScoopReturn',),4.),
            ('Dump:end','Home',('DumpReturn',),7.08),
        ]
        for start, action, route, duration in cases:
            with self.subTest(start=start, action=action):
                plan = plan_from_current(library, action, dict(zip(NAMES, m['nodes'][start])), hold_s=0.)
                self.assertEqual(plan.route, route)
                self.assertEqual(plan.hold_s, 0.)
                self.assertAlmostEqual(plan.duration, duration)
                for a,b in zip(plan.segments,plan.segments[1:]):
                    self.assertLess(max(abs(x-y) for x,y in zip(a.points[-1], b.points[0])), 1e-12)
                if len(plan.segments)>1:
                    boundary=plan.segments[0].duration
                    actual=plan.sample(boundary+.02);following=plan.segments[1].sample(.02)
                    self.assertLess(max(abs(a-b) for a,b in zip(actual,following)), 1e-12)
                    self.assertNotEqual(actual, plan.segments[0].points[-1])

    def test_base_has_three_native_layers_and_mode_override(self):
        from unittest.mock import patch
        from tools.deployment.deploy import base_commands
        with patch.dict(os.environ, {'VIRTUAL_ENV':'upper', 'LD_LIBRARY_PATH':'upper-pin'}):
            for mode in ('impedance', 'position'):
                plan = base_commands(Path('/deployment/artifacts/robot'), mode)
                self.assertEqual([name for name, _, _ in plan], ['servo','middle','upper'])
                self.assertEqual(plan[0][1][-2:], ['-p', 'execution_mode:='+mode])
                for _, _, env in plan:
                    self.assertNotIn('VIRTUAL_ENV', env)
                    self.assertNotIn('LD_LIBRARY_PATH', env)
            self.assertEqual(base_commands(Path('/deployment'))[0][1][-1], 'execution_mode:=impedance')

    def test_upper_inherits_actual_base_mode_after_startup(self):
        from types import SimpleNamespace as State
        from tools.deployment.deploy import base_mode
        states = {name:State(startup_complete=False,control_source='position') for name in ('servo','middle','upper')}
        self.assertIsNone(base_mode(states))
        for state in states.values():state.startup_complete=True
        self.assertEqual(base_mode(states), 'position')
        states['servo'].control_source='impedance'
        self.assertEqual(base_mode(states), 'impedance')
        states.pop('middle')
        self.assertIsNone(base_mode(states))

    def test_replay_sequence_and_executor_use_same_observed_mode(self):
        from tools.deployment.deploy import executor_command,replay_commands
        cfg,runs=Path('/config'),Path('/runs')
        for mode in ('position','impedance'):
            self.assertIn('execution_mode:='+mode,executor_command(cfg,runs,mode,'bindu_snowbot'))
            commands=replay_commands('replay',cfg,runs,mode,'run')
            self.assertEqual(len(commands),2)
            self.assertEqual(commands[0][commands[0].index('--action')+1],'Prepare')
            self.assertIn('--interactive',commands[1])
            for command in commands:
                self.assertEqual(command[command.index('--execution-mode')+1],mode)
            menu=replay_commands('menu',cfg,runs,mode,'run')
            self.assertEqual(len(menu),1)
            self.assertNotIn('Prepare',menu[0])
            prepare=replay_commands('prepare',cfg,runs,mode,'run')
            for command in [*commands,*menu,*prepare]:
                self.assertEqual(float(command[command.index('--speed')+1]),.5 if 'Prepare' in command else 1.)

    def test_vr_replay_share_upper_lock_and_release_it(self):
        from unittest.mock import patch
        from snowbot_bringup.runtime.run_vr import startup_lock
        with tempfile.TemporaryDirectory() as tmp,patch('tempfile.gettempdir',return_value=tmp):
            with startup_lock(7,'test_upper'):
                with self.assertRaisesRegex(RuntimeError,'ROS'):
                    startup_lock(7,'test_upper')
            with startup_lock(7,'test_upper'):
                pass

    def test_service_log_keeps_wkc_changes_and_faults_without_healthy_spam(self):
        from tools.deployment.deploy import keep_service_log
        previous = []
        healthy = '[ECAT-MON] pdo=100 WKC=67/67 bad_wkc=0 good_age_ms=0.2\n'
        self.assertTrue(keep_service_log(healthy, previous, 0.))
        self.assertFalse(keep_service_log(healthy.replace('100', '101'), previous, 1.))
        self.assertFalse(keep_service_log(healthy, previous, 5.))
        self.assertTrue(keep_service_log(healthy, previous, 10.))
        self.assertTrue(keep_service_log(healthy.replace('67/67', '45/67'), previous, 10.1))
        self.assertTrue(keep_service_log(healthy, previous, 10.2))
        self.assertTrue(keep_service_log(healthy.replace('bad_wkc=0', 'bad_wkc=1'), previous, 10.3))
        self.assertFalse(keep_service_log('TARGET_PREDICTION accepted=1 error=\n', previous, 11.))
        self.assertTrue(keep_service_log('TARGET_PREDICTION accepted=0 error=joint_limit\n', previous, 11.))
        self.assertTrue(keep_service_log('TARGET_PREDICTION accepted=1 error=admission_failed\n', previous, 11.))
        self.assertTrue(keep_service_log('[ERROR] drive fault\n', previous, 11.))

    def test_service_console_coalesces_fault_snapshots_and_shows_changed_faults(self):
        from unittest.mock import patch
        from tools.deployment.deploy import ServiceConsole
        console = ServiceConsole()
        warning = '[servo] [WARN] [1.0] [servo_node]: Position session: impedance_joint_feedback_invalid; pdo_sample=1 ft_valid=0x0\n'
        # Keep the fake clock local to deploy; lazy imports must see real time.
        from types import SimpleNamespace
        from unittest.mock import Mock
        with patch('tools.deployment.deploy.time',
                   SimpleNamespace(monotonic=Mock(side_effect=[0., 1., 10., 10.1]))):
            self.assertIn('ft_valid=0x0', console('servo', warning))
            self.assertIsNone(console('servo', warning.replace('pdo_sample=1', 'pdo_sample=2')))
            summary = console('servo', warning.replace('pdo_sample=1', 'pdo_sample=3'))
            self.assertIn('impedance_joint_feedback_invalid', summary)
            self.assertIn('repeat +2', summary)
            self.assertNotIn('pdo_sample', summary)
            self.assertIn('drive_fault', console('servo', warning.replace('impedance_joint_feedback_invalid', 'drive_fault')))

    def test_session_only_notice_prints_once_without_hiding_real_faults(self):
        from tools.deployment.deploy import ServiceConsole
        console = ServiceConsole()
        notice = '[servo] [WARN] [1.0] [servo_node]: Position session: legacy_source_disabled_for_target_stream\n'
        self.assertIn('INFO target_stream: legacy input ignored', console('servo', notice))
        for _ in range(5):
            self.assertIsNone(console('servo', notice))
        self.assertIn('ERROR drive fault', console('servo', '[servo] [ERROR] [1.1] [servo_node]: drive fault\n'))
        self.assertIsNone(console('servo', notice))
        self.assertIn('ERROR', console('servo', notice.replace('[WARN]', '[ERROR]')))

    def test_service_output_saves_all_lines_and_only_echoes_key_events(self):
        from io import StringIO
        from types import SimpleNamespace
        from unittest.mock import patch
        from tools.deployment.deploy import ServiceConsole, copy_service_output
        from snowbot_bringup.visualization.snowbot_diagnostics import DiagnosticWriter
        lines = ['  joint_name Kp Kd\n', '  shoulder_joint 1200 600\n',
                 '[INFO] [1.0] [servo_node]: pinocchio joint idx_q=1 idx_v=1\n',
                 '[EtherCAT] 7 slaves found\n',
                 '[ECAT-MON] pdo=1 WKC=13/13 bad_wkc=0 good_age_ms=0.2 L4{cw=0x0000}\n',
                 '[ECAT-MON] pdo=2 WKC=13/13 bad_wkc=0 good_age_ms=0.3 L4{cw=0x0000}\n',
                 'TARGET_PREDICTION accepted=1 error=\n',
                 '[WARN] [1.0] [servo_node]: impedance_joint_feedback_invalid\n',
                 '[WARN] [2.0] [servo_node]: impedance_joint_feedback_invalid\n',
                 '[ERROR] [2.0] [servo_node]: drive fault\n',
                 '警告：所有电机反馈为0，跳过回零控制。\n',
                 '收到中断信号，进入关机回零模式...\n',
                 'Traceback (most recent call last):\n', '  File "a.py", line 2\n',
                 'ValueError: invalid target\n']
        output = StringIO()
        with tempfile.TemporaryDirectory() as tmp, patch('sys.stdout', output):
            writer = DiagnosticWriter(Path(tmp)/'process-output.jsonl', echo=ServiceConsole())
            child = SimpleNamespace(stdout=StringIO(''.join(lines)))
            copy_service_output('servo', child, writer)
            writer.close()
            saved = (Path(tmp)/'servo.log').read_text()
            self.assertEqual(saved, ''.join('[servo] '+line for line in lines))
            self.assertTrue(child.stdout.closed)
            self.assertEqual(writer.dropped, 0)
        visible = output.getvalue()
        self.assertNotIn('Kp Kd', visible)
        self.assertNotIn('idx_q', visible)
        self.assertNotIn('TARGET_PREDICTION', visible)
        self.assertNotIn('L4{', visible)
        self.assertNotIn('LOG_REPEAT', visible)
        self.assertEqual(visible.count('WKC='), 1)
        self.assertEqual(visible.count('impedance_joint_feedback_invalid'), 1)
        for text in ('7 slaves found', 'drive fault', '警告', '关机回零', 'Traceback', 'ValueError'):
            self.assertIn(text, visible)

    def test_service_wait_uses_readiness_and_never_advances_after_exit(self):
        import threading
        from unittest.mock import Mock
        from tools.deployment.deploy import wait_service
        stopped = threading.Event(); state = []
        child = Mock(); child.poll.return_value = None
        spin = Mock(side_effect=lambda:state.append('ready'))
        self.assertTrue(wait_service([('servo', child)], lambda:bool(state), spin, stopped))
        spin.assert_called_once()
        child.poll.return_value = 3
        with self.assertRaisesRegex(RuntimeError, 'servo exited: code=3'):
            wait_service([('servo', child)], lambda:True, spin, stopped)
        stopped.set()
        self.assertFalse(wait_service([], lambda:False, spin, stopped))
        stopped.clear()
        with self.assertRaisesRegex(RuntimeError, 'STARTUP_TIMEOUT'):
            wait_service([], lambda:False, spin, stopped, timeout=0.)

    def test_base_exit_waits_for_servo_then_reverses_other_children_without_force_kill(self):
        import signal
        from unittest.mock import Mock, patch
        from tools.deployment import deploy
        children = []
        for index, name in enumerate(('servo', 'middle', 'upper', 'executor'), 1):
            child = Mock(pid=index); child.poll.return_value = None; child.wait.return_value = 0
            children.append((name, child))
        children[-1][1].wait.side_effect = [subprocess.TimeoutExpired('executor', 60.), 0]
        with patch.object(deploy.os, 'killpg') as send:
            self.assertTrue(deploy.stop_services(children))
        self.assertEqual(send.call_args_list, [unittest.mock.call(i, signal.SIGINT) for i in (1,4,3,2)])
        self.assertEqual(children[-1][1].wait.call_count, 2)
        children[0][1].wait.return_value = 2
        with patch.object(deploy.os, 'killpg'):
            self.assertFalse(deploy.stop_services(children[:1]))

    def test_lock_fetches_exact_revision_and_preserves_existing_checkouts(self):
        from unittest.mock import patch
        from tools.deployment import deploy
        def run(root,*args):
            return subprocess.check_output(['git','-C',str(root),*args],text=True).strip()
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);remote=root/'origin';remote.mkdir()
            run(remote,'init','-q');run(remote,'config','user.name','fixture');run(remote,'config','user.email','fixture@example.invalid')
            (remote/'src').mkdir();(remote/'src/value').write_text('locked')
            run(remote,'add','.');run(remote,'commit','-qm','locked');locked=run(remote,'rev-parse','HEAD')
            (remote/'src/value').write_text('newer');run(remote,'commit','-qam','newer');newer=run(remote,'rev-parse','HEAD')
            (root/'dependencies.repos').write_text(json.dumps({'repositories':{'bindu':dict(type='git',url=str(remote),version=locked)}}))
            with patch.object(deploy,'ROOT',root):
                source=deploy.locked_bindu(None,acquire=True)
                self.assertEqual(run(source,'rev-parse','HEAD'),locked)
                self.assertEqual((source/'src/value').read_text(),'locked')
                (source/'src/value').write_text('local edits')
                with self.assertRaisesRegex(ValueError,'local changes'):
                    deploy.locked_bindu(None,acquire=True)
                self.assertEqual((source/'src/value').read_text(),'local edits')
                with self.assertRaisesRegex(ValueError,'left unchanged'):
                    deploy.locked_bindu(remote,acquire=True)
                self.assertEqual(run(remote,'rev-parse','HEAD'),newer)
                (source/'src/value').write_text('locked')
                data=json.loads((root/'dependencies.repos').read_text());data['repositories']['bindu']['version']=newer
                (root/'dependencies.repos').write_text(json.dumps(data))
                with self.assertRaisesRegex(ValueError,'left unchanged'):
                    deploy.locked_bindu(None)  # Daily start must not update source.
                self.assertEqual(run(source,'rev-parse','HEAD'),locked)
                self.assertEqual(run(deploy.locked_bindu(None,acquire=True),'rev-parse','HEAD'),newer)

    def test_generated_vendor_reuses_cache_preserves_site_and_rejects_local_edits(self):
        from unittest.mock import patch
        from tools.deployment import deploy
        from bindu_contracts.resources import package_share
        bindu=package_share('bindu_execution').parents[2]
        with tempfile.TemporaryDirectory() as tmp:
            workspace=Path(tmp)
            site=workspace/'vendor-position.yaml';site.write_text('site-owned parameters')
            output=deploy.assemble(bindu,workspace)
            self.assertTrue((output/'src/servo_control/package.xml').is_file())
            self.assertEqual(deploy.assemble(bindu,workspace),output)
            self.assertEqual(site.read_text(),'site-owned parameters')
            digest=deploy.digest
            with patch.object(deploy,'digest',side_effect=lambda p:'changed-lock' if p==deploy.ROOT/'dependencies.repos' else digest(p)), \
                    patch.object(deploy.subprocess,'run') as generate:
                self.assertEqual(deploy.assemble(bindu,workspace),output)
                self.assertEqual(generate.call_count,0)
            native=bindu/'src/control/bindu_execution/native/interpolation.hpp'
            with patch.object(deploy,'digest',side_effect=lambda p:'changed-native' if p==native else digest(p)):
                self.assertEqual(deploy.assemble(bindu,workspace),output)
            self.assertEqual(len(list(workspace.glob('vendor-before-*'))),1)
            self.assertEqual(site.read_text(),'site-owned parameters')
            changed=output/'src/servo_control/src/main.cpp';changed.write_text('local maintenance')
            with self.assertRaisesRegex(ValueError,'Generated vendor changed'):
                deploy.assemble(bindu,workspace)
            self.assertEqual(changed.read_text(),'local maintenance')

    def test_impedance_backend_selection_is_preserved_on_rebuild(self):
        from tools.deployment import deploy
        from bindu_contracts.resources import package_share
        bindu=package_share('bindu_execution').parents[2]
        with tempfile.TemporaryDirectory() as tmp:
            workspace=Path(tmp)
            output=deploy.assemble(bindu,workspace)
            self.assertFalse(json.loads((output/'position_release.json').read_text())['impedance_available'])
            output=deploy.assemble(bindu,workspace,with_impedance=True)
            self.assertTrue(json.loads((output/'position_release.json').read_text())['impedance_available'])
            self.assertEqual(deploy.assemble(bindu,workspace),output)
            self.assertTrue(json.loads((output/'position_release.json').read_text())['impedance_available'])
            self.assertEqual(len(list(workspace.glob('vendor-before-*'))),1)

    def test_prepare_build_and_start_have_separate_side_effects(self):
        from unittest.mock import Mock, patch
        from tools.deployment import deploy
        with tempfile.TemporaryDirectory() as tmp:
            workspace=Path(tmp);(workspace/'upper').mkdir();(workspace/'upper/deployment.json').write_text('{}')
            calls=Mock();bindu=workspace/'bindu';vendor=workspace/'vendor'
            with patch.object(deploy,'require_platform'),patch.object(deploy,'require_stopped'), \
                 patch.object(deploy,'locked_bindu',return_value=bindu), \
                 patch.object(deploy,'assemble',side_effect=lambda *a,**kw:(calls.assemble(),vendor)[1]), \
                 patch.object(deploy,'vendor_build',side_effect=lambda *a:calls.vendor()), \
                 patch.object(deploy,'prepare_ros_dependencies',side_effect=lambda *a:calls.rosdep()), \
                 patch.object(deploy,'x86',side_effect=lambda b,w,c,*a:calls.upper(c)), \
                 patch.object(deploy,'receipt',side_effect=lambda *a:calls.receipt()):
                self.assertEqual(deploy.main(['prepare','--workspace',str(workspace),'--with-impedance']),0)
                self.assertTrue(deploy.assemble.call_args.kwargs['with_impedance'])
                self.assertEqual([c[0] for c in calls.mock_calls],['assemble','rosdep','vendor','upper','upper','receipt'])
                self.assertEqual([c.args for c in calls.mock_calls if c[0]=='upper'],[('prepare',),('build',)])
                calls.reset_mock();self.assertEqual(deploy.main(['build','--workspace',str(workspace)]),0)
                self.assertEqual([c[0] for c in calls.mock_calls],['assemble','vendor','upper','receipt'])
                calls.reset_mock();self.assertEqual(deploy.main(['start','--workspace',str(workspace),'--enable-commands']),0)
                self.assertEqual(calls.mock_calls,[unittest.mock.call.upper('start')])

if __name__=='__main__':unittest.main()

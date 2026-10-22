#!/usr/bin/env python3
"""Compile/run only native tests; no ROS processes, transport or hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

SHIM='''#pragma once
#include <string>
namespace rclcpp {
enum class QoSCompatibility {Ok,Warning,Error};
struct QoS {QoSCompatibility result=QoSCompatibility::Ok;};
struct Check {QoSCompatibility compatibility;std::string reason;};
inline Check qos_check_compatible(const QoS& offered,const QoS&) {return {offered.result,"test_policy"};}
}
'''

NODE_SHIM="""#pragma once
#include <atomic>
namespace rclcpp {
struct Node {
 std::atomic<unsigned> callbacks{0},state_updates{0};
 Node* get_node_base_interface(){return this;}
};
inline bool ok(){return true;}
inline void spin_some(Node* node){++node->callbacks;++node->state_updates;}
}
"""

def write_drive_cycle_check(source, work, root):
    """Actual command writer with in-memory PDOs; SOEM header deliberately empty."""
    driver=source/'servo_control'
    (work/'ethercat.h').write_text('')
    (work/'driver_test_access.h').write_text((driver/'include/servo_control/ethercat_driver.h').read_text().replace('private:','public:'))
    text=(driver/'src/servo_control/ethercat_driver.cpp').read_text()
    cycle=text[text.index('void EtherCATDriver::cycle() {'):text.index('    // --- PDO 收发 ---')]+'\n    (void)position_written;\n}\n'
    diag=text[text.index('std::string EtherCATDriver::driveDiagnostics() {'):text.index('void EtherCATDriver::cycle() {')]
    output=work/'test_drive_cycle.cpp'
    output.write_text((root/'test_drive_cycle.inc').read_text().replace('// @ACTUAL_CYCLE@',cycle).replace('// @ACTUAL_DIAGNOSTICS@',diag))
    return output

def write_pdo_check(source, work):
    """Compile actual final PDO writes/owned-CSP fault guard with memory PDOs."""
    text=(source/'servo_control/src/servo_control/ethercat_driver.cpp').read_text()
    block=text[text.index('    uint32_t position_written=0;'):text.index('    // --- PDO 收发 ---')]
    start=text.index('            if(session_packet.selected && (position_session::controlled_mask')
    guard=text[start:text.index('            if (!session_watchdog_.motor_enabled',start)]
    preamble=r'''
#include "servo_control/position_watchdog.h"
#include <cassert>
#include <array>
#include <iostream>
using namespace position_session;
struct PT {float target_position=99,target_speed=99,KP=99,KD=99,target_torque=99;};
struct CSP {int32_t position=99,velocity=99;int16_t torque=99;uint16_t controlword=99;int ModeOp=99;};
void check_position_pdo_output(){
 constexpr int EN_STEP=5,MODE_CSP=8;
 std::array<PT,20> pt;std::array<CSP,20> csp;
 std::array<PT*,20> m_rx_{};std::array<CSP*,20> csp_rx_{};std::array<bool,20> is_csp_motor_{};
 std::array<int,20> enable_step;enable_step.fill(EN_STEP*6);
 for(unsigned i=0;i<20;++i){if(i>=16){is_csp_motor_[i]=true;csp_rx_[i]=&csp[i];}else m_rx_[i]=&pt[i];}
 Output session_packet;session_packet.selected=true;session_packet.policy=2;session_packet.operation=1;
 session_packet.session=7;session_packet.sequence=1;session_packet.deadline=1200000000;
 session_packet.q[1]=.2;session_packet.q[8]=-.2;session_packet.q[14]=.12;session_packet.q[15]=-.08;session_packet.v[14]=.03;
 Observation obs;obs.sample=1;obs.sampled_ns=obs.attempted_ns=1000000000;
 obs.valid_mask=0xfffff;obs.wkc=obs.expected_wkc=67;obs.status.fill(0x27);
 Watchdog session_watchdog_;session_watchdog_.evaluate(session_packet,obs,1000000000);
 auto apply=[&](){
'''
    tests=r'''
 return position_written;
 };
 auto disable_csp=[&](){for(int m=0;m<20;++m){if(!csp_rx_[m])continue;
'''+guard+r'''
 }};
 // The exact final-write block must replace stale legacy caches during enable,
 // while withholding the output/hold acknowledgement until all axes are enabled.
 enable_step.fill(0);session_watchdog_.output.kp.fill(123);session_watchdog_.output.kd.fill(45);session_watchdog_.output.torque.fill(6);
 auto written=apply();assert(written==0);
 assert(pt[0].target_position==session_watchdog_.output.q[0]&&pt[0].KP==123&&pt[0].KD==45&&pt[0].target_torque==6);
 assert(csp[18].position==csp_counts(.12f)&&csp[19].position==csp_counts(-.08f));
 session_watchdog_.report_hold(obs,written);assert(!obs.hold_valid);
 enable_step.fill(EN_STEP*6);written=apply();assert(written==controlled_mask);
 assert(csp[18].position==csp_counts(.12f)&&csp[19].position==csp_counts(-.08f));
 assert(csp[18].velocity==csp_counts(.03f));
 assert(pt[7].target_position==99&&pt[15].target_position==99&&csp[16].position==99&&csp[17].position==99);
 session_packet.operation=2;++session_packet.sequence;session_watchdog_.evaluate(session_packet,obs,1000000000);
 written=apply();++obs.sample;session_watchdog_.report_hold(obs,written);
 assert(obs.hold_valid&&std::abs(obs.hold_position[14]-csp_radians(csp[18].position))<1e-7);
 csp_rx_[18]=nullptr;written=apply();session_watchdog_.report_hold(obs,written);assert(!obs.hold_valid);
 csp_rx_[18]=&csp[18];obs.status[18]|=8;session_watchdog_.evaluate(session_packet,obs,1000000000);assert(session_watchdog_.disabled);
 disable_csp();assert(csp[18].controlword==6&&csp[19].controlword==6&&csp[18].velocity==0&&csp[18].torque==0);
 assert(csp[16].controlword==99&&csp[17].controlword==99&&enable_step[18]==0&&enable_step[19]==0);
 session_watchdog_.report_hold(obs,controlled_mask);assert(!obs.hold_valid);
 std::cout<<"PASS actual PDO waist CSP/PT writes, quantized fixed hold, missing output/Fault and head/gripper isolation\n";
}
'''
    path=work/'position_pdo_check.cpp';path.write_text(preamble+block+tests);return path


def write_receipt_poll_check(source, work, root):
    """Reuse the existing poll fixture without its unrelated Eigen controller."""
    fixture=(root/'test_session_impedance.cpp').read_text()
    text=(source/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
    preamble=fixture[:fixture.index('constexpr double TEMPTIME')]
    preamble='\n'.join(line for line in preamble.splitlines() if
        'impedance/cartesian_impedance_controller.h' not in line and 'EIGEN_RUNTIME_NO_MALLOC' not in line)+'\n'
    body=fixture[fixture.index('constexpr double TEMPTIME'):fixture.index('// PRODUCTION_POLL_METHODS')]
    body='\n'.join(line for line in body.splitlines() if
        'snowbot_impedance::' not in line and 'constexpr double TEMPTIME' not in line)+'\n'
    body=body[:body.index(' void start(')]
    if 'void ServoNode::position_target_dispatch()' in text:
        body+=TARGET_FIELDS
    body+='};\n'
    start=text.index('void ServoNode::position_poll()')
    poll=text[start:text.index('void ServoNode::position_step()',start)]
    if 'void ServoNode::position_target_dispatch()' in text:
        begin=text.index('void ServoNode::position_target_dispatch()')
        poll=text[begin:start]+poll
    if 'void ServoNode::position_interrupt()' in text:
        begin=text.index('void ServoNode::position_interrupt()');end=text.index('void ServoNode::position_submit(',begin)
        poll=text[begin:end]+poll
    ready=''
    if 'bool ServoNode::session_impedance_ready() const' in text:
        start=text.index('bool ServoNode::session_impedance_ready() const')
        ready=text[start:text.index('\n}',start)+2]+'\n'
    described=fixture[fixture.index('TrajectoryWire described('):fixture.index('void check_adapter_memory')]
    checks=fixture[fixture.index('void check_production_poll()'):fixture.index('void check_trajectory_impedance')]
    if not ready:
        checks=checks.replace('invalid<5','invalid<1')
        checks=checks.replace('initialized/calibration/sensor admission gates','initialized admission gate (position-only candidate)')
        body=body.replace('snapshot.sensor_sampled_ns.fill(t);','')
        checks=checks.replace('if(sensor_age)snapshot.sensor_sampled_ns.fill(steady_ns()-sensor_age);','(void)sensor_age;')
        checks=checks.replace('snapshot.sensor_valid_mask=invalid==3?1:3;','').replace('snapshot.sensor_valid_mask=3;','')
    main='\nint main(){\n snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);snapshot.sensor_valid_mask=3;\n snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;\n check_production_poll();\n}\n'
    if not ready:main=main.replace('snapshot.sensor_valid_mask=3;','')
    path=work/'test_production_receipt.cpp'
    path.write_text(preamble+body+poll+ready+described+checks+main)
    return path

TARGET_FIELDS = r"""
 std::string position_feedback_gid_="publisher";
 position_session::Meta position_admission_source_{};position_session::FixedText position_target_error_{};
 Packet::SharedPtr position_target_pending_;
 uint64_t position_target_barrier_=0;bool position_target_pause_requested_=false;
 int64_t position_target_prepare_ns_=10000000;
 double trajectory_max_velocity_=.25,trajectory_max_acceleration_=.5,trajectory_max_jerk_=162.;
 position_session::ServoCommand position_target_takeover_{};bool position_target_takeover_valid_=false;
 void position_interrupt();void position_target_dispatch();void position_submit(const Packet::SharedPtr packet);
"""


def write_target_poll_check(source, work, root):
    fixture=(root/'test_session_impedance.cpp').read_text()
    text=(source/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
    preamble=fixture[:fixture.index('constexpr double TEMPTIME')]
    preamble='\n'.join(line for line in preamble.splitlines() if
        'impedance/cartesian_impedance_controller.h' not in line and 'EIGEN_RUNTIME_NO_MALLOC' not in line)+'\n'
    body=fixture[fixture.index('constexpr double TEMPTIME'):fixture.index('// PRODUCTION_POLL_METHODS')]
    body='\n'.join(line for line in body.splitlines() if
        'snowbot_impedance::' not in line and 'constexpr double TEMPTIME' not in line)+'\n'
    body=body[:body.index(' void start(')]+TARGET_FIELDS+'};\n'
    body=body.replace('snapshot.sensor_sampled_ns.fill(t);','')
    start=text.index('void ServoNode::position_interrupt()') if 'void ServoNode::position_interrupt()' in text else text.index('void ServoNode::position_submit(')
    methods=text[start:text.index('void ServoNode::position_step()',start)]
    methods=methods.replace('snowbot_ctl_msgs::msg::PositionSessionServo','Packet')
    checks=(root/'test_position_target_poll.inc').read_text()
    main='\nint main(){snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;check_target_production();check_bootstrap_interrupts();}\n'
    path=work/'test_production_target.cpp';path.write_text(preamble+body+methods+checks+main);return path

def write_position_cycle_check(source, work, root):
    fixture=(root/'test_session_impedance.cpp').read_text()
    text=(source/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
    preamble=fixture[:fixture.index('constexpr double TEMPTIME')]
    preamble='\n'.join(line for line in preamble.splitlines() if
        'impedance/cartesian_impedance_controller.h' not in line and 'EIGEN_RUNTIME_NO_MALLOC' not in line)+'\n'
    clock='int64_t cycle_ros_offset_ns=0;int64_t cycle_now_ns=1000000000;int64_t cycle_clock(){cycle_now_ns+=100;return cycle_now_ns;}\nposition_session::OutputCache* active_cycle_cache=nullptr;\n'
    body=fixture[fixture.index('constexpr double TEMPTIME'):fixture.index('// PRODUCTION_POLL_METHODS')]
    body='\n'.join(line for line in body.splitlines() if
        'snowbot_impedance::' not in line and 'constexpr double TEMPTIME' not in line)+'\n'
    body=body[:body.index(' void start(')]+TARGET_FIELDS+'};\n'
    body=body.replace('snapshot.sensor_sampled_ns.fill(t);','').replace('position_session::steady_ns()','cycle_clock()')
    body=body.replace('nanoseconds()const{return cycle_clock();}', 'nanoseconds()const{return cycle_clock()+cycle_ros_offset_ns;}')
    body=body.replace('position_feedback_.ready_ros=t;', 'position_feedback_.ready_ros=t+cycle_ros_offset_ns;')
    body=body.replace('position_feedback_.progress=position_feedback_.source=t;', 'position_feedback_.progress=t;position_feedback_.source=t+cycle_ros_offset_ns;')
    body=body.replace('void EtherCAT_positionOutput(const position_session::Output& p){applied=p;++writes;}',
        'void EtherCAT_positionOutput(const position_session::Output& p){active_cycle_cache->publish(p);applied=p;++writes;}')
    start=text.index('void ServoNode::position_interrupt()')
    methods=text[start:text.index('bool ServoNode::position_ready()',start)]
    methods=methods.replace('snowbot_ctl_msgs::msg::PositionSessionServo','Packet').replace('position_session::steady_ns()','cycle_clock()')
    if 'struct CycleFault' in (source/'servo_control/include/servo_control/position_observation.h').read_text():
        preamble='#define POSITION_CYCLE_DIAGNOSTICS 1\n'+preamble
    checks=(root/'test_position_cycle.inc').read_text()
    path=work/'test_production_position_cycle.cpp';path.write_text(preamble+clock+body+methods+checks);return path

def write_legacy_ingress_check(source, work, root):
    """Run the actual old-channel callback with memory-only command/enable sinks."""
    text=(source/'servo_control/src/ros_controller/rosLoop.cpp').read_text()
    start=text.index('void ServoNode::message_callback(')
    method=text[start:text.index('//函数2',start)]
    fixture=(root/'test_position_legacy_ingress.cpp').read_text()
    path=work/'test_production_legacy_ingress.cpp'
    path.write_text(fixture.replace('// PRODUCTION_LEGACY_CALLBACK',method))
    return path


def write_middle_check(source, work, root):
    text=(source/'dual_arm_control/src/controller/ArmInterface.cpp').read_text()
    start=text.index('std::vector<outpWrap> ArmCtlOptInterface::Traj_temp_outp(')
    opening=text.index('{',start);depth=1;end=opening+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    fixture=(root/'test_position_middle.cpp').read_text()
    path=work/'test_position_middle.cpp';path.write_text(fixture.replace('// PRODUCTION_FUNCTION',text[start:end]))
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True,help='Unified vendor src root')
    parser.add_argument("--output",type=Path)
    parser.add_argument("--baseline",type=Path,help="Old prepared src for touched middle default-path differential")
    parser.add_argument("--sanitize",action="store_true",help="AddressSanitizer and UndefinedBehaviorSanitizer, pure memory tests only")
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]/"native"
    compiler=shutil.which('c++')
    if not compiler:raise SystemExit('C++ compiler unavailable')
    sources=[args.source/'robot_control/include'/name for name in (
        'position_session_contract.h','position_connection_diagnostics.h','position_session_ros.h')]
    sources += [args.source/'servo_control/include/servo_control'/name for name in
                ('position_watchdog.h','position_observation.h','position_follower.h')]
    sources.append(args.source/'servo_control/src/servo_control/ethercat_driver.cpp')
    names=['test_snowbot_reconnect_tracker','test_snowbot_connection_contract','test_position_connection_diagnostics','test_position_initialization','test_position_session']
    if (args.source/'servo_control/include/position_receipt.h').is_file():
        sources.append(args.source/'servo_control/include/position_receipt.h')
        names.extend(('test_position_receipt','test_production_receipt'))
        sources.extend((args.source/'servo_control/src/ros_controller/rosLoop.cpp',root/'test_session_impedance.cpp'))
    native_include=args.source/'servo_control/include/bindu_execution_native'
    native_sources=[args.source/'servo_control/src/bindu_execution_native'/name
                    for name in ('interpolation.cpp','adaptive_stream.cpp')]
    trajectory=args.source/'servo_control/include/position_trajectory.h'
    if trajectory.is_file():
        # Compile the exact common implementation copied and hashed by the preparer.
        sources += [trajectory,*(native_include/name for name in
                    ('interpolation.hpp','adaptive_stream.hpp','curve_math.hpp')),*native_sources]
        missing=[str(path) for path in sources if not path.is_file()]
        if missing:raise RuntimeError('CANDIDATE_NATIVE_SOURCE_MISSING:'+','.join(missing))
        manifest=json.loads((args.source.parent/'position_release.json').read_text())
        native_paths=[*(native_include/name for name in
                      ('interpolation.hpp','adaptive_stream.hpp','curve_math.hpp')),*native_sources]
        expected=manifest.get('native_execution',{})
        if any(expected.get(path.name)!=hashlib.sha256(path.read_bytes()).hexdigest() for path in native_paths):
            raise RuntimeError('CANDIDATE_NATIVE_SHA256_MISMATCH')
        names.append('test_position_trajectory')
        if 'prepare_target' in trajectory.read_text():
            names.append('test_position_target')
            if 'void ServoNode::position_target_dispatch()' in (args.source/'servo_control/src/ros_controller/rosLoop.cpp').read_text():
                names.extend(('test_production_target','test_production_legacy_ingress'));sources.extend((root/'test_position_target_poll.inc',root/'test_position_legacy_ingress.cpp'))
                if 'void ServoNode::position_interrupt()' in (args.source/'servo_control/src/ros_controller/rosLoop.cpp').read_text():
                    names.append('test_production_position_cycle');sources.append(root/'test_position_cycle.inc')
        middle=args.source/'dual_arm_control/src/controller/ArmInterface.cpp'
        if 'reference_only' in middle.read_text():
            names.append('test_position_middle');sources.append(middle)
    if (args.source/'servo_control/include/servo_control/drive_maintenance.h').is_file():
        names.extend(('test_drive_reset','test_drive_cycle'))
        sources.extend((args.source/'servo_control/include/servo_control/drive_maintenance.h',args.source/'servo_control/include/servo_control/ethercat_driver.h',root/'test_drive_cycle.inc'))
    sources.append(Path(__file__).resolve())
    if args.baseline:sources.append(args.baseline/'dual_arm_control/src/controller/ArmInterface.cpp')
    sources.extend(root/(name+'.cpp') for name in names if (root/(name+'.cpp')).is_file())
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    results={}
    def failed(detail):
        if args.output:args.output.write_text(json.dumps(dict(passed=False,source=str(args.source),tests=results,failure=detail,source_hashes=before,hardware_accessed=False),indent=2)+'\n')
        raise RuntimeError(detail)
    with tempfile.TemporaryDirectory(prefix='connection-review-') as tmp:
        work=Path(tmp)
        shim=work/'rclcpp/qos.hpp';shim.parent.mkdir();shim.write_text(SHIM)
        (work/'rclcpp/rclcpp.hpp').write_text(NODE_SHIM)
        for name in names:
            binary=work/name
            if name=='test_drive_cycle':
                copied=write_drive_cycle_check(args.source,work,root)
            elif name=='test_production_legacy_ingress':
                copied=write_legacy_ingress_check(args.source,work,root)
            elif name=='test_position_middle':
                copied=write_middle_check(args.source,work,root)
            elif name=='test_production_position_cycle':
                copied=write_position_cycle_check(args.source,work,root)
            elif name=='test_production_target':
                copied=write_target_poll_check(args.source,work,root)
            elif name=='test_production_receipt':
                copied=write_receipt_poll_check(args.source,work,root)
            else:
                copied=work/(name+'.cpp');copied.write_bytes((root/(name+'.cpp')).read_bytes())
            command=[compiler,'-std=c++17','-Wall','-Wextra','-Werror','-pthread',
                '-I'+str(args.source/'robot_control/include'),'-I'+str(args.source/'servo_control/include'),'-I'+str(native_include),'-I'+str(work),'-I'+str(root),
                str(copied),'-o',str(binary)]
            if name=='test_drive_cycle':command.append('-Wno-unused-variable')
            if name=='test_position_middle':
                command[1:1]=['-DTARGET_AUXILIARY']
                command+=['-Wno-sign-compare','-Wno-comment']
            if args.sanitize:command[1:1]=['-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer']
            if name=='test_position_session':
                command.insert(1,'-DPOSITION_PDO_OUTPUT_TEST')
                command.append(str(write_pdo_check(args.source,work)))
            elif name in ('test_position_trajectory','test_production_receipt','test_position_target','test_production_target','test_production_position_cycle'):
                command.extend(map(str,native_sources))
            compiled=subprocess.run(command,capture_output=True,text=True)
            if compiled.returncode:failed(compiled.stdout+compiled.stderr)
            result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=30)
            if result.returncode:failed(result.stdout+result.stderr)
            if name=='test_position_middle':
                if args.baseline:
                    baseline=work/'middle-baseline';baseline.mkdir()
                    old=write_middle_check(args.baseline,baseline,root)
                    baseline_command=[flag for flag in command if flag!='-DTARGET_AUXILIARY']
                    baseline_command[baseline_command.index(str(copied))]=str(old)
                    baseline_command[baseline_command.index('-o')+1]=str(baseline/'check')
                    compiled_old=subprocess.run(baseline_command,capture_output=True,text=True)
                    if compiled_old.returncode:failed(compiled_old.stdout+compiled_old.stderr)
                    old_result=subprocess.run([str(baseline/'check')],capture_output=True,text=True,timeout=30)
                    if old_result.returncode or old_result.stdout!=result.stdout:failed('MIDDLE_DEFAULT_OUTPUT_DIFFERENTIAL:'+old_result.stderr)
                results[name]=dict(default_output_sha256=hashlib.sha256(result.stdout.encode()).hexdigest(),baseline_compared=bool(args.baseline),checks=result.stderr.strip())
                print(result.stderr.strip())
            else:
                results[name]=result.stdout.strip()
                print(result.stdout.strip())
    after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    if before!=after:raise RuntimeError('SOURCE_CHANGED_DURING_NATIVE_TESTS')
    report=dict(passed=True,source=str(args.source),source_hashes=after,tests=results,
        real_ros_transport=False,real_controller_input=False,hardware_accessed=False,sanitized=args.sanitize)
    if args.output:args.output.write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()

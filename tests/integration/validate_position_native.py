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
 auto written=apply();assert(written==controlled_mask);
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

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True,help='Unified vendor src root')
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]/"native"
    compiler=shutil.which('c++')
    if not compiler:raise SystemExit('C++ compiler unavailable')
    sources=[args.source/'robot_control/include'/name for name in (
        'position_session_contract.h','position_connection_diagnostics.h','position_session_ros.h')]
    names=['test_snowbot_reconnect_tracker','test_snowbot_connection_contract','test_position_connection_diagnostics','test_position_initialization','test_position_session']
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
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    results={}
    with tempfile.TemporaryDirectory(prefix='connection-review-') as tmp:
        work=Path(tmp)
        shim=work/'rclcpp/qos.hpp';shim.parent.mkdir();shim.write_text(SHIM)
        (work/'rclcpp/rclcpp.hpp').write_text(NODE_SHIM)
        for name in names:
            binary=work/name
            copied=work/(name+'.cpp');copied.write_bytes((root/(name+'.cpp')).read_bytes())
            command=[compiler,'-std=c++17','-Wall','-Wextra','-Werror','-pthread',
                '-I'+str(args.source/'robot_control/include'),'-I'+str(args.source/'servo_control/include'),'-I'+str(native_include),'-I'+str(work),'-I'+str(root),
                str(copied),'-o',str(binary)]
            if name=='test_position_session':
                command.insert(1,'-DPOSITION_PDO_OUTPUT_TEST')
                command.append(str(write_pdo_check(args.source,work)))
            elif name=='test_position_trajectory':
                command.extend(map(str,native_sources))
            subprocess.run(command,check=True,capture_output=True,text=True)
            result=subprocess.run([str(binary)],check=True,capture_output=True,text=True,timeout=30)
            results[name]=result.stdout.strip()
            print(result.stdout.strip())
    after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    if before!=after:raise RuntimeError('SOURCE_CHANGED_DURING_NATIVE_TESTS')
    report=dict(passed=True,source=str(args.source),source_hashes=after,tests=results,
        real_ros_transport=False,real_controller_input=False,hardware_accessed=False)
    if args.output:args.output.write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()

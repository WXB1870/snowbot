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
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    results={}
    with tempfile.TemporaryDirectory(prefix='connection-review-') as tmp:
        work=Path(tmp)
        shim=work/'rclcpp/qos.hpp';shim.parent.mkdir();shim.write_text(SHIM)
        (work/'rclcpp/rclcpp.hpp').write_text(NODE_SHIM)
        for name in ('test_snowbot_reconnect_tracker','test_snowbot_connection_contract','test_position_connection_diagnostics','test_position_initialization','test_position_session'):
            binary=work/name
            copied=work/(name+'.cpp');copied.write_bytes((root/(name+'.cpp')).read_bytes())
            command=[compiler,'-std=c++17','-Wall','-Wextra','-Werror','-pthread',
                '-I'+str(args.source/'robot_control/include'),'-I'+str(args.source/'servo_control/include'),'-I'+str(work),'-I'+str(root),
                str(copied),'-o',str(binary)]
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

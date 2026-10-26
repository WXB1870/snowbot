from pathlib import Path
import re,json,hashlib
import argparse
p=argparse.ArgumentParser(description='Check original messages, caches, bus calls, startup/exit and impedance isolation')
p.add_argument('--baseline',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path)
args=p.parse_args();a=args.baseline;b=args.source
def section(text,start,end):return text.split(start,1)[1].split(end,1)[0]
checks={}
for msg in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
 rel='ctl_msgs/msg/'+msg+'.msg';checks['legacy_'+msg]=(a/rel).read_bytes()==(b/rel).read_bytes()
rel='servo_control/src/ros_controller/rosLoop.cpp';old=(a/rel).read_text();new=(b/rel).read_text()
checks['servo_startup']=section(old,'void ServoNode::run(){','    //开启ros2循环') in new
# Ignore the added completion flag immediately AFTER the original startup/exit declarations.
checks['servo_startup']=section(old,'void ServoNode::run(){','    //开启ros2循环')==section(new,'void ServoNode::run(){','    position_initialized_=true;position_init_error_.clear();')
checks['shutdown_branch']=section(old,'        //----- Ctrl+C 关机回零:', '        //第一轮控制')==section(new,'        //----- Ctrl+C 关机回零:', '        //第一轮控制')
checks['shutdown_trajectory']=section(old,'        //----- 关机回零: 覆盖上位机目标', '        //-----轨迹计算-----')==section(new,'        //----- 关机回零: 覆盖上位机目标', '        //-----轨迹计算-----')
checks['shutdown_tail']=section(old,'    // 关机收尾:', '//目标值包装协议')==section(new,'    // 关机收尾:', '//目标值包装协议')
checks['impedance_body']=section(old,'void ServoNode::applyImpedanceControl()', 'void ServoNode::rosLoopVar_Init()')==section(new,'void ServoNode::applyImpedanceControl()', 'void ServoNode::rosLoopVar_Init()')
# Every impedance source/header and config retains its original bytes.
checks['impedance_sources']=all(p.read_bytes()==(b/p.relative_to(a)).read_bytes() for p in a.rglob('*') if p.is_file() and ('impedance' in str(p.relative_to(a)).lower()))
rel='servo_control/src/servo_control/ethercat_driver.cpp';old=(a/rel).read_text();new=(b/rel).read_text()
# Only the reviewed position resolution changes inside the existing feedback cache.
expected_cache=section(old,'    // 只在实时线程读取 PDO', '    ++completed_pdo_cycles_;').replace('csp_tx_[m]->position * (2.0 * M_PI / 65536.0)', 'csp_tx_[m]->position * (2.0 * M_PI / 262144.0)')
checks['internal_feedback_and_force_caches']=expected_cache==section(new,'    // 只在实时线程读取 PDO','    auto& obs=observation_rt_;')
checks['bus_calls_unchanged']=all(old.count(call)==new.count(call) for call in ('ec_send_processdata(', 'ec_receive_processdata(', 'ec_SDOread(', 'ec_SDOwrite(', 'ec_readstate('))
rel='robot_control/src/robot_control/msgPublisher.cpp';old=(a/rel).read_text();new=(b/rel).read_text();checks['upper_startup']=section(old,'            double threshold_ang = 0.2;','        //-------------------')==section(new,'            double threshold_ang = 0.2;','        //-------------------')
print(json.dumps(checks,indent=2));assert all(checks.values())
if args.output:args.output.write_text(json.dumps(checks,indent=2)+'\n')

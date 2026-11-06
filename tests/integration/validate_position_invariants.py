from pathlib import Path
import json
import argparse
p=argparse.ArgumentParser(description='Check preserved vendor messages, feedback caches and bus calls')
p.add_argument('--baseline',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path)
args=p.parse_args();a=args.baseline;b=args.source
def section(text,start,end):return text.split(start,1)[1].split(end,1)[0]
checks={}
for msg in ('RetInformation','JointPositionCommand','MidNodeCtlMsgSend','CtlInformation'):
 rel='ctl_msgs/msg/'+msg+'.msg';checks['legacy_'+msg]=(a/rel).read_bytes()==(b/rel).read_bytes()
# Lifecycle behavior is exercised by the native validators; impedance inputs
# are locked by the preparation manifest and its existing provenance tests.
rel='servo_control/src/servo_control/ethercat_driver.cpp';old=(a/rel).read_text();new=(b/rel).read_text()
# Only the reviewed position resolution changes inside the existing feedback cache.
expected_cache=section(old,'    // 只在实时线程读取 PDO', '    ++completed_pdo_cycles_;').replace('csp_tx_[m]->position * (2.0 * M_PI / 65536.0)', 'csp_tx_[m]->position * (2.0 * M_PI / 262144.0)')
if (b/'servo_control/include/position_waist_feedback.h').is_file():
 expected_cache=expected_cache.replace('static_cast<float>(csp_tx_[m]->velocity * (2.0 * M_PI / 65536.0))', 'static_cast<float>(waist_feedback_units_.speed(m,csp_tx_[m]->velocity * (2.0 * M_PI / 65536.0)))')
checks['internal_feedback_and_force_caches']=expected_cache==section(new,'    // 只在实时线程读取 PDO','    auto& obs=observation_rt_;')
feedback_reads=int((b/'servo_control/include/position_waist_feedback.h').is_file())
checks['bus_calls_unchanged']=all(old.count(call)+(feedback_reads if call=='ec_SDOread(' else 0)==new.count(call) for call in ('ec_send_processdata(', 'ec_receive_processdata(', 'ec_SDOread(', 'ec_SDOwrite(', 'ec_readstate('))
if feedback_reads:
 checks['ratio_read_before_pdo_config']=new.index('ec_SDOread(slave,0x200E,2,')<new.index('// === 2. 电机 PDO 配置 (PREOP) ===')
rel='robot_control/src/robot_control/msgPublisher.cpp';old=(a/rel).read_text();new=(b/rel).read_text();checks['upper_startup']=section(old,'            double threshold_ang = 0.2;','        //-------------------')==section(new,'            double threshold_ang = 0.2;','        //-------------------')
print(json.dumps(checks,indent=2));assert all(checks.values())
if args.output:args.output.write_text(json.dumps(checks,indent=2)+'\n')

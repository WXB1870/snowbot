
#include "impedance/cartesian_impedance_controller.h"
#include "servo_control/position_watchdog.h"
#include "servo_control/position_follower.h"
#include "servo_control/position_clock.h"
#include <cassert>
#include <iostream>
constexpr double TEMPTIME=.001;
position_session::Observation snapshot;
position_session::Output applied;
unsigned writes=0;std::array<float,20> gravity{};
position_session::Observation EtherCAT_positionObservation(){return snapshot;}
void EtherCAT_positionOutput(const position_session::Output& p){applied=p;++writes;}
void EtherCAT_setGravityFF(unsigned i,float x){gravity[i]=x;}
struct ServoNode {
 std::string session_execution_mode_="position",position_rejection_;
 bool impedance_enable_=false,session_calibration_confirmed_=false,impedance_inited_=true,session_impedance_active_=false;
 uint64_t session_impedance_session_=0;
 std::shared_ptr<snowbot_impedance::CartesianImpedanceController> impedance_left_,impedance_right_;
 std::array<int,20> session_q_index_{},session_v_index_{};
 position_session::Gate position_gate_;position_session::Tracker position_feedback_;
 position_session::Interval position_interval_;position_session::Follower position_follower_;
 position_session::Output position_output_;
 struct Time {int64_t nanoseconds()const{return position_session::steady_ns();}};Time now()const{return {};}
 bool session_impedance_ready()const;bool session_impedance_step();void position_step();
 void feedback(){auto t=position_session::steady_ns();snapshot.sampled_ns=snapshot.attempted_ns=t;++snapshot.sample;
  position_feedback_.advances=2;position_feedback_.expected=position_feedback_.wkc=67;position_feedback_.valid=0xfffff;
  position_feedback_.age=0;position_feedback_.progress=position_feedback_.source=t;
  for(unsigned i=0;i<16;++i)position_feedback_.q[i]=snapshot.position[position_session::position_slot(i)];
 }
 void start(uint8_t operation=1,uint64_t sequence=1){
  feedback();auto t=position_session::steady_ns();position_session::Meta m{99,sequence,t,t+200000000,operation};position_gate_.commit(m,t,t);
  position_follower_.seed(position_feedback_.q);position_interval_.reset(t);
  position_output_.selected=true;position_output_.session=99;position_output_.sequence=sequence;position_output_.operation=operation;position_output_.policy=2;position_output_.deadline=position_gate_.deadline();position_output_.failure_reason=0;
  position_output_.kp.fill(40);position_output_.kd.fill(2);position_output_.torque.fill(0);
  position_output_.hold_kp=position_output_.kp;position_output_.hold_kd=position_output_.kd;
 }
};

// PRODUCTION_METHODS

int main(int argc,char**argv){assert(argc==2);ServoNode node;
 snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);snapshot.sensor_valid_mask=3;
 snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;
 node.position_step();assert(writes==0);node.start();node.position_step();assert(writes==1&&!applied.impedance&&applied.kp[0]==40);
 auto ros=std::make_shared<rclcpp::Node>();
 for(const std::string side:{"left","right"}){auto cfg=snowbot_impedance::ImpedanceConfig::makeDefault(side,argv[1]);cfg.session_reference_only=true;cfg.filter_params_pose=cfg.filter_params_nullspace=1.;cfg.tcp_to_ee={0,0,.192,0,0,0,1};auto ctl=std::make_shared<snowbot_impedance::CartesianImpedanceController>();assert(ctl->init(ros,cfg));if(side=="left")node.impedance_left_=ctl;else node.impedance_right_=ctl;}
 for(unsigned i=0;i<20;++i){const std::string name=i<8?"arm_l_joint_"+std::to_string(i+1):i<16?"arm_r_joint_"+std::to_string(i-7):i<18?"head_joint_"+std::to_string(i-15):"body_joint_"+std::to_string(i-17);node.session_q_index_[i]=node.impedance_left_->getJointQIdx(name);node.session_v_index_[i]=node.impedance_left_->getJointVIdx(name);}
 node.session_execution_mode_="impedance";node.feedback();assert(!node.session_impedance_ready());
 node.session_calibration_confirmed_=true;node.start();assert(node.session_impedance_ready());node.position_step();assert(applied.impedance&&applied.kp[0]==1&&applied.kd[0]==.2f&&std::isfinite(applied.torque[0]));
 assert(applied.q[14]==snapshot.position[18]&&applied.q[15]==snapshot.position[19]);
 assert(gravity[7]==0&&gravity[15]==0&&gravity[16]==0&&gravity[17]==0);
 // Actual production watchdog, so ROS repetition cannot keep old torque alive.
 auto live=applied;position_session::Watchdog watchdog;node.feedback();watchdog.evaluate(live,snapshot,position_session::steady_ns());assert(!watchdog.disabled);
 snapshot.sampled_ns=live.deadline;watchdog.evaluate(live,snapshot,live.deadline);assert(watchdog.reason==position_session::EXPIRED&&!watchdog.disabled&&!watchdog.output.impedance);assert(watchdog.output.kp[0]==40&&watchdog.output.torque==live.hold_torque);
 live.policy=1;watchdog.evaluate(live,snapshot,live.deadline);assert(watchdog.disabled);
 node.start(2,2);node.position_step();assert(!applied.impedance&&applied.kp[0]==40);assert(!node.session_impedance_active_);
 node.start(1,3);snapshot.sensor_valid_mask=1;node.position_step();assert(applied.failure_reason==position_session::IMPEDANCE_INVALID&&!node.position_gate_.live(position_session::steady_ns()));
 watchdog.evaluate(live,snapshot,position_session::steady_ns());assert(watchdog.disabled&&watchdog.reason==position_session::IMPEDANCE_INVALID);
 snapshot.sensor_valid_mask=3;snapshot.wrench[0]=NAN;node.feedback();assert(!node.session_impedance_ready());
 snapshot.wrench[0]=0;node.feedback();snapshot.sampled_ns-=21000000;assert(!node.session_impedance_ready());
 std::cout<<"PASS actual servo default position, gated impedance, waist output, sensor/calibration rejection, fixed-position PAUSE, RT expiry torque discard and resource isolation\n";
}

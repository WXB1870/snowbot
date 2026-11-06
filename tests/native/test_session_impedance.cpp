#include "position_receipt.h"
#define EIGEN_RUNTIME_NO_MALLOC

#include "impedance/cartesian_impedance_controller.h"
#include "servo_control/position_watchdog.h"
#include "servo_control/position_follower.h"
#include "servo_control/position_clock.h"
#include "position_trajectory.h"
#include <cassert>
#include <iostream>
#include <thread>
#include <tuple>
#include <cstdlib>
#include <new>
thread_local bool count_periodic_memory=false;
thread_local size_t periodic_allocations=0,periodic_deletions=0;
void* operator new(std::size_t n){if(count_periodic_memory)++periodic_allocations;if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{if(count_periodic_memory&&p)++periodic_deletions;std::free(p);}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void* p)noexcept{::operator delete(p);}
void operator delete(void* p,std::size_t)noexcept{::operator delete(p);}
void operator delete[](void* p,std::size_t)noexcept{::operator delete(p);}


constexpr double TEMPTIME=.001;
position_session::Observation snapshot;
position_session::Output applied;
unsigned writes=0,gravity_writes=0;std::array<float,20> gravity{};
position_session::Observation EtherCAT_positionObservation(){return snapshot;}
void EtherCAT_positionOutput(const position_session::Output& p){applied=p;++writes;}
void EtherCAT_setGravityFF(unsigned i,float x){gravity[i]=x;++gravity_writes;}
struct TrajectoryWire {
 unsigned version=2;uint64_t segment_id=0;int64_t effective_at_ns=0;
 std::vector<double> piece_start,duration,source_duration,source_elapsed,controls;
 std::vector<unsigned char> control_count;std::array<double,16> max_velocity{},max_acceleration{},max_jerk{};
 bool operator!=(const TrajectoryWire& b)const{return std::tie(version,segment_id,effective_at_ns,piece_start,duration,source_duration,source_elapsed,controls,control_count,max_velocity,max_acceleration,max_jerk)!=std::tie(b.version,b.segment_id,b.effective_at_ns,b.piece_start,b.duration,b.source_duration,b.source_elapsed,b.controls,b.control_count,b.max_velocity,b.max_acceleration,b.max_jerk);}
};

struct Packet {
 using SharedPtr=std::shared_ptr<Packet>;
 struct Metadata {
  uint64_t session=0,sequence=0,feedback_boot_id=0,feedback_sample=0,middle_instance_id=0,epoch=4;
  int64_t source_stamp_ns=0,expires_at_ns=0;uint8_t operation=1,reference_mode=2;
  std::string source_command_id="source-command-long-enough-for-heap",lease_id="lease-long-enough-for-heap",profile_hash="profile-long-enough-for-heap";
  bool operator!=(const Metadata& b)const{return std::tie(session,sequence,feedback_boot_id,feedback_sample,middle_instance_id,epoch,source_stamp_ns,expires_at_ns,operation,reference_mode,source_command_id,lease_id,profile_hash)!=std::tie(b.session,b.sequence,b.feedback_boot_id,b.feedback_sample,b.middle_instance_id,b.epoch,b.source_stamp_ns,b.expires_at_ns,b.operation,b.reference_mode,b.source_command_id,b.lease_id,b.profile_hash);}
 }meta;
 struct {bool if_use_debugmod=false,if_topic_point_position=true;std::array<double,7>l_position{},r_position{},l_kp{},l_kd{},r_kp{},r_kd{},l_torque_grav_comp{},r_torque_grav_comp{};}command;
 uint64_t frame_sequence=1;std::array<double,2>waist_position{};TrajectoryWire trajectory;
};
std::atomic<bool>sigInterrupt{false},inspect_hold{false},inspect_entered{false};
std::array<bool,20> enabled{};unsigned enable_requests=0;
void EtherCAT_setSingleMotorEnable(unsigned slot,bool value){enabled[slot]=value;++enable_requests;}

struct ServoNode {
 std::string session_execution_mode_="position",position_rejection_;
 double impedance_following_error_rad_=.25;
 bool impedance_enable_=false,impedance_inited_=true,session_impedance_active_=false;
 uint64_t session_impedance_session_=0;
 std::shared_ptr<snowbot_impedance::CartesianImpedanceController> impedance_left_,impedance_right_;
 std::array<int,20> session_q_index_{},session_v_index_{};
 position_session::Gate position_gate_;position_session::Tracker position_feedback_;
 position_session::Interval position_interval_;
 position_session::Output position_output_;
 position_session::ServoAdmissionState position_acceptance_;
 uint64_t position_instance_id_=47,position_rejected_session_=0,position_rejected_sequence_=0;
 bool position_initialized_=true,position_startup_complete_=true,shutdown_mode_=false,position_local_hold_=false;
 const char*position_cycle_reason_=nullptr;uint8_t position_policy_=2;
 Packet::SharedPtr position_origin_packet_;
 std::unique_ptr<position_session::TrajectoryAdmission<Packet>> position_admission_;
 bool position_receipts_enabled_=true;position_session::ReceiptQueue position_receipts_;
 snowbot_impedance::VectorXd session_q_,session_dq_,session_efforts_,session_nominal_,session_velocity_;
 uint8_t position_reference_mode_=2;position_session::TrajectoryQueue position_trajectory_;
 struct Time {int64_t nanoseconds()const{return position_session::steady_ns();}};Time now()const{return {};}
 bool session_impedance_ready()const;bool session_impedance_step(double dt);void position_step();void position_poll();
 void session_impedance_workspace();bool session_impedance_inputs(const position_session::Observation&);
 void feedback(){auto t=position_session::steady_ns();snapshot.sampled_ns=snapshot.attempted_ns=t;snapshot.sensor_sampled_ns.fill(t);++snapshot.sample;
  position_feedback_.boot=9;position_feedback_.sample=snapshot.sample;if(!position_feedback_.ready_sample){position_feedback_.ready_sample=snapshot.sample;position_feedback_.ready_ros=t;}
  position_feedback_.advances=2;position_feedback_.expected=position_feedback_.wkc=67;position_feedback_.valid=0xfffff;
  position_feedback_.age=0;position_feedback_.progress=position_feedback_.source=t;
  for(unsigned i=0;i<16;++i)position_feedback_.q[i]=snapshot.position[position_session::position_slot(i)];
 }

};

// PRODUCTION_POLL_METHODS

// PRODUCTION_METHODS



void check_adapter_memory(ServoNode& node){
 auto original=snapshot;for(unsigned i=0;i<20;++i){snapshot.position[i]=float(i)*.001f;snapshot.speed[i]=float(i)*.002f;}
 node.position_trajectory_.q.fill(.1);node.position_trajectory_.v.fill(.02);
 auto*nominal=node.session_nominal_.data();auto*velocity=node.session_velocity_.data();
 periodic_allocations=periodic_deletions=0;count_periodic_memory=true;Eigen::internal::set_is_malloc_allowed(false);
 for(unsigned repeat=0;repeat<1000;++repeat)assert(node.session_impedance_inputs(snapshot));
 Eigen::internal::set_is_malloc_allowed(true);count_periodic_memory=false;
 assert(!periodic_allocations&&!periodic_deletions&&nominal==node.session_nominal_.data()&&velocity==node.session_velocity_.data());
 for(unsigned i=0;i<20;++i){assert(node.session_q_[node.session_q_index_[i]]==snapshot.position[i]);assert(node.session_dq_[node.session_v_index_[i]]==snapshot.speed[i]);}
 for(unsigned i=0;i<16;++i){auto slot=position_session::position_slot(i);assert(node.session_nominal_[node.session_q_index_[slot]]==.1&&node.session_velocity_[node.session_v_index_[slot]]==.02);}
 snapshot=original;
 std::cout<<"PASS actual production 20-joint/waist impedance input mapping: 1000 calls, zero new/delete and Eigen malloc disabled; controller internals excluded\n";
}



void check_target_impedance(ServoNode& previous){
 using namespace position_session;
 ServoNode node;node.impedance_left_=previous.impedance_left_;node.impedance_right_=previous.impedance_right_;
 node.session_q_index_=previous.session_q_index_;node.session_v_index_=previous.session_v_index_;node.session_impedance_workspace();
 node.session_execution_mode_="impedance";node.position_reference_mode_=2;
 snapshot.wrench.fill(0);snapshot.sensor_valid_mask=3;snapshot.status.fill(0x27);
 node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_,[](auto& result){
  if(auto error=inspect_servo_command(*result.packet,result.origin.get(),result.context,result.command))throw std::invalid_argument(error);
 });
 auto packet=[&](uint64_t sequence,uint8_t operation){
  node.feedback();auto p=std::make_shared<Packet>();const auto now=steady_ns();
  p->meta.session=701;p->meta.sequence=sequence;p->meta.operation=operation;p->meta.reference_mode=2;
  p->meta.source_stamp_ns=now;p->meta.expires_at_ns=now+(operation==2?350000000:200000000);
  p->meta.feedback_boot_id=node.position_feedback_.boot;p->meta.feedback_sample=node.position_feedback_.sample;p->meta.middle_instance_id=32;
  for(unsigned i=0;i<7;++i){p->command.l_position[i]=node.position_feedback_.q[i];p->command.r_position[i]=node.position_feedback_.q[i+7];p->command.l_kp[i]=p->command.r_kp[i]=40;p->command.l_kd[i]=p->command.r_kd[i]=2;}
  p->waist_position={node.position_feedback_.q[14],node.position_feedback_.q[15]};
  p->trajectory.version=2;p->trajectory.segment_id=sequence;p->trajectory.max_velocity.fill(.25);p->trajectory.max_acceleration.fill(.5);p->trajectory.max_jerk.fill(162);return p;
 };
 auto tick=[&](){node.feedback();node.position_poll();node.position_interval_.reset(steady_ns()-1000000);node.position_step();};
 auto settle=[&](){const auto until=steady_ns()+1000000000;do{tick();if(steady_ns()>=until){std::cerr<<"target admission stalled: "<<(node.position_cycle_reason_?node.position_cycle_reason_:"none")<<" adopted sequence="<<node.position_gate_.last().sequence<<"\n";assert(false);}std::this_thread::sleep_for(std::chrono::milliseconds(1));}while(node.position_admission_->busy()||node.position_target_pending_);};
 node.position_submit(packet(1,1));settle();assert(node.position_gate_.selected()&&applied.impedance&&applied.reference_mode==2);
 auto move=packet(2,0);
#ifdef SNOWBOT_RIGID_PREDICTION
 // A shared-waist target preserves the model's relative grasp exactly.
 move->trajectory.version=3;move->waist_position[0]+=.005;
#else
 move->command.l_position[0]+=.005;
#endif
 node.position_submit(move);settle();
 if(node.position_gate_.last().sequence!=2)std::cerr<<"ordinary target rejected: "<<(node.position_cycle_reason_?node.position_cycle_reason_:"none")<<"\n";
 assert(node.position_gate_.last().sequence==2);
 for(unsigned i=0;i<20;++i){tick();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 auto pause=packet(3,2);node.position_submit(pause);settle();
 const auto until=steady_ns()+300000000;while(applied.trajectory_phase!=4){tick();if(steady_ns()>=until){std::cerr<<"target pause stalled: "<<(node.position_cycle_reason_?node.position_cycle_reason_:"none")<<" phase="<<unsigned(applied.trajectory_phase)<<" failure="<<unsigned(applied.failure_reason)<<" sequence="<<applied.sequence<<"\n";assert(false);}std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 assert(applied.impedance&&applied.nominal_hold);const auto fixed=applied.reference_position;const auto deadline=applied.deadline;
 const auto source_stamp=applied.source_stamp_ns,expires=applied.expires_at_ns;const auto old_torque=applied.torque;
 // Real controller sees an independent measured displacement; reference stays fixed.
 snapshot.position[0]+=.03f;Watchdog guard;
 do{
  tick();const auto now=steady_ns();guard.evaluate(applied,snapshot,now);snapshot.attempted_ns=now;guard.report_hold(snapshot,controlled_mask);
  assert(applied.impedance&&applied.failure_reason==0&&applied.reference_position==fixed&&applied.deadline==deadline);
  assert(applied.source_stamp_ns==source_stamp&&applied.expires_at_ns==expires&&guard.reason==PAUSED&&!guard.disabled);
  assert(snapshot.hold_valid&&snapshot.hold_basis==0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }while(steady_ns()<deadline+20000000);
 assert(applied.torque!=old_torque); // Feedback changes torque, not nominal position.
 auto invalid=guard;auto feedback=snapshot;feedback.sensor_valid_mask=1;invalid.evaluate(applied,feedback,steady_ns());
 assert(invalid.disabled&&invalid.reason==IMPEDANCE_INVALID);
 // Both waist CSP axes permit lag beyond the arm following threshold.
 for(unsigned axis:{14u,15u}){
  snapshot.position[position_slot(axis)]=float(fixed[axis]-.6);tick();
  assert(applied.failure_reason==0);guard.evaluate(applied,snapshot,steady_ns());assert(!guard.disabled);
  snapshot.position[position_slot(axis)]=float(fixed[axis]);tick();
 }
 snapshot.position[0]=float(fixed[0]+node.impedance_following_error_rad_+.01);tick();
 assert(applied.failure_reason==CYCLE_OR_TRACKING_ERROR);guard.evaluate(applied,snapshot,steady_ns());assert(guard.disabled);
 node.feedback();tick();assert(applied.failure_reason==CYCLE_OR_TRACKING_ERROR);
 std::cout<<"PASS actual session_v3 impedance target/pause: fixed nominal despite measured displacement, live torque, original TTL, post-expiry local hold, sensor and hard-following failure\n";
}

int main(int argc,char**argv){assert(argc==2);ServoNode node;
 snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);snapshot.sensor_valid_mask=3;
 snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;
 node.position_step();assert(writes==0);
 auto ros=std::make_shared<rclcpp::Node>();
 for(const std::string side:{"left","right"}){auto cfg=snowbot_impedance::ImpedanceConfig::makeDefault(side,argv[1]);cfg.session_reference_only=true;cfg.mode=snowbot_impedance::ImpedanceMode::JOINT;cfg.joint_stiffness.assign(7,20.);cfg.joint_damping.assign(7,2.);cfg.filter_params_pose=cfg.filter_params_nullspace=1.;cfg.tcp_to_ee={0,0,.192,0,0,0,1};auto ctl=std::make_shared<snowbot_impedance::CartesianImpedanceController>();assert(ctl->init(ros,cfg));if(side=="left")node.impedance_left_=ctl;else node.impedance_right_=ctl;}
 for(unsigned i=0;i<20;++i){const std::string name=i<8?"arm_l_joint_"+std::to_string(i+1):i<16?"arm_r_joint_"+std::to_string(i-7):i<18?"head_joint_"+std::to_string(i-15):"body_joint_"+std::to_string(i-17);node.session_q_index_[i]=node.impedance_left_->getJointQIdx(name);node.session_v_index_[i]=node.impedance_left_->getJointVIdx(name);}
 node.session_impedance_workspace();check_adapter_memory(node);
 node.session_execution_mode_="impedance";node.feedback();assert(node.session_impedance_ready());
 snapshot.sensor_valid_mask=1;assert(!node.session_impedance_ready());
 snapshot.sensor_valid_mask=3;snapshot.wrench[0]=NAN;assert(!node.session_impedance_ready());
 snapshot.wrench[0]=0;node.feedback();snapshot.sampled_ns-=21000000;assert(!node.session_impedance_ready());
 node.feedback();
 const auto published=rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::calls;
 check_target_impedance(node);
 assert(rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::calls==published);
 node.impedance_left_->publishPendingStatus();node.impedance_right_->publishPendingStatus();
 assert(rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::calls==published+4);
 node.impedance_left_->publishPendingStatus();node.impedance_right_->publishPendingStatus();
 assert(rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::calls==published+4);
 std::cout<<"PASS session controller: no ROS publish in control updates; telemetry worker drains latest status/effort once\n";
 std::cout<<"PASS actual session_v3 impedance inputs, sensor feedback, waist position, nominal PAUSE and source expiry\n";
}

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
 unsigned version=1;uint64_t segment_id=0;int64_t effective_at_ns=0;
 std::vector<double> piece_start,duration,source_duration,source_elapsed,controls;
 std::vector<unsigned char> control_count;std::array<double,16> max_velocity{},max_acceleration{},max_jerk{};
 bool operator!=(const TrajectoryWire& b)const{return std::tie(version,segment_id,effective_at_ns,piece_start,duration,source_duration,source_elapsed,controls,control_count,max_velocity,max_acceleration,max_jerk)!=std::tie(b.version,b.segment_id,b.effective_at_ns,b.piece_start,b.duration,b.source_duration,b.source_elapsed,b.controls,b.control_count,b.max_velocity,b.max_acceleration,b.max_jerk);}
};

struct Packet {
 using SharedPtr=std::shared_ptr<Packet>;
 struct Metadata {
  uint64_t session=0,sequence=0,feedback_boot_id=0,feedback_sample=0,middle_instance_id=0,epoch=4;
  int64_t source_stamp_ns=0,expires_at_ns=0;uint8_t operation=1,reference_mode=1;
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
 bool impedance_enable_=false,session_calibration_confirmed_=false,impedance_inited_=true,session_impedance_active_=false;
 uint64_t session_impedance_session_=0;
 std::shared_ptr<snowbot_impedance::CartesianImpedanceController> impedance_left_,impedance_right_;
 std::array<int,20> session_q_index_{},session_v_index_{};
 position_session::Gate position_gate_;position_session::Tracker position_feedback_;
 position_session::Interval position_interval_;position_session::Follower position_follower_;
 position_session::Output position_output_;
 position_session::ServoAdmissionState position_v2_;
 uint64_t position_instance_id_=47,position_rejected_session_=0,position_rejected_sequence_=0;
 bool position_initialized_=true,position_startup_complete_=true,shutdown_mode_=false,position_local_hold_=false;
 const char*position_cycle_reason_=nullptr;uint8_t position_policy_=2;
 Packet::SharedPtr position_origin_packet_;
 std::unique_ptr<position_session::TrajectoryAdmission<Packet>> position_admission_;
 snowbot_impedance::VectorXd session_q_,session_dq_,session_efforts_,session_nominal_,session_velocity_;
 uint8_t position_reference_mode_=0;position_session::TrajectoryQueue position_trajectory_;
 struct Time {int64_t nanoseconds()const{return position_session::steady_ns();}};Time now()const{return {};}
 bool session_impedance_ready()const;bool session_impedance_step();void position_step();void position_poll();
 void session_impedance_workspace();bool session_impedance_inputs(const position_session::Observation&);
 void feedback(){auto t=position_session::steady_ns();snapshot.sampled_ns=snapshot.attempted_ns=t;snapshot.sensor_sampled_ns.fill(t);++snapshot.sample;
  position_feedback_.boot=9;position_feedback_.sample=snapshot.sample;if(!position_feedback_.ready_sample){position_feedback_.ready_sample=snapshot.sample;position_feedback_.ready_ros=t;}
  position_feedback_.advances=2;position_feedback_.expected=position_feedback_.wkc=67;position_feedback_.valid=0xfffff;
  position_feedback_.age=0;position_feedback_.progress=position_feedback_.source=t;
  for(unsigned i=0;i<16;++i)position_feedback_.q[i]=snapshot.position[position_session::position_slot(i)];
 }
 void start(uint8_t operation=1,uint64_t sequence=1){
  feedback();auto t=position_session::steady_ns();position_session::Meta m{99,sequence,t,t+200000000,operation};position_gate_.commit(m,t,t);
  position_follower_.seed(position_feedback_.q);position_interval_.reset(t);
  position_output_.selected=true;position_output_.session=99;position_output_.sequence=sequence;position_output_.operation=operation;position_output_.policy=2;position_output_.deadline=position_gate_.deadline();position_output_.failure_reason=0;position_output_.source_stamp_ns=m.stamp;position_output_.expires_at_ns=m.expires;position_output_.admission_generation=position_gate_.generation();position_local_hold_=false;
  position_output_.kp.fill(40);position_output_.kd.fill(2);position_output_.torque.fill(0);
  position_output_.hold_kp=position_output_.kp;position_output_.hold_kd=position_output_.kd;
 }
};

// PRODUCTION_POLL_METHODS

// PRODUCTION_METHODS


TrajectoryWire described(const bindu::execution::PreparedSegment& segment,uint64_t sequence,int64_t effective){
 TrajectoryWire wire;wire.segment_id=sequence;wire.effective_at_ns=effective;wire.max_velocity.fill(.25);wire.max_acceleration.fill(.5);wire.max_jerk.fill(162);
 for(const auto& d:segment.describe()){
  wire.piece_start.push_back(d.start);wire.duration.push_back(d.duration);wire.source_duration.push_back(d.source_duration);wire.source_elapsed.push_back(d.source_elapsed);
  for(const auto& axis:d.controls){wire.control_count.push_back(axis.size());wire.controls.insert(wire.controls.end(),axis.begin(),axis.end());}
 }
 return wire;
}
void check_adapter_memory(ServoNode& node){
 auto original=snapshot;for(unsigned i=0;i<20;++i){snapshot.position[i]=float(i)*.001f;snapshot.speed[i]=float(i)*.002f;}
 node.position_follower_.q.fill(.1);node.position_follower_.v.fill(.02);
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
void check_production_poll(){
 using namespace position_session;using namespace bindu::execution;
 ServoNode node;node.position_reference_mode_=1;node.feedback();
 node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_,[](auto& result){
  inspect_entered=true;while(inspect_hold.load())std::this_thread::yield();
  if(auto error=inspect_servo_command(*result.packet,result.origin.get(),result.context,result.command))throw std::invalid_argument(error);
 });
 auto make_packet=[&](uint64_t session){
  node.feedback();auto p=std::make_shared<Packet>();auto t=steady_ns();
  p->meta.session=session;p->meta.sequence=1;p->meta.source_stamp_ns=t;p->meta.expires_at_ns=t+200000000;
  p->meta.feedback_boot_id=node.position_feedback_.boot;p->meta.feedback_sample=node.position_feedback_.sample;
  p->meta.middle_instance_id=32;
  JointLimits limits;JointState state;state.q.resize(16);state.v.resize(16);state.a.resize(16);
  for(unsigned i=0;i<16;++i){state.q[i]=node.position_feedback_.q[i];limits.push_back({lower[i],upper[i],.25,.5,162});}
  for(unsigned i=0;i<7;++i){p->command.l_position[i]=state.q[i];p->command.r_position[i]=state.q[i+7];p->command.l_kp[i]=p->command.r_kp[i]=40;p->command.l_kd[i]=p->command.r_kd[i]=2;}
  p->waist_position={state.q[14],state.q[15]};auto target=state.q;target[0]+=.001;
  p->trajectory=described(PreparedSegment::target(limits,state,target,0.,30.,.2),1,t+100000000);return p;
 };
 auto submit=[&](Packet::SharedPtr p){
  const auto context=node.position_v2_.context(node.position_gate_,node.position_instance_id_,"publisher");
  inspect_entered=false;assert(node.position_admission_->submit(p,.25,.5,162,context,node.position_origin_packet_));
 };
 auto finish=[&](int64_t sensor_age=0){
  auto until=steady_ns()+1000000000;periodic_allocations=periodic_deletions=0;
  while(node.position_admission_->busy()){
   node.feedback();if(sensor_age)snapshot.sensor_sampled_ns.fill(steady_ns()-sensor_age);count_periodic_memory=true;node.position_poll();count_periodic_memory=false;
   assert(periodic_allocations==0&&periodic_deletions==0);assert(steady_ns()<until);
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
 };
 auto p=make_packet(61);inspect_hold=true;submit(p);
 while(!inspect_entered.load())std::this_thread::yield();
 node.position_poll();assert(!node.position_gate_.selected()); // Imported work has not been committed/ACKed.
 inspect_hold=false;finish();assert(node.position_gate_.selected()&&node.position_gate_.last().session==61&&node.position_origin_packet_==p);
 assert(!node.position_cycle_reason_&&node.position_output_.selected&&node.position_v2_.frame==1);
 auto derived=std::make_shared<Packet>(*p);derived->frame_sequence=2;submit(derived);finish();assert(node.position_v2_.frame==2&&node.position_gate_.generation()==1);
 // In-flight work cannot survive revocation, then cannot survive instance change.
 auto next=make_packet(62);inspect_hold=true;submit(next);while(!inspect_entered.load())std::this_thread::yield();
 node.position_gate_.interrupt();inspect_hold=false;finish();assert(node.position_gate_.last().session==61);
 assert(std::string(node.position_cycle_reason_)=="admission_generation_changed");
 next=make_packet(63);inspect_hold=true;submit(next);while(!inspect_entered.load())std::this_thread::yield();
 ++node.position_instance_id_;inspect_hold=false;finish();assert(node.position_gate_.last().session==61);
 assert(std::string(node.position_cycle_reason_)=="admission_generation_changed");
 next=make_packet(64);next->command.l_position[0]=NAN;submit(next);finish();
 assert(node.position_gate_.last().session==61&&std::string(node.position_cycle_reason_)=="joint_limit");
 const auto admission=node.position_gate_.generation();const auto enables=enable_requests;
 node.session_execution_mode_="impedance";
 for(unsigned invalid=0;invalid<5;++invalid){
  node.position_initialized_=invalid!=0;node.session_calibration_confirmed_=invalid!=1;node.impedance_inited_=invalid!=2;
  snapshot.sensor_valid_mask=invalid==3?1:3;
  next=make_packet(65+invalid);submit(next);finish(invalid==4?21000000:0);
  assert(node.position_gate_.generation()==admission&&node.position_gate_.last().session==61&&enable_requests==enables);
  assert(std::string(node.position_cycle_reason_)=="startup_shutdown_or_mode_conflict");
 }
 snapshot.sensor_valid_mask=3;
 std::cout<<"PASS actual production position_poll: import/derived/error paths, ACK after commit, generation/restart rejection, initialized/calibration/sensor admission gates, no rejected enable requests, zero periodic new/delete\n";
}
void check_trajectory_impedance(ServoNode& node){
 using namespace position_session;using namespace bindu::execution;
 snapshot.wrench.fill(0);snapshot.sensor_valid_mask=3;node.start();node.position_reference_mode_=1;
 node.position_follower_.q[14]=.4; // The trajectory path must bypass this legacy follower.
 JointLimits limits;JointState state;state.q.resize(16);state.v.resize(16);state.a.resize(16);
 for(unsigned i=0;i<16;++i){state.q[i]=snapshot.position[position_slot(i)];limits.push_back({lower[i],upper[i],.25,.5,162});}
 auto goal=state.q;goal[0]+=.001;goal[14]+=.001;
 const auto segment=PreparedSegment::target(limits,state,goal,0.,30.,.2);
 auto now=steady_ns();Meta source{99,1,now,now+200000000,1};auto wire=described(segment,1,now+5000000);
 node.position_gate_.commit(source,now,now);node.position_output_.impedance_mode=true;
 node.position_trajectory_.accept(wire,source,now,now,node.position_feedback_.q,{});
 node.position_interval_.reset(now-1000000);node.feedback();node.position_step();
 assert(applied.trajectory&&applied.impedance&&applied.trajectory_phase==1&&applied.q[14]==snapshot.position[18]);
 std::this_thread::sleep_for(std::chrono::milliseconds(6));node.feedback();node.position_interval_.reset(steady_ns()-1000000);node.position_step();
 assert(applied.impedance&&applied.reference_position==node.position_trajectory_.q&&applied.reference_speed==node.position_trajectory_.v);
 for(uint8_t phase:{uint8_t(2),uint8_t(3)}){
  auto pending=applied;pending.operation=phase==3?2:1;pending.trajectory_phase=phase;pending.nominal_hold=false;
  Watchdog expiry;auto feedback=snapshot;auto t=steady_ns();feedback.sampled_ns=t;feedback.sensor_sampled_ns.fill(t);expiry.evaluate(pending,feedback,t);assert(!expiry.disabled);
  t=pending.deadline;pending.rt_deadline=t+20000000;feedback.sampled_ns=t;feedback.sensor_sampled_ns.fill(t);expiry.evaluate(pending,feedback,t);
  assert(expiry.reason==EXPIRED&&!expiry.output.impedance&&!expiry.nominal_pause);
 }

 now=steady_ns();Meta pause{99,2,now,now+200000000,2};const auto effective=now+3000000;
 const auto brake=PreparedSegment::stop(limits,segment.sample((effective-wire.effective_at_ns)*1e-9),0.,.19);
 auto pause_wire=described(brake,2,effective);node.position_trajectory_.accept(pause_wire,pause,now,now,node.position_feedback_.q,{});
 node.position_gate_.commit(pause,now,now);
 bool braking=false,held=false;
 for(unsigned tick=0;tick<100 && !held;++tick){
  std::this_thread::sleep_for(std::chrono::milliseconds(1));node.feedback();node.position_interval_.reset(steady_ns()-1000000);node.position_step();
  assert(applied.impedance&&applied.kp[0]==1&&applied.kd[0]==.2f&&applied.failure_reason==0);
  braking|=applied.trajectory_phase==3;held=applied.trajectory_phase==4;
 }
 assert(braking&&held&&node.session_impedance_active_&&applied.nominal_hold);
 Watchdog guard;node.feedback();guard.evaluate(applied,snapshot,steady_ns());guard.report_hold(snapshot,controlled_mask);
 assert(guard.reason==PAUSED&&!guard.disabled&&snapshot.hold_valid&&snapshot.hold_basis==1);
 assert(guard.output.impedance&&guard.output.q==applied.q&&guard.output.kp==applied.kp&&guard.output.kd==applied.kd&&guard.output.torque==applied.torque);
 const auto original_deadline=applied.deadline,original_stamp=applied.source_stamp_ns,original_expires=applied.expires_at_ns;
 const auto fixed_nominal=applied.reference_position;const auto old_torque=applied.torque;const auto old_writes=gravity_writes;
 snapshot.position[0]+=.005f; // Feedback displacement must update torque against the same fixed nominal.
 while(steady_ns()<original_deadline+30000000){
  std::this_thread::sleep_for(std::chrono::milliseconds(1));node.feedback();node.position_interval_.reset(steady_ns()-1000000);node.position_step();
  const auto t=steady_ns();guard.evaluate(applied,snapshot,t);snapshot.attempted_ns=t;guard.report_hold(snapshot,controlled_mask);
  assert(!guard.disabled&&guard.reason==PAUSED&&snapshot.hold_valid&&snapshot.hold_basis==1);
  assert(applied.impedance&&applied.nominal_hold&&applied.kp[0]==1&&applied.kd[0]==.2f);
  assert(applied.deadline==original_deadline&&node.position_gate_.deadline()==original_deadline&&applied.source_stamp_ns==original_stamp&&applied.expires_at_ns==original_expires);
  assert(applied.reference_position==fixed_nominal&&guard.output.torque==applied.torque);
 }
 assert(gravity_writes>old_writes&&applied.torque!=old_torque);
 // The local exception preserves every safety gate after the original TTL.
 for(auto cause:{FEEDBACK_INVALID,DRIVE_FAULT,IMPEDANCE_INVALID,CYCLE_OR_TRACKING_ERROR,CONTROL_SOURCE_CONFLICT}){
  auto failed=guard;auto input=applied;auto feedback=snapshot;auto t=steady_ns();
  feedback.sampled_ns=feedback.attempted_ns=t;feedback.sensor_sampled_ns.fill(t);input.rt_deadline=t+20000000;
  if(cause==FEEDBACK_INVALID)feedback.wkc=61;
  else if(cause==DRIVE_FAULT)feedback.status[0]|=8;
  else if(cause==IMPEDANCE_INVALID)feedback.sensor_valid_mask=1;
  else if(cause==CYCLE_OR_TRACKING_ERROR)input.rt_deadline=t;
  else input.failure_reason=cause;
  failed.evaluate(input,feedback,t);failed.report_hold(feedback,controlled_mask);assert(failed.disabled&&failed.reason==cause&&!feedback.hold_valid);
  input=applied;input.rt_deadline=t+40000000;feedback=snapshot;feedback.sampled_ns=feedback.attempted_ns=t+1000000;feedback.sensor_sampled_ns.fill(t+1000000);
  failed.evaluate(input,feedback,t+1000000);failed.report_hold(feedback,controlled_mask);assert(failed.disabled&&failed.reason==cause&&!feedback.hold_valid);
 }
 // A phase-4 claim first observed after expiry cannot invent a local hold.
 auto late=applied;Watchdog unadopted;late.rt_deadline=steady_ns()+20000000;node.feedback();unadopted.evaluate(late,snapshot,steady_ns());assert(unadopted.reason==EXPIRED&&!unadopted.output.impedance);
 std::cout<<"PASS actual nominal impedance PAUSE held beyond original TTL with fixed nominal/deadline, live gains/torque updates, hold_basis=1 and all safety failures latched\n";
}

int main(int argc,char**argv){assert(argc==2);ServoNode node;
 snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);snapshot.sensor_valid_mask=3;
 snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;
 node.position_step();assert(writes==0);node.start();node.position_step();assert(writes==1&&!applied.impedance&&applied.kp[0]==40);
 auto ros=std::make_shared<rclcpp::Node>();
 for(const std::string side:{"left","right"}){auto cfg=snowbot_impedance::ImpedanceConfig::makeDefault(side,argv[1]);cfg.session_reference_only=true;cfg.filter_params_pose=cfg.filter_params_nullspace=1.;cfg.tcp_to_ee={0,0,.192,0,0,0,1};auto ctl=std::make_shared<snowbot_impedance::CartesianImpedanceController>();assert(ctl->init(ros,cfg));if(side=="left")node.impedance_left_=ctl;else node.impedance_right_=ctl;}
 for(unsigned i=0;i<20;++i){const std::string name=i<8?"arm_l_joint_"+std::to_string(i+1):i<16?"arm_r_joint_"+std::to_string(i-7):i<18?"head_joint_"+std::to_string(i-15):"body_joint_"+std::to_string(i-17);node.session_q_index_[i]=node.impedance_left_->getJointQIdx(name);node.session_v_index_[i]=node.impedance_left_->getJointVIdx(name);}
 node.session_impedance_workspace();check_adapter_memory(node);
 check_production_poll();
 node.session_execution_mode_="impedance";node.feedback();assert(!node.session_impedance_ready());
 node.session_calibration_confirmed_=true;node.start();assert(node.session_impedance_ready());node.position_step();assert(applied.impedance&&applied.kp[0]==1&&applied.kd[0]==.2f&&std::isfinite(applied.torque[0]));
 assert(applied.q[14]==snapshot.position[18]&&applied.q[15]==snapshot.position[19]);
 assert(gravity[7]==0&&gravity[15]==0&&gravity[16]==0&&gravity[17]==0);
 // Actual production watchdog, so ROS repetition cannot keep old torque alive.
 auto live=applied;position_session::Watchdog watchdog;node.feedback();watchdog.evaluate(live,snapshot,position_session::steady_ns());assert(!watchdog.disabled);
 snapshot.sampled_ns=live.deadline;snapshot.sensor_sampled_ns.fill(live.deadline);watchdog.evaluate(live,snapshot,live.deadline);assert(watchdog.reason==position_session::EXPIRED&&!watchdog.disabled&&!watchdog.output.impedance);assert(watchdog.output.kp[0]==40&&watchdog.output.torque==live.hold_torque);
 live.policy=1;watchdog.evaluate(live,snapshot,live.deadline);assert(watchdog.disabled);
 node.start(2,2);node.position_step();assert(!applied.impedance&&applied.kp[0]==40);assert(!node.session_impedance_active_);
 node.start(1,3);snapshot.sensor_valid_mask=1;node.position_step();assert(applied.failure_reason==position_session::IMPEDANCE_INVALID&&!node.position_gate_.live(position_session::steady_ns()));
 position_session::Watchdog sensor_guard;sensor_guard.evaluate(live,snapshot,position_session::steady_ns());assert(sensor_guard.disabled&&sensor_guard.reason==position_session::IMPEDANCE_INVALID);
 snapshot.sensor_valid_mask=3;snapshot.wrench[0]=NAN;node.feedback();assert(!node.session_impedance_ready());
 snapshot.wrench[0]=0;node.feedback();snapshot.sampled_ns-=21000000;assert(!node.session_impedance_ready());
 check_trajectory_impedance(node);
 std::cout<<"PASS actual servo default position, gated impedance, waist output, sensor/calibration rejection, fixed-position PAUSE, RT expiry torque discard and resource isolation\n";
}

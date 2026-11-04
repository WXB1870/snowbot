#include "position_receipt.h"

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
 std::string position_rejection_;
 bool impedance_enable_=false;
 position_session::Gate position_gate_;position_session::Tracker position_feedback_;
 position_session::Interval position_interval_;position_session::Follower position_follower_;
 position_session::Output position_output_;
 position_session::ServoAdmissionState position_v2_;
 uint64_t position_instance_id_=47,position_rejected_session_=0,position_rejected_sequence_=0;
 bool position_initialized_=true,position_startup_complete_=true,shutdown_mode_=false,position_local_hold_=false;
 const char*position_cycle_reason_=nullptr;uint8_t position_policy_=2;
 Packet::SharedPtr position_origin_packet_;
 std::unique_ptr<position_session::TrajectoryAdmission<Packet>> position_admission_;
 bool position_receipts_enabled_=true;position_session::ReceiptQueue position_receipts_;
 uint8_t position_reference_mode_=0;position_session::TrajectoryQueue position_trajectory_;
 struct Time {int64_t nanoseconds()const{return position_session::steady_ns();}};Time now()const{return {};}
 void position_step();void position_poll();
 void feedback(){auto t=position_session::steady_ns();snapshot.sampled_ns=snapshot.attempted_ns=t;++snapshot.sample;
  position_feedback_.boot=9;position_feedback_.sample=snapshot.sample;if(!position_feedback_.ready_sample){position_feedback_.ready_sample=snapshot.sample;position_feedback_.ready_ros=t;}
  position_feedback_.advances=2;position_feedback_.expected=position_feedback_.wkc=67;position_feedback_.valid=0xfffff;
  position_feedback_.age=0;position_feedback_.progress=position_feedback_.source=t;
  for(unsigned i=0;i<16;++i)position_feedback_.q[i]=snapshot.position[position_session::position_slot(i)];
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
 auto finish=[&](){
  auto until=steady_ns()+1000000000;periodic_allocations=periodic_deletions=0;
  while(node.position_admission_->busy()){
   node.feedback();count_periodic_memory=true;node.position_poll();count_periodic_memory=false;
   assert(periodic_allocations==0&&periodic_deletions==0);assert(steady_ns()<until);
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
 };
 auto p=make_packet(61);inspect_hold=true;submit(p);
 while(!inspect_entered.load())std::this_thread::yield();
 node.position_poll();assert(!node.position_gate_.selected()); // Imported work has not been committed/ACKed.
 AdmissionReceipt receipt;assert(!node.position_receipts_.pop(receipt));
 inspect_hold=false;finish();assert(node.position_gate_.selected()&&node.position_gate_.last().session==61&&node.position_origin_packet_==p);
 assert(!node.position_cycle_reason_&&node.position_output_.selected&&node.position_v2_.frame==1);
 assert(node.position_receipts_.pop(receipt)&&receipt.accepted&&receipt.session==61&&receipt.sequence==1&&receipt.boot==9);
 auto derived=std::make_shared<Packet>(*p);derived->frame_sequence=2;submit(derived);finish();assert(node.position_v2_.frame==2&&node.position_gate_.generation()==1);
 assert(!node.position_receipts_.pop(receipt));
 // In-flight work cannot survive revocation, then cannot survive instance change.
 auto next=make_packet(62);inspect_hold=true;submit(next);while(!inspect_entered.load())std::this_thread::yield();
 node.position_gate_.interrupt();inspect_hold=false;finish();assert(node.position_gate_.last().session==61);
 assert(std::string(node.position_cycle_reason_)=="admission_generation_changed");
 assert(node.position_receipts_.pop(receipt)&&!receipt.accepted&&receipt.session==next->meta.session);
 next=make_packet(63);inspect_hold=true;submit(next);while(!inspect_entered.load())std::this_thread::yield();
 ++node.position_instance_id_;inspect_hold=false;finish();assert(node.position_gate_.last().session==61);
 assert(std::string(node.position_cycle_reason_)=="admission_generation_changed");
 assert(node.position_receipts_.pop(receipt)&&!receipt.accepted&&receipt.session==next->meta.session);
 next=make_packet(64);next->command.l_position[0]=NAN;submit(next);finish();
 assert(node.position_gate_.last().session==61&&std::string(node.position_cycle_reason_)=="joint_limit");
 assert(node.position_receipts_.pop(receipt)&&!receipt.accepted&&receipt.session==64);
 const auto admission=node.position_gate_.generation();const auto enables=enable_requests;
 node.position_initialized_=false;
 next=make_packet(65);submit(next);finish();
 assert(node.position_gate_.generation()==admission&&node.position_gate_.last().session==61&&enable_requests==enables);
 assert(std::string(node.position_cycle_reason_)=="startup_shutdown_or_mode_conflict");
 assert(node.position_receipts_.pop(receipt)&&!receipt.accepted&&receipt.session==65);
 std::cout<<"PASS actual production position_poll: import/derived/error paths, ACK after commit, generation/restart rejection, initialized admission gate, no rejected enable requests, zero periodic new/delete\n";
}

#include "position_session_contract.h"
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
struct Observation {
 struct {struct {int32_t sec=10;uint32_t nanosec=0;} stamp;} header;
 uint64_t boot_id=1,pdo_sample=1;double pdo_age=.001;
 int wkc=67,expected_wkc=67;uint32_t valid_mask=0xfffff;
 std::array<uint16_t,20> status{};
 std::array<double,20> position{},speed{};
 std::array<double,20> current{};
};
#ifdef POSITION_NATIVE_OBSERVATION
namespace snowbot_ctl_msgs {namespace msg {using PositionObservation=::Observation;}}
constexpr unsigned LARMDOF=7,RARMDOF=7;
struct ArmCtlNode {
 position_session::Tracker position_feedback_;
 bool position_initialized_=false,native_feedback_seen_=false,has_received_command_=false;
 std::array<double,7> lArm_pos_fdback{},rArm_pos_fdback{},lArm_spd_fdback{},rArm_spd_fdback{},lArm_iq_fdback{},rArm_iq_fdback{};
 bool position_native_feedback(const snowbot_ctl_msgs::msg::PositionObservation&,int64_t,int64_t);
};
// @ACTUAL_NATIVE_FEEDBACK@
#endif
static void receive(position_session::Tracker& tracker,Observation& msg,uint64_t seq,int64_t offset) {
 msg.pdo_sample=seq;msg.header.stamp.nanosec=uint32_t(offset);
 tracker.observe(msg,10000000000LL+offset,1000000000LL+offset);
}
static bool fresh(const position_session::Tracker& tracker,int64_t offset) {
 return tracker.fresh(10000000000LL+offset,1000000000LL+offset);
}
int main() {
 unsigned failures=0;
#ifdef POSITION_NATIVE_OBSERVATION
 {
  ArmCtlNode node;Observation msg;
  for(unsigned i=0;i<20;++i){msg.position[i]=double(i+1);msg.speed[i]=-.01*(i+1);msg.current[i]=.25*(i+1);}
  receive(node.position_feedback_,msg,1,0);receive(node.position_feedback_,msg,2,1000000);
  node.position_native_feedback(msg,10001000000LL,1001000000LL);
  if(node.native_feedback_seen_||node.has_received_command_)++failures;
  node.position_initialized_=true;
  if(!node.position_native_feedback(msg,10001000000LL,1001000000LL)||!node.native_feedback_seen_||!node.has_received_command_)++failures;
  if(node.lArm_pos_fdback.front()!=1.||node.lArm_pos_fdback.back()!=7.||node.rArm_pos_fdback.front()!=9.||node.rArm_pos_fdback.back()!=15.)++failures;
  if(node.rArm_spd_fdback.front()!=-.09||node.rArm_iq_fdback.front()!=2.25)++failures;
  const auto previous=node.lArm_pos_fdback;
  msg.pdo_age=.021;msg.position[0]=100.;receive(node.position_feedback_,msg,3,2000000);
  if(node.position_native_feedback(msg,10002000000LL,1002000000LL)||node.lArm_pos_fdback!=previous)++failures;
  msg.pdo_age=.001;msg.wkc=66;receive(node.position_feedback_,msg,4,3000000);
  if(node.position_native_feedback(msg,10003000000LL,1003000000LL)||node.lArm_pos_fdback!=previous)++failures;
  std::cout<<"PASS actual middle PDO feedback: initialization guard, measured left/right motor-slot mapping, original velocity/current units, stale/WKC rejection without fabricated fallback\n";
 }
#endif
 {position_session::Tracker tracker;Observation msg;
  receive(tracker,msg,1,0);receive(tracker,msg,2,1000000);
  if(!fresh(tracker,1000000)){std::cerr<<"valid baseline unavailable\n";return 2;}
  msg.position[0]=std::numeric_limits<double>::quiet_NaN();receive(tracker,msg,3,2000000);
  if(fresh(tracker,2000000)){std::cerr<<"nonfinite sample accepted\n";++failures;}
  msg.position[0]=0;receive(tracker,msg,4,3000000);
  if(fresh(tracker,3000000)){std::cerr<<"FAIL: first valid sample after NaN reused old baseline\n";++failures;}
 }
 {position_session::Tracker tracker;Observation msg;
  const auto late=position_session::ros_feedback_timeout_ns+1000000;
  receive(tracker,msg,1,0);receive(tracker,msg,2,late);
  if(fresh(tracker,late)){std::cerr<<"FAIL: delayed second sample reused baseline beyond ROS feedback budget\n";++failures;}
 }
 {position_session::Tracker tracker;Observation msg;msg.position[18]=.12;msg.position[19]=-.08;
  receive(tracker,msg,1,0);receive(tracker,msg,2,1000000);
  if(!fresh(tracker,1000000)||tracker.q[14]!=.12||tracker.q[15]!=-.08)++failures;
  msg.status[19]=8;receive(tracker,msg,3,2000000);if(fresh(tracker,2000000))++failures;
  msg.status[19]=0;msg.valid_mask&=~(1u<<18);receive(tracker,msg,4,3000000);if(fresh(tracker,3000000))++failures;
  msg.valid_mask|=1u<<18;msg.speed[18]=std::numeric_limits<double>::quiet_NaN();receive(tracker,msg,5,4000000);if(fresh(tracker,4000000))++failures;
  msg.speed[18]=0;msg.status[7]=msg.status[16]=8;receive(tracker,msg,6,5000000);receive(tracker,msg,7,6000000);if(!fresh(tracker,6000000))++failures;
 }
 std::cout<<"tracker reconnect failures="<<failures<<"\n";
 return failures ? 1:0;
}

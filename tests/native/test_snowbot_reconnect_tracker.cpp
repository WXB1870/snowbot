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
};
static void receive(position_session::Tracker& tracker,Observation& msg,uint64_t seq,int64_t offset) {
 msg.pdo_sample=seq;msg.header.stamp.nanosec=uint32_t(offset);
 tracker.observe(msg,10000000000LL+offset,1000000000LL+offset);
}
static bool fresh(const position_session::Tracker& tracker,int64_t offset) {
 return tracker.fresh(10000000000LL+offset,1000000000LL+offset);
}
int main() {
 unsigned failures=0;
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

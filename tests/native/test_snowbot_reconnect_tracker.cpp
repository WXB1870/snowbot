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
  receive(tracker,msg,1,0);receive(tracker,msg,2,30000000);
  if(fresh(tracker,30000000)){std::cerr<<"FAIL: delayed second sample reused baseline across 30ms gap\n";++failures;}
 }
 std::cout<<"tracker reconnect failures="<<failures<<"\n";
 return failures ? 1:0;
}

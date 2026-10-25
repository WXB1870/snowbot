// Production callback pump, with a deterministic executor double; no ROS/IO.
#include "servo_control/position_clock.h"
#include "position_session_contract.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <new>
#include <type_traits>
#include <thread>
#include <cstring>
#include <cstdio>
#include <atomic>
static bool count_memory=false;
static unsigned allocations=0,deletions=0;
void* operator new(std::size_t n){if(count_memory)++allocations;if(auto p=std::malloc(n))return p;throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{if(p&&count_memory)++deletions;std::free(p);}
void operator delete[](void* p)noexcept{::operator delete(p);}
namespace rclcpp { namespace executors {
struct SingleThreadedExecutor {
 static unsigned instances;
 unsigned adds=0,spins=0;void* node=nullptr;
 SingleThreadedExecutor(){++instances;}
 void add_node(void* n){assert(!adds);++adds;node=n;}
 void spin_some(){assert(node&&adds==1);++spins;}
};
unsigned SingleThreadedExecutor::instances=0;
}}
#ifdef POSITION_FEEDBACK_SUBPHASES
static std::atomic<int64_t> feedback_now{1000000000};
static int64_t feedback_clock(){return feedback_now.load();}
namespace snowbot_ctl_msgs {namespace msg {
struct RetInformation {
 struct {int64_t stamp=0;} header;
 std::array<float,8> l_position{},l_speed{},l_iq{},r_position{},r_speed{},r_iq{};
 int if_first_time=0;
};
}}
struct FeedbackPublisher {
 snowbot_ctl_msgs::msg::RetInformation value{};
 unsigned calls=0;std::atomic<bool> block{false},entered{false},release{false};
 void publish(const snowbot_ctl_msgs::msg::RetInformation& v){
  value=v;++calls;
  if(block.load()){entered.store(true);while(!release.load())std::this_thread::yield();}
  feedback_now+=3000000;
 }
};
static constexpr int LARMDOF=8;
struct Ret {float joint_posRet=0.,joint_spdRet=0.,joint_currentRet=0.;};
static std::array<Ret,16> servo_retMsg_unpacked{};
static bool getter_delay=true;
static void getGripperFb(int side,float& q,float& v,float& current,uint16_t& status){
 if(getter_delay)feedback_now+=side?2000000:196000000;
 q=side?2.f:1.f;v=.3f;current=.4f;status=0x218;
}
#endif
struct ServoNode {
 rclcpp::executors::SingleThreadedExecutor ros_executor_;
 bool ros_executor_added_=false;
 void* get_node_base_interface(){return this;}
#ifdef POSITION_FEEDBACK_SUBPHASES
 position_session::Gate position_gate_;
 position_session::LoopTiming position_feedback_timing_;
 FeedbackPublisher publisher;FeedbackPublisher* RetData_pub=&publisher;
 bool ifFirstTemp=true;
 int64_t now(){return feedback_now.load();}
 void msg_publish();
#ifdef POSITION_ASYNC_FEEDBACK
 struct FeedbackSample {snowbot_ctl_msgs::msg::RetInformation message;int64_t sampled_ns=0;};
 position_session::LatestTelemetry<FeedbackSample> feedback_mailbox_;
 bool feedback_first_sent_=false;
 void feedback_send();
#endif
#endif
 void ctlmsg_get();
};
// @ACTUAL_CALLBACK_PUMP@
// @ACTUAL_FEEDBACK_PUBLISH@
// @ACTUAL_FEEDBACK_SEND@
// @ACTUAL_TRANSPORT_SETUP@
int main(){
#ifdef POSITION_ASYNC_TRANSPORT
 unsetenv("RMW_FASTRTPS_PUBLICATION_MODE");assert(configure_transport()==0);
 assert(std::strcmp(std::getenv("RMW_FASTRTPS_PUBLICATION_MODE"),"ASYNCHRONOUS")==0);
 setenv("RMW_FASTRTPS_PUBLICATION_MODE","SYNCHRONOUS",1);assert(configure_transport()==0);
 assert(std::strcmp(std::getenv("RMW_FASTRTPS_PUBLICATION_MODE"),"SYNCHRONOUS")==0);
 unsetenv("RMW_FASTRTPS_PUBLICATION_MODE");
#endif
 using Timing=position_session::LoopTiming;
 static_assert(std::is_trivially_copyable<Timing>::value,"fixed diagnostic value required");
 Timing timing;timing.mark(Timing::SpinHead,1000000000);assert(!timing.snapshot(1023000000).last[Timing::SpinHead].begin);
 timing.bind(42);timing.mark(Timing::SpinHead,1000000000);
 // An unfinished callback span is visible to a timer running inside that pump.
 auto s=timing.snapshot(1023000000);assert(s.longest[Timing::SpinHead].end-s.longest[Timing::SpinHead].begin==23000000);
 timing.mark(Timing::Compute,1025000000);timing.mark(Timing::SpinHead,1026000000);timing.mark(Timing::Sample,1027000000);
 s=timing.snapshot(1028000000);assert(s.last[Timing::SpinHead].end-s.last[Timing::SpinHead].begin==1000000);
 assert(s.longest[Timing::SpinHead].end-s.longest[Timing::SpinHead].begin==25000000);
 timing.bind(42);assert(timing.snapshot(1028000000).session==42);
 timing.bind(43);s=timing.snapshot(1029000000);for(auto span:s.longest)assert(!span.begin);
 ServoNode node;assert(node.ros_executor_.instances==1&&!node.ros_executor_.adds);
 // Both original pump sites use this actual method, in sequence on one thread.
 node.ctlmsg_get();count_memory=true;
 for(unsigned i=0;i<2000;++i){timing.mark(Timing::SpinTail,1030000000+i*1000000LL);(void)timing.snapshot(1030000001+i*1000000LL);node.ctlmsg_get();}
 count_memory=false;assert(!allocations&&!deletions);
 assert(node.ros_executor_.instances==1&&node.ros_executor_.adds==1&&node.ros_executor_.spins==2001&&node.ros_executor_.node==&node);
#ifdef POSITION_FEEDBACK_SUBPHASES
 // Actual feedback method with inert getter/publisher doubles. Long spans
 // retain the call boundary and cannot change the returned feedback values.
 position_session::Meta source{42,1,1,200000001,1};
 assert(node.position_gate_.accepts(source,1,1));node.position_gate_.commit(source,1,1);node.msg_publish();
 auto feedback=node.position_feedback_timing_.snapshot(feedback_now);
 assert(feedback.session==42);
 assert(feedback.longest[Timing::FeedbackPublish].end-feedback.longest[Timing::FeedbackPublish].begin==196000000);
 assert(feedback.longest[Timing::SpinHead].end-feedback.longest[Timing::SpinHead].begin==2000000);
#ifdef POSITION_ASYNC_FEEDBACK
 assert(node.publisher.calls==0); // msg_publish never invokes DDS
 node.feedback_send();assert(node.publisher.calls==0); // slow acquisition is stale too
 getter_delay=false;node.msg_publish();node.feedback_send();assert(node.publisher.calls==1);
 assert(node.publisher.value.l_position[7]==1.f&&node.publisher.value.r_position[7]==2.f);
 assert(node.publisher.value.if_first_time==1);node.feedback_send();assert(node.publisher.calls==1);
 // Simulate DDS stalled while the main thread continues handing off samples.
 // No timeout or sleeping assertion: release is possible only after all puts.
 node.msg_publish();node.publisher.block=true;
 std::thread stalled([&]{node.feedback_send();});
 while(!node.publisher.entered.load())std::this_thread::yield();
 for(unsigned i=0;i<10000;++i)node.msg_publish();
 const auto latest_stamp=feedback_now.load();
 node.publisher.release=true;stalled.join();node.publisher.block=false;
 node.feedback_send();assert(node.publisher.calls==3);
 assert(node.publisher.value.header.stamp==latest_stamp);
 assert(node.publisher.value.if_first_time==0);
 node.msg_publish();feedback_now+=21000000;node.feedback_send();assert(node.publisher.calls==3);
 node.feedback_send();assert(node.publisher.calls==3); // no replay of a consumed sample
 position_session::LatestTelemetry<std::array<unsigned,2>> mailbox;
 std::array<unsigned,2> received{};count_memory=true;
 for(unsigned i=1;i<=10000;++i)mailbox.put({i,i});
 assert(mailbox.take(received)&&received[0]==10000&&received[1]==10000&&!mailbox.take(received));
 count_memory=false;assert(!allocations&&!deletions);
 std::cout<<"PASS actual feedback handoff/send: blocked DDS isolated, latest coherent sample only, acquisition timestamp preserved, stale/duplicate dropped, first-sent marker retained, fixed storage\n";
#else
 assert(feedback.longest[Timing::Admission].end-feedback.longest[Timing::Admission].begin==3000000);
 assert(node.publisher.value.l_position[7]==1.f&&node.publisher.value.r_position[7]==2.f);
 const auto left=feedback.longest[Timing::FeedbackPublish];
 assert(left.begin<1020000000&&left.end>1020000000); // synthetic first-fault time
 std::cout<<"PASS actual feedback method: distinct getter/publish spans, retained 196ms boundary, unchanged sampled values\n";
#endif
#endif
 std::cout<<"PASS actual callback pump: one executor and one node attachment across both pump sites; bounded diagnostic spans include open stalls, session reset and zero counted allocation/deletion\n";
}

// Production callback pump, with a deterministic executor double; no ROS/IO.
#include "servo_control/position_clock.h"
#include "position_session_contract.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <new>
#include <type_traits>
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
static int64_t feedback_now=1000000000;
static int64_t feedback_clock(){return feedback_now;}
namespace snowbot_ctl_msgs {namespace msg {
struct RetInformation {
 struct {int64_t stamp=0;} header;
 std::array<float,8> l_position{},l_speed{},l_iq{},r_position{},r_speed{},r_iq{};
 int if_first_time=0;
};
}}
struct FeedbackPublisher {
 snowbot_ctl_msgs::msg::RetInformation value{};
 void publish(const snowbot_ctl_msgs::msg::RetInformation& v){value=v;feedback_now+=3000000;}
};
static constexpr int LARMDOF=8;
struct Ret {float joint_posRet=0.,joint_spdRet=0.,joint_currentRet=0.;};
static std::array<Ret,16> servo_retMsg_unpacked{};
static void getGripperFb(int side,float& q,float& v,float& current,uint16_t& status){
 feedback_now+=side?2000000:196000000;
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
 int64_t now(){return feedback_now;}
 void msg_publish();
#endif
 void ctlmsg_get();
};
// @ACTUAL_CALLBACK_PUMP@
// @ACTUAL_FEEDBACK_PUBLISH@
int main(){
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
 assert(feedback.longest[Timing::Admission].end-feedback.longest[Timing::Admission].begin==3000000);
 assert(node.publisher.value.l_position[7]==1.f&&node.publisher.value.r_position[7]==2.f);
 const auto left=feedback.longest[Timing::FeedbackPublish];
 assert(left.begin<1020000000&&left.end>1020000000); // synthetic first-fault time
 std::cout<<"PASS actual feedback method: distinct getter/publish spans, retained 196ms boundary, unchanged sampled values\n";
#endif
 std::cout<<"PASS actual callback pump: one executor and one node attachment across both pump sites; bounded diagnostic spans include open stalls, session reset and zero counted allocation/deletion\n";
}

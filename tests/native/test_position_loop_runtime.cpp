// Production callback pump, with a deterministic executor double; no ROS/IO.
#include "servo_control/position_clock.h"
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
struct ServoNode {
 rclcpp::executors::SingleThreadedExecutor ros_executor_;
 bool ros_executor_added_=false;
 void* get_node_base_interface(){return this;}
 void ctlmsg_get();
};
// @ACTUAL_CALLBACK_PUMP@
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
 std::cout<<"PASS actual callback pump: one executor and one node attachment across both pump sites; bounded diagnostic spans include open stalls, session reset and zero counted allocation/deletion\n";
}

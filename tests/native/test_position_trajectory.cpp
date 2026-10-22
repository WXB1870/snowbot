#include "position_trajectory.h"
#include "servo_control/position_watchdog.h"
#include "servo_control/position_clock.h"
#include <cassert>
#include <iostream>
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

using namespace position_session;
using namespace bindu::execution;
struct Wire {
 unsigned version=1;uint64_t segment_id=0;int64_t effective_at_ns=0;
 std::vector<double> piece_start,duration,source_duration,source_elapsed,controls;
 std::vector<unsigned char> control_count;
 std::array<double,16> max_velocity{},max_acceleration{},max_jerk{};
};
Wire wire(const PreparedSegment& segment,uint64_t id,int64_t effective){
 Wire w;w.segment_id=id;w.effective_at_ns=effective;w.max_velocity.fill(.25);w.max_acceleration.fill(.5);w.max_jerk.fill(162);
 for(const auto& d:segment.describe()){
  w.piece_start.push_back(d.start);w.duration.push_back(d.duration);w.source_duration.push_back(d.source_duration);w.source_elapsed.push_back(d.source_elapsed);
  for(const auto& axis:d.controls){w.control_count.push_back(axis.size());w.controls.insert(w.controls.end(),axis.begin(),axis.end());}
 }
 return w;
}
template<class F> void reject(F run){bool rejected=false;try{run();}catch(const std::invalid_argument&){rejected=true;}assert(rejected);}

std::atomic<bool> importing{false},import_finished{false};
struct SlowDuration:std::vector<double>{
 size_t size()const{importing=true;std::this_thread::sleep_for(std::chrono::milliseconds(100));import_finished=true;return std::vector<double>::size();}
};
struct DelayedWire:Wire {SlowDuration duration;};
struct AdmissionPacket {
 using SharedPtr=std::shared_ptr<AdmissionPacket>;
 struct {uint64_t session=7,sequence=2;int64_t source_stamp_ns=1020000000,expires_at_ns=1220000000;uint8_t operation=0,reference_mode=1;} meta;
 DelayedWire trajectory;
};
void check_admission_isolation(const PreparedSegment& first,const PreparedSegment& second,
 const std::array<double,16>& measured){
 TrajectoryQueue queue;std::array<double,16> speed{};Meta source{7,1,1000000000,1200000000,1};
 queue.accept(wire(first,1,1060000000),source,1000000000,1000000000,measured,speed);
 TrajectoryAdmission<AdmissionPacket> importer(queue);auto packet=std::make_shared<AdmissionPacket>();
 auto next=wire(second,2,1080000000);static_cast<Wire&>(packet->trajectory)=next;
 packet->trajectory.duration.assign(next.duration.begin(),next.duration.end());
 assert(importer.submit(packet,.25,.5,162));assert(!importer.submit(packet,.25,.5,162));
 while(!importing.load())std::this_thread::yield();
 // A deliberately blocked 100 ms importer cannot stall the periodic sampler.
 for(int i=0;i<1000;++i){assert(queue.sample(1060000000+i*1000));assert(queue.id()==1);}
 assert(!import_finished.load());
 TrajectoryAdmission<AdmissionPacket>::Result* result=nullptr;
 while(!(result=importer.poll()))std::this_thread::sleep_for(std::chrono::milliseconds(1));
 assert(result->error.empty()&&queue.sample(1070000000)&&queue.id()==1); // Import is not acceptance.
 assert(queue.commit(*result->prepared,1080000000,1080000000,measured,speed));
 assert(queue.sample(1070000000)&&queue.id()==1); // Recheck effective time after import.
 count_periodic_memory=true;
 assert(!queue.commit(*result->prepared,1020000000,1020000000,measured,speed));
 for(int i=0;i<1000;++i)assert(queue.sample(1080000000+i*1000));
 count_periodic_memory=false;assert(periodic_allocations==0&&periodic_deletions==0);
 assert(queue.sample(1080000000)&&queue.id()==2);importer.release(result);
 while(importer.busy())std::this_thread::sleep_for(std::chrono::milliseconds(1));
 JointLimits limits;for(unsigned i=0;i<16;++i)limits.push_back({lower[i],upper[i],.25,.5,162});
 auto third=PreparedSegment::target(limits,second.sample(.07),second.sample(second.end()).q,0.,30.,.2);
 packet=std::make_shared<AdmissionPacket>();packet->meta.sequence=3;
 packet->meta.source_stamp_ns=1090000000;packet->meta.expires_at_ns=1290000000;
 auto third_wire=wire(third,3,1150000000);static_cast<Wire&>(packet->trajectory)=third_wire;
 packet->trajectory.duration.assign(third_wire.duration.begin(),third_wire.duration.end());
 assert(importer.submit(packet,.25,.5,162));
 while(!(result=importer.poll()))std::this_thread::sleep_for(std::chrono::milliseconds(1));
 assert(result->error.empty());count_periodic_memory=true;
 assert(!queue.commit(*result->prepared,1090000000,1090000000,measured,speed)); // Retire an elapsed segment.
 for(int i=0;i<1000;++i)assert(queue.sample(1150000000+i*1000));
 count_periodic_memory=false;assert(periodic_allocations==0&&periodic_deletions==0);
 importer.release(result);while(importer.busy())std::this_thread::sleep_for(std::chrono::milliseconds(1));
 packet=std::make_shared<AdmissionPacket>();packet->trajectory.version=9;
 assert(importer.submit(packet,.25,.5,162));
 while(!(result=importer.poll()))std::this_thread::sleep_for(std::chrono::milliseconds(1));
 assert(!result->error.empty()&&queue.sample(1151000000)&&queue.id()==3);importer.release(result);
 std::cout<<"PASS 100 ms import isolation, single in-flight SPSC, post-import deadline check, zero commit/sample allocation or deletion, no adoption before commit and failed-import preservation\n";
}

int main(){
 RejectedTrajectory rejected;Meta failed{7,2,1020000000,1220000000,0};rejected.reject(failed,"trajectory_controls");
 for(unsigned relay=0;relay<1000;++relay){assert(rejected.matches(failed));assert(rejected.reason=="trajectory_controls");}
 auto delayed=failed;--delayed.sequence;assert(rejected.blocks(delayed));
 auto next_source=failed;++next_source.sequence;assert(!rejected.blocks(next_source));assert(!rejected.matches(next_source));rejected.clear();assert(!rejected.matches(failed));
 Interval cycle;cycle.reset(1000000000);cycle.admit(1020000000,false);
 assert(cycle.tick(1021000000)>.01); // New target cannot conceal 20 ms without sampling.
 cycle.admit(1030000000,true);assert(cycle.tick(1031000000)<.01);
 JointLimits limits;for(unsigned i=0;i<16;++i)limits.push_back({lower[i],upper[i],.25,.5,162});
 JointState initial;initial.q.resize(16);initial.q[1]=.2;initial.q[8]=-.2;initial.q[14]=.12;initial.q[15]=-.08;
 initial.v.resize(16);initial.a.resize(16);
 auto goal=initial.q;goal[0]+=.02;goal[14]+=.01;
 auto first=PreparedSegment::target(limits,initial,goal,0.,30.,.2);
 std::array<double,16> measured{},speed{};std::copy(initial.q.begin(),initial.q.end(),measured.begin());
 TrajectoryQueue queue;Meta m{7,1,1000000000,1200000000,1};auto a=wire(first,1,1060000000);
 queue.accept(a,m,1000000000,1000000000,measured,speed);
 assert(queue.sample(1001000000)&&queue.phase(1001000000)==1&&queue.q==measured);
 auto bad=a;bad.effective_at_ns=1000000000;reject([&]{queue.accept(bad,m,1000000000,1000000000,measured,speed);});
 bad=a;bad.controls[0]=NAN;reject([&]{queue.accept(bad,m,1000000000,1000000000,measured,speed);});
 bad=a;bad.max_velocity[0]=1.;reject([&]{queue.accept(bad,m,1000000000,1000000000,measured,speed);});
 auto state=first.sample(.02);goal[7]-=.01;auto second=PreparedSegment::target(limits,state,goal,0.,30.,.2);
 check_admission_isolation(first,second,measured);
 auto b=wire(second,2,1080000000);Meta second_source{7,2,1020000000,1220000000,0};
 bad=b;bad.controls[0]+=.001;reject([&]{queue.accept(bad,second_source,1020000000,1020000000,measured,speed);});
 queue.accept(b,second_source,1020000000,1020000000,measured,speed);
 assert(queue.sample(1079999999)&&queue.id()==1);
 assert(queue.sample(1080000000)&&queue.id()==2);
 for(unsigned i=0;i<16;++i){assert(std::abs(queue.q[i]-state.q[i])<1e-10);assert(std::abs(queue.v[i]-state.v[i])<1e-10);assert(std::abs(queue.a[i]-state.a[i])<1e-10);}
 auto pause_state=second.sample(.02);auto stop=PreparedSegment::stop(limits,pause_state,0.,.139);
 auto c=wire(stop,3,1100000000);Meta pause{7,3,1040000000,1240000000,2};queue.accept(c,pause,1040000000,1040000000,measured,speed);
 Observation obs;obs.wkc=obs.expected_wkc=67;obs.valid_mask=0xfffff;obs.status.fill(0x27);Watchdog watchdog;
 bool brake=false,held=false;
 for(int64_t now=1060000000;now<1239000000;now+=1000000){
  assert(queue.sample(now));Output output;output.selected=true;output.policy=2;output.trajectory=true;
  output.session=queue.source().session;output.sequence=queue.source().sequence;output.operation=queue.source().operation;
  output.deadline=queue.deadline();output.rt_deadline=now+20000000;output.trajectory_phase=queue.phase(now);
  output.reference_position=queue.q;output.reference_speed=queue.v;output.reference_acceleration=queue.a;
  output.q=std::array<float,16>{};for(unsigned i=0;i<16;++i){output.q[i]=queue.q[i];output.v[i]=queue.v[i];obs.position[position_slot(i)]=queue.q[i];}
  obs.sampled_ns=obs.attempted_ns=now;++obs.sample;watchdog.evaluate(output,obs,now);watchdog.report_hold(obs,controlled_mask);
  assert(!watchdog.disabled);
  if(output.trajectory_phase==3){brake=true;assert(!obs.hold_valid&&watchdog.reason==OK);}
  if(output.trajectory_phase==4){held=true;assert(obs.hold_valid&&watchdog.reason==PAUSED);}
  if(now==1198000000){auto frozen=watchdog;obs.sampled_ns=now+20000000;frozen.evaluate(output,obs,now+20000000);assert(frozen.disabled&&frozen.reason==CYCLE_OR_TRACKING_ERROR);}
 }
 assert(brake&&held);
 Output expired=watchdog.output;expired.rt_deadline=1250000000;obs.sampled_ns=1240000000;
 watchdog.evaluate(expired,obs,1240000000);assert(watchdog.reason==EXPIRED&&!watchdog.disabled);
 // Source deadline remains 1.24 s even after hundreds of derived samples.
 assert(queue.sample(1239000000)&&queue.deadline()==1240000000);
 std::cout<<"PASS 16-axis described curves, scheduled switch C2, malformed/late/limit rejection, 1ms sampling, controlled PAUSE and independent RT sample watchdog\n";
}

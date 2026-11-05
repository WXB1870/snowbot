#include "position_trajectory.h"
#include "position_shutdown.h"
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

void check_shutdown(){
 Shutdown action;TargetSnapshot seed;seed.q[1]=.2;seed.q[8]=-.2;seed.q[3]=-.916625;
 seed.v[0]=.03;seed.a[0]=.02;seed.valid=true;
 auto goal=seed.q;goal[0]+=.02;
 auto now=steady_ns();seed.effective_steady=now+50000000;
 action.begin(seed,goal,.25,.5,162.,now,8.);
 while(steady_ns()<seed.effective_steady){action.poll(steady_ns());std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 assert(action.poll(seed.effective_steady));assert(action.sampling());
 Observation obs;obs.sample=1;obs.valid_mask=0xfffff;obs.wkc=obs.expected_wkc=67;
 obs.status.fill(0x27);obs.output_valid=true;
 for(unsigned i=0;i<joint_count;++i)obs.position[position_slot(i)]=float(seed.q[i]);
 Output output;output.selected=true;output.policy=2;output.kp.fill(1);output.kd.fill(.2f);output.torque.fill(6);
 now=seed.effective_steady;obs.sampled_ns=now;action.tick(now,obs,output,.01);
 for(unsigned i=0;i<joint_count;++i){assert(std::abs(output.reference_position[i]-seed.q[i])<1e-12);assert(std::abs(output.reference_speed[i]-seed.v[i])<1e-12);assert(std::abs(output.reference_acceleration[i]-seed.a[i])<1e-12);}
 assert(output.kp[3]==1&&output.kd[3]==.2f&&output.torque[3]==6&&!output.shutdown_disable);
 bool stopped=false,returned=false,disabled=false;unsigned moving_feedback=0;
 for(unsigned n=0;n<20000&&action.phase!=Shutdown::Phase::Done;++n){
  now+=1000000;obs.sampled_ns=now;++obs.sample;
  // Explicit memory device response; a reference alone cannot confirm motion.
  for(unsigned i=0;i<joint_count;++i)obs.position[position_slot(i)]=float(output.reference_position[i]);
  obs.speed.fill(0);
  if(action.phase==Shutdown::Phase::StopHold && moving_feedback<600){obs.speed[0]=.1f;++moving_feedback;}
  action.tick(now,obs,output,.01);
  if(moving_feedback && moving_feedback<600)assert(action.phase==Shutdown::Phase::StopHold);
  stopped|=action.phase==Shutdown::Phase::StopHold;
  returned|=action.phase==Shutdown::Phase::Returning;
  if(output.shutdown_disable){disabled=true;assert(action.phase==Shutdown::Phase::Disabling || action.phase==Shutdown::Phase::Done);obs.shutdown_disabled=true;}
 }
 assert(stopped&&returned&&disabled&&moving_feedback==600&&!action.failed()&&action.phase==Shutdown::Phase::Done);
 Watchdog guard;Output packet;packet.selected=true;packet.policy=2;packet.operation=1;packet.admission_generation=1;packet.deadline=now+100000000;
 obs.shutdown_disabled=false;obs.sampled_ns=now;guard.evaluate(packet,obs,now);
 obs.wkc=28;guard.evaluate(packet,obs,now);assert(guard.latched_failure==FEEDBACK_INVALID);
 obs.wkc=67;packet.shutdown=true;packet.shutdown_deadline=now+1000000000;packet.admission_generation=99;
 guard.evaluate(packet,obs,now);assert(guard.disabled&&guard.latched_failure==FEEDBACK_INVALID);
 Shutdown failed;failed.disable(now,"existing_fault");failed.tick(now,obs,packet,.01);
 assert(packet.shutdown_disable&&failed.phase==Shutdown::Phase::Disabling);
 failed.tick(now+8000000000LL,obs,packet,.01);assert(failed.failed()&&failed.phase==Shutdown::Phase::Done);
 Shutdown stale;stale.disable(now);obs.shutdown_disabled=true;obs.sampled_ns=now-21000000;
 stale.tick(now,obs,packet,.01);assert(stale.phase==Shutdown::Phase::Disabling);
 obs.sampled_ns=now;stale.tick(now,obs,packet,.01);assert(stale.phase==Shutdown::Phase::Done);
 Shutdown late;seed.effective_steady=now-1;late.begin(seed,goal,.25,.5,162.,now,8.);
 assert(late.poll(now)&&late.failed()&&late.phase==Shutdown::Phase::Disabling);
 std::cout<<"PASS shutdown C2 first frame, unchanged gains/feedforward, fresh measured stop/home, disable ACK and retained fault latch\n";
}
int main(){
 check_shutdown();
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
 // A prepared replay is imported once. Heartbeats renew only its dated
 // authority, leaving the original start, q/v/a and polynomial identity intact.
 {
  TrajectoryQueue replay;Meta source{71,1,1000000000,1250000000,1};
  auto full=wire(first,1,1120000000);full.version=4;TargetSnapshot snapshot;
  auto imported=TrajectoryQueue::prepare_replay(full,source,snapshot,.25,.5,162.);
  assert(!replay.commit(*imported,1000000000,1000000000,measured,speed));
  auto beat=full;beat.piece_start.clear();beat.duration.clear();beat.source_duration.clear();
  beat.source_elapsed.clear();beat.control_count.clear();beat.controls.clear();
  Meta heartbeat{71,2,1130000000,1380000000,0};
  auto ready=TrajectoryQueue::prepare_replay(beat,heartbeat,snapshot,.25,.5,162.);
  count_periodic_memory=true;
  assert(!replay.commit(*ready,1130000000,1130000000,measured,speed));
  assert(replay.sample(1140000000));
  count_periodic_memory=false;
  assert(periodic_allocations==0&&periodic_deletions==0);
  const auto expected=first.sample(.02);
  for(unsigned i=0;i<16;++i){assert(std::abs(replay.q[i]-expected.q[i])<1e-10);
   assert(std::abs(replay.v[i]-expected.v[i])<1e-10);assert(std::abs(replay.a[i]-expected.a[i])<1e-10);}
  assert(replay.id()==2&&replay.deadline()==1380000000);
  ready->id=2;assert(replay.commit(*ready,1140000000,1140000000,measured,speed));
  ready->id=1;ready->source.sequence=3;
  assert(replay.commit(*ready,1380000000,1380000000,measured,speed));
  TargetSnapshot braking;
  assert(replay.snapshot(1150000000,1150000000,2,20000000,measured,braking));
  Wire pause;pause.version=2;pause.segment_id=3;pause.max_velocity=full.max_velocity;
  pause.max_acceleration=full.max_acceleration;pause.max_jerk=full.max_jerk;
  Meta stop{71,3,1150000000,3150000000,2};
  auto stopping=TrajectoryQueue::prepare_target(pause,stop,braking,measured,.25,.5,162.);
  assert(!replay.commit(*stopping,1150000000,1150000000,measured,speed));
  assert(replay.sample(2000000000)&&replay.phase(2000000000)==4);
  assert(replay.commit(*ready,1160000000,1160000000,measured,speed)); // Cannot revive after PAUSE.
  TrajectoryQueue empty;full.version=5;Meta entry{72,1,1000000000,1250000000,1};
  auto empty_ready=TrajectoryQueue::prepare_replay(full,entry,snapshot,.25,.5,162.);
  assert(!empty_ready->inner_reference);
  assert(!empty.commit(*empty_ready,1000000000,1000000000,measured,speed));
 }
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

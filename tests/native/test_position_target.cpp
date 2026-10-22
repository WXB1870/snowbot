// Pure-memory production target-stream tests. No ROS or devices.
#include "position_trajectory.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <new>
thread_local bool periodic=false;
thread_local size_t allocated=0,deleted=0;
void* operator new(std::size_t n){if(periodic)++allocated;if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{if(periodic&&p)++deleted;std::free(p);}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void* p)noexcept{::operator delete(p);}
void operator delete(void* p,std::size_t)noexcept{::operator delete(p);}
void operator delete[](void* p,std::size_t)noexcept{::operator delete(p);}
using namespace position_session;
struct Wire {
 unsigned version=2;uint64_t segment_id=0;int64_t effective_at_ns=0;
 std::vector<double> piece_start,duration,source_duration,source_elapsed,controls;
 std::vector<unsigned char> control_count;
 std::array<double,16> max_velocity{},max_acceleration{},max_jerk{};
 Wire(){max_velocity.fill(.25);max_acceleration.fill(.5);max_jerk.fill(162);}
};
template<class F>void rejects(F run){bool yes=false;try{run();}catch(const std::exception&){yes=true;}assert(yes);}
std::array<double,16> seed(){std::array<double,16> q{};q[1]=.2;q[8]=-.2;q[14]=.12;q[15]=-.08;return q;}
void bounded(const TrajectoryQueue& q){for(unsigned i=0;i<16;++i){assert(q.q[i]>=lower[i]-1e-9&&q.q[i]<=upper[i]+1e-9);assert(std::abs(q.v[i])<=.25+1e-9);assert(std::abs(q.a[i])<=.5+1e-9);assert(std::abs(q.j[i])<=162+1e-7);}}
void stream(int period,bool jitter){
 TrajectoryQueue q;auto measured=seed(),goal=measured;std::array<double,16> speed{};
 int next=0;uint64_t sequence=0;double max_delta_a=0;bool moving=false;
 for(int tick=0;tick<2000;++tick){
  const int64_t now=1000000000LL+tick*1000000LL;
  if(tick==next){
   Meta m{7,++sequence,now,now+200000000,sequence==1?uint8_t(1):uint8_t(0)};Wire w;w.segment_id=sequence;
   const double t=tick*.001;goal=seed();goal[0]+=.004*std::sin(t*3);goal[7]+=.003*std::sin(t*5);goal[14]+=.0015*std::sin(t*4);
   TargetSnapshot snapshot;periodic=true;assert(q.snapshot(now,now,m.operation,10000000,measured,snapshot));periodic=false;
   auto p=TrajectoryQueue::prepare_target(w,m,snapshot,goal);
   const auto first=p->segment->sample(0.);
   for(unsigned i=0;i<16;++i){assert(std::abs(first.q[i]-snapshot.q[i])<1e-12);assert(std::abs(first.v[i]-snapshot.v[i])<1e-12);assert(std::abs(first.a[i]-snapshot.a[i])<1e-11);}
   // Neither unrelated measurement movement nor worker preparation reseeds an update.
   auto disturbed=measured;if(sequence>1)disturbed[0]+=.5;
   periodic=true;assert(!q.commit(*p,now+1000000,now+1000000,disturbed,speed));periodic=false;
   q.reclaim();next+=period+(jitter?(sequence%4==0?4:sequence%4==1?-3:sequence%4==2?2:-3):0);
  }
  const auto before=q.a;periodic=true;assert(q.sample(now));periodic=false;bounded(q);
  if(tick)for(unsigned i=0;i<16;++i){const auto delta=std::abs(q.a[i]-before[i]);max_delta_a=std::max(max_delta_a,delta);assert(delta<=.162+1e-7);}
  moving|=std::abs(q.v[0])>1e-6;
 }
 assert(moving&&max_delta_a>0);
 const int64_t now=3000000000LL;Meta pause{7,++sequence,now,now+200000000,2};Wire w;w.segment_id=sequence;
 TargetSnapshot snapshot;assert(q.snapshot(now,now,2,10000000,measured,snapshot));
 auto stop=TrajectoryQueue::prepare_target(w,pause,snapshot,goal);periodic=true;assert(!q.commit(*stop,now+1000000,now+1000000,measured,speed));periodic=false;q.reclaim();
 bool stopped=false;
 for(int tick=0;tick<200;++tick){periodic=true;assert(q.sample(now+tick*1000000));periodic=false;bounded(q);stopped|=q.phase(now+tick*1000000)==4;if(tick>=10)assert(q.deadline()==pause.expires);}
 assert(stopped);for(unsigned i=0;i<16;++i){assert(std::abs(q.v[i])<1e-10);assert(std::abs(q.a[i])<1e-10);}
 assert(!allocated&&!deleted);
 std::cout<<"PASS target stream "<<1000/period<<"Hz jitter="<<jitter<<": measured-seed isolation, exact C2 seed, sampled jerk/dynamics, timestamped 1ms sampling, PAUSE/TTL, zero periodic new/delete\n";
}
void stale_and_invalid(){
 TrajectoryQueue q;const auto measured=seed();std::array<double,16> speed{};auto goal=measured;goal[0]+=.005;
 const int64_t now=1000000000;Meta m{1,1,now,now+200000000,1};Wire w;w.segment_id=1;TargetSnapshot s;assert(q.snapshot(now,now,1,10000000,measured,s));
 auto first=TrajectoryQueue::prepare_target(w,m,s,goal);assert(!q.commit(*first,now+1000000,now+1000000,measured,speed));assert(q.sample(now+10000000));
 auto wrong=w;wrong.version=1;rejects([&]{TrajectoryQueue::prepare_target(wrong,m,s,goal);});
 wrong=w;wrong.duration.push_back(.1);rejects([&]{TrajectoryQueue::prepare_target(wrong,m,s,goal);});
 wrong=w;wrong.max_acceleration[0]=.5001;rejects([&]{TrajectoryQueue::prepare_target(wrong,m,s,goal);});
 wrong=w;wrong.effective_at_ns=now+10000000;rejects([&]{TrajectoryQueue::prepare_target(wrong,m,s,goal);});
 auto badgoal=goal;badgoal[0]=NAN;rejects([&]{TrajectoryQueue::prepare_target(w,m,s,badgoal);});
 const int64_t update=now+20000000;Meta next{1,2,update,update+200000000,0};w.segment_id=2;assert(q.snapshot(update,update,0,10000000,measured,s));
 auto late=TrajectoryQueue::prepare_target(w,next,s,goal);
 assert(std::string(q.commit(*late,update+12000000,update+12000000,measured,speed))=="target_generation_late");assert(q.sample(update+12000000)&&q.id()==1);
 auto short_ttl=next;short_ttl.expires=update+5000000;rejects([&]{TrajectoryQueue::prepare_target(w,short_ttl,s,goal);});
 auto old=TrajectoryQueue::prepare_target(w,next,s,goal);
 auto pause=next;pause.sequence=3;pause.operation=2;w.segment_id=3;
 auto stop=TrajectoryQueue::prepare_target(w,pause,s,goal);assert(!q.commit(*stop,update+1000000,update+1000000,measured,speed));
 assert(std::string(q.commit(*old,update+1000000,update+1000000,measured,speed))=="target_snapshot_changed");
 assert(q.sample(update+10000000)&&q.id()==3);assert(q.source().operation==2);q.reclaim();
 // A takeover requires rest; ordinary updates above never use this measurement.
 const int64_t restart=update+50000000;Meta take{2,1,restart,restart+200000000,1};w.segment_id=1;assert(q.snapshot(restart,restart,1,10000000,measured,s));
 auto p=TrajectoryQueue::prepare_target(w,take,s,goal);speed[0]=.011;assert(std::string(q.commit(*p,restart+1000000,restart+1000000,measured,speed))=="trajectory_takeover_seed");
 std::cout<<"PASS explicit target version/curve/limit/layout rejection, 12ms late result, original TTL, PAUSE supersedes stale result, takeover rest gate\n";
}
void target_pause_window(){
 using namespace bindu::execution;
 static_assert(maximum_ttl==200000000&&maximum_target_pause_ttl==2000000000LL);
 const int64_t now=1000000000;Meta take{44,1,now,now+200000000,1};assert(fresh(take,now));
 auto over=take;++over.expires;assert(!fresh(over,now,maximum_target_pause_ttl));
 Meta pause{44,2,now+20000000,now+2020000000,2};assert(!fresh(pause,pause.stamp));assert(fresh(pause,pause.stamp,maximum_target_pause_ttl));
 assert(!fresh(pause,pause.expires,maximum_target_pause_ttl));auto too_long=pause;++too_long.expires;assert(!fresh(too_long,too_long.stamp,maximum_target_pause_ttl));
 Gate gate;assert(gate.accepts(take,now,now));gate.commit(take,now,now);
 assert(!gate.accepts(pause,pause.stamp,pause.stamp));assert(gate.accepts(pause,pause.stamp,pause.stamp,maximum_target_pause_ttl));gate.commit(pause,pause.stamp,pause.stamp);
 const auto deadline=gate.deadline();for(unsigned i=0;i<1000;++i){assert(fresh(pause,pause.stamp+i*1000000,maximum_target_pause_ttl));assert(gate.deadline()==deadline);}assert(deadline==pause.expires);
 unsigned feasible=0,impossible=0;double longest=0.;Wire w;w.segment_id=2;
 Limits l;l.lower=lower[0];l.upper=upper[0];l.velocity=.25;l.acceleration=.5;l.jerk=162;
 for(int vi=-20;vi<=20;++vi)for(int ai=-20;ai<=20;++ai){
  const double v=.25*vi/20,a=.5*ai/20;Brake brake;
  if(!AdaptiveStream::brake(l,{0.,v,a,0.},brake)){++impossible;continue;}++feasible;
  TargetSnapshot snapshot;snapshot.q=seed();snapshot.v[0]=v;snapshot.a[0]=a;snapshot.valid=true;snapshot.effective_ros=snapshot.effective_steady=pause.stamp+110000000;snapshot.version=1;
  auto stop=TrajectoryQueue::prepare_target(w,pause,snapshot,snapshot.q);longest=std::max(longest,stop->segment->end());
  assert(stop->segment->end()<(pause.expires-snapshot.effective_ros)*1e-9);
  const auto end=stop->segment->sample(stop->segment->end());for(unsigned i=0;i<16;++i){assert(std::abs(end.v[i])<1e-8&&std::abs(end.a[i])<1e-8);}
 }
 assert(feasible>1000&&impossible>0);
 TargetSnapshot unsafe;unsafe.q=seed();unsafe.q[0]=upper[0]-.0001;unsafe.v[0]=.25;unsafe.valid=true;unsafe.effective_ros=unsafe.effective_steady=pause.stamp+10000000;rejects([&]{TrajectoryQueue::prepare_target(w,pause,unsafe,unsafe.q);});
 std::cout<<"PASS distinct fixed 2s target PAUSE; ordinary/legacy remain 200ms; expired/overlong reject; 16-axis kernel stop grid feasible="<<feasible<<" impossible="<<impossible<<" longest="<<longest<<"s with 110ms source-age/switch deduction; near-limit impossible stop rejected\n";
}

void allowed_limit_seeds(){
 const int64_t now=1000000000;Meta take{909,1,now,now+200000000,1};Wire wire;wire.segment_id=1;
 wire.max_velocity.fill(.03);wire.max_acceleration.fill(.3);
 auto measured=seed();measured[8]=0.00023968449386302382;
 TrajectoryQueue queue;TargetSnapshot snapshot;std::array<double,16> velocity{};
 assert(queue.snapshot(now,now,1,10000000,measured,snapshot));
 auto prepared=TrajectoryQueue::prepare_target(wire,take,snapshot,measured);
 assert(prepared->q[8]==measured[8]);
 assert(!queue.commit(*prepared,now+1000000,now+1000000,measured,velocity));
 assert(queue.sample(now+10000000)&&std::abs(queue.q[8]-measured[8])<1e-12);
 for(unsigned i=0;i<joint_count;++i){
  auto boundary=measured;boundary[i]=upper[i];TargetSnapshot point;
  assert(queue.snapshot(now,now,1,10000000,boundary,point));
  auto at_upper=TrajectoryQueue::prepare_target(wire,take,point,boundary);assert(at_upper->q[i]==boundary[i]);
  boundary[i]=lower[i];assert(queue.snapshot(now,now,1,10000000,boundary,point));
  auto at_lower=TrajectoryQueue::prepare_target(wire,take,point,boundary);assert(at_lower->q[i]==boundary[i]);
  auto outside=boundary;outside[i]=lower[i]-1e-8;
  assert(queue.snapshot(now,now,1,10000000,outside,point));
  rejects([&]{TrajectoryQueue::prepare_target(wire,take,point,outside);});
  outside[i]=upper[i]+1e-8;assert(queue.snapshot(now,now,1,10000000,outside,point));
  rejects([&]{TrajectoryQueue::prepare_target(wire,take,point,outside);});
 }
 std::cout<<"PASS all 16 expanded boundary seeds; beyond 0.1 degree rejected; actual R2 measurement retained; 0.03/0.3 limits\n";
}
int main(){allowed_limit_seeds();stream(20,false);stream(10,false);stream(20,true);stream(10,true);stale_and_invalid();target_pause_window();}

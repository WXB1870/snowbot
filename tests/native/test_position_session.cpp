#include "servo_control/position_watchdog.h"
#include "servo_control/position_follower.h"
#include "servo_control/position_clock.h"
#include "position_session_contract.h"
#include <cassert>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>
using namespace position_session;
#ifdef POSITION_PDO_OUTPUT_TEST
void check_position_pdo_output();
#endif
// One continuous watchdog crosses TTL; healthy frames, PAUSE and deselection
// cannot release a prior fault. Only a newly admitted explicit takeover can.
void check_latched_failures(){
 Observation obs;obs.sample=1;obs.valid_mask=0xfffff;obs.wkc=obs.expected_wkc=67;obs.status.fill(0x27);obs.position.fill(.2);
 for(auto cause:{FEEDBACK_INVALID,DRIVE_FAULT,UNCONFIRMED_POLICY,CONTROL_SOURCE_CONFLICT,CYCLE_OR_TRACKING_ERROR}){
  Watchdog guard;Output cmd;cmd.selected=true;cmd.policy=2;cmd.operation=1;cmd.admission_generation=10;
  cmd.session=20;cmd.sequence=1;cmd.deadline=1200000000;cmd.trajectory=true;cmd.rt_deadline=1020000000;
  obs.sampled_ns=1000000000;obs.wkc=67;obs.status.fill(0x27);guard.evaluate(cmd,obs,1000000000);assert(!guard.disabled);
  if(cause==FEEDBACK_INVALID)obs.wkc=61;
  else if(cause==DRIVE_FAULT)obs.status[0]|=8;
  else if(cause==UNCONFIRMED_POLICY)cmd.policy=0;
  else if(cause==CONTROL_SOURCE_CONFLICT)cmd.failure_reason=cause;
  // No fresh sample: genuine RT sampler fault occurs before the source TTL.
  const auto fault_time=cause==CYCLE_OR_TRACKING_ERROR?1020000000:1001000000;
  obs.sampled_ns=fault_time;guard.evaluate(cmd,obs,fault_time);assert(guard.disabled&&guard.reason==cause);
  obs.wkc=67;obs.status.fill(0x27);cmd.policy=2;cmd.failure_reason=0;
  for(int64_t t=fault_time+1000000;t<=1260000000;t+=1000000){
   obs.sampled_ns=obs.attempted_ns=t;cmd.rt_deadline=t+20000000;
   cmd.operation=(t%3000000==0)?2:1;cmd.trajectory_phase=cmd.operation==2?4:2;
   guard.evaluate(cmd,obs,t);guard.report_hold(obs,controlled_mask);
   assert(guard.disabled&&guard.reason==cause&&!obs.hold_valid&&!guard.motor_enabled(true,true,0));
  }
  cmd.selected=false;guard.evaluate(cmd,obs,1260000000);assert(guard.disabled&&guard.output.selected);
  cmd.selected=true;cmd.operation=1;cmd.deadline=1460000000;cmd.rt_deadline=1280000000;
  guard.evaluate(cmd,obs,1260000000);assert(guard.disabled); // Replay old takeover token.
  ++cmd.sequence;++cmd.admission_generation;
  guard.evaluate(cmd,obs,1260000000);assert(!guard.disabled&&guard.reason==OK&&!guard.captured);
 }
 std::cout<<"PASS continuous latched sampler/feedback/drive/policy/conflict failures across TTL, replay/deselection rejection and new takeover recovery\n";
}
int main(){
#ifdef POSITION_PDO_OUTPUT_TEST
 check_position_pdo_output();
#endif
 ObservationCache cache;Observation o;o.valid_mask=0xfffff;o.wkc=o.expected_wkc=67;o.sample=1;o.sampled_ns=1000000000;
 o.position.fill(.2);o.status.fill(0x27);cache.publish(o);assert(cache.read().sample==1);
 Gate gate;Meta m{1,1,1000000000,1200000000,1};assert(gate.accepts(m,1000000000,1000000000));gate.commit(m,1000000000,1000000000);
 assert(!gate.accepts(m,1000000000,1000000000));m.sequence=2;m.operation=0;assert(gate.accepts(m,1010000000,1010000000));gate.commit(m,1010000000,1010000000);
 const auto deadline=gate.deadline();m.sequence=1;assert(!gate.accepts(m,1010000000,1010000000));assert(gate.deadline()==deadline);
 m.sequence=3;assert(!gate.accepts(m,1200000000,1200000000));m.stamp=1210000000;m.expires=1300000000;assert(!gate.accepts(m,1210000000,1210000000));
 m.operation=1;assert(gate.accepts(m,1210000000,1210000000));gate.commit(m,1210000000,1210000000);m.operation=2;++m.sequence;assert(gate.accepts(m,1220000000,1220000000));gate.commit(m,1220000000,1220000000);
 Meta restart{2,1,1230000000,1400000000,1};assert(gate.accepts(restart,1230000000,1230000000));gate.commit(restart,1230000000,1230000000);m.operation=1;m.stamp=1410000000;m.expires=1500000000;assert(!gate.accepts(m,1410000000,1410000000));
 gate.interrupt();Meta old_session=restart;old_session.sequence=2;old_session.stamp=1410000000;old_session.expires=1500000000;
 assert(!gate.accepts(old_session,1410000000,1410000000));Meta new_session{3,1,1410000000,1500000000,1};assert(gate.accepts(new_session,1410000000,1410000000));
 Follower f;std::array<double,joint_count> q{};q[0]=.4;q[1]=.2;q[8]=-.2;f.seed(q);q[0]=.8;assert(f.set(q));assert(f.step(.001));assert(f.q[0]==.4);double previous=f.q[0],velocity=0;
 for(int i=0;i<4000;++i){assert(f.step(.001));assert(std::abs(f.v[0])<=f.vmax+1e-9);assert(std::abs(f.v[0]-velocity)<=f.amax*.001+1e-9);assert(std::abs(f.q[0]-previous)<=f.vmax*.001+1e-9);previous=f.q[0];velocity=f.v[0];if(i==100){q[0]=.6;f.set(q);assert(f.q[0]==previous);}}
 // Nonzero waist seeds and continuous retargets retain the same state.
 Follower coupled;std::array<double,joint_count> seed{};seed[14]=.12;seed[15]=-.08;coupled.seed(seed);
 auto goal=seed;goal[0]+=.01;goal[7]+=.10;goal[14]+=.04;assert(coupled.set(goal));
 assert(coupled.step(.001));
 for(unsigned cycle=0;cycle<200;++cycle){assert(coupled.step(.001));
  assert(std::abs((coupled.q[0]-seed[0])/.01-(coupled.q[7]-seed[7])/.10)<1e-10);
  assert(std::abs((coupled.q[14]-seed[14])/.04-(coupled.q[7]-seed[7])/.10)<1e-10);
 }
 for(unsigned cycle=0;cycle<3000;++cycle){auto before=coupled.q;auto speed=coupled.v;
  if(cycle%200==0){goal=seed;goal[0]+=(cycle%400?-.3:.3);goal[7]+=(cycle%400?.1:-.1);goal[15]+=.1;assert(coupled.set(goal));assert(coupled.v==speed&&coupled.q==before);}
  assert(coupled.step(.001));for(unsigned i=0;i<joint_count;++i){assert(std::abs(coupled.v[i])<=coupled.vmax+1e-10);assert(std::abs(coupled.v[i]-speed[i])<=coupled.amax*.001+1e-10);assert(std::abs(coupled.q[i]-before[i])<=coupled.vmax*.001+1e-10);}
 }
 Output selected;selected.selected=true;selected.policy=2;selected.session=90;selected.sequence=1;selected.operation=1;selected.deadline=1200000000;
 Watchdog isolation;isolation.evaluate(selected,o,1000000000);
 for(unsigned i=0;i<20;++i){assert(isolation.motor_enabled(false,true,i)==bool(controlled_mask&(1u<<i)));assert(!isolation.motor_enabled(false,false,i));}
 assert(position_slot(14)==18&&position_slot(15)==19&&controlled_mask==0xc7f7f);
 assert(std::abs(csp_radians(csp_counts(.1234))-.1234)<=.5/csp_counts_per_rad);
 assert(!f.step(.1));Interval clock;clock.reset(1000000000);assert(std::abs(clock.tick(1001000000)-.001)<1e-12);assert(clock.tick(1001000000)<0);
 Output p;p.selected=true;p.policy=2;p.operation=1;p.session=1;p.sequence=1;p.admission_generation=1;p.deadline=1100000000;p.q.fill(.3);Watchdog w;w.evaluate(p,o,1000000000);assert(!w.disabled&&w.reason==OK);
 o.sampled_ns=1100000000;w.evaluate(p,o,1100000000);assert(w.reason==EXPIRED&&!w.disabled&&w.output.q[0]==.2f);
 o.position[0]=.6f;o.sampled_ns=1101000000;w.evaluate(p,o,1101000000);assert(w.output.q[0]==.2f); // Capture once, not drifting feedback.
 o.wkc=61;w.evaluate(p,o,1102000000);assert(w.disabled&&w.reason==FEEDBACK_INVALID);
 o.wkc=67;o.status[0]|=8;w.evaluate(p,o,1103000000);assert(w.disabled&&w.reason==FEEDBACK_INVALID); // First failure stays latched.
 o.status[0]=0x27;o.sampled_ns=1104000000;++p.sequence;++p.admission_generation;p.deadline=1300000000;w.evaluate(p,o,1104000000);assert(w.reason==OK&&!w.disabled);
 // Fresh ROS output copies cannot extend the original RT deadline.
 o.sampled_ns=1300000000;w.evaluate(p,o,1300000000);assert(w.reason==EXPIRED);
 p.policy=1;w.evaluate(p,o,1300000000);assert(w.disabled);
 p.selected=false;w.evaluate(p,o,1300000000);assert(w.disabled&&w.reason==EXPIRED);
 // Motion continued during transport: servo PAUSE captures .41, while the
 // sender had measured .40. The acknowledged point is the fixed RT request.
 p.selected=true;p.policy=2;p.operation=1;p.session=4;p.sequence=1;++p.admission_generation;p.deadline=1600000000;
 o.sampled_ns=1400000000;o.sample=20;w.evaluate(p,o,1400000000);
 p.operation=2;++p.sequence;p.q.fill(.41f);o.position.fill(.42f);
 w.evaluate(p,o,1401000000);assert(w.reason==PAUSED&&!w.disabled&&w.captured);
 assert(w.hold[0]==.41f&&w.hold_sample==20&&w.output.q[0]==.41f);
 o.attempted_ns=1401000000;w.report_hold(o,controlled_mask);assert(o.hold_valid&&o.hold_position[0]==.41f);
 w.report_hold(o,controlled_mask&~1u);assert(!o.hold_valid);
 o.status[0]=0x23;w.report_hold(o,controlled_mask);assert(!o.hold_valid);
 o.status[0]=0x27;w.report_hold(o,controlled_mask);assert(o.hold_valid);
 p.q.fill(.43f);o.position.fill(.44f);o.sample=21;o.sampled_ns=1600000000;
 w.evaluate(p,o,1600000000);assert(w.reason==EXPIRED&&!w.disabled&&w.output.q[0]==.41f&&w.hold_sample==20);
 // Declared measured_hold does not turn invalid feedback/Fault/conflict or
 // cycle errors into a healthy hold. Keep the configured policy unchanged.
 for(auto failure:{CONTROL_SOURCE_CONFLICT,CYCLE_OR_TRACKING_ERROR}){
  p.operation=1;++p.sequence;++p.admission_generation;p.deadline=1800000000;p.failure_reason=0;
  o.sampled_ns=1601000000;w.evaluate(p,o,1601000000);assert(!w.disabled);
  p.failure_reason=failure;
  o.sampled_ns=1601000000;w.evaluate(p,o,1601000000);
  assert(w.disabled&&w.reason==failure&&p.policy==2);
  w.report_hold(o,controlled_mask);assert(!o.hold_valid);
 }
 p.failure_reason=0;p.operation=1;++p.sequence;++p.admission_generation;w.evaluate(p,o,1601000000);
 assert(w.reason==OK&&!w.disabled&&!w.captured);
 p.operation=2;++p.sequence;p.policy=1;p.deadline=1602000000;
 w.evaluate(p,o,1601000000);assert(w.reason==PAUSED&&!w.disabled);
 o.sampled_ns=1602000000;w.evaluate(p,o,1602000000);assert(w.disabled&&w.reason==EXPIRED);
 p.policy=0;w.evaluate(p,o,1602000000);assert(w.disabled&&w.reason==EXPIRED);
 check_latched_failures();
 std::atomic<bool> done{false};std::thread writer([&]{Observation s;for(uint64_t i=1;i<=200000;++i){s.sample=i;s.sampled_ns=i;s.position.fill(float(i));s.speed.fill(-float(i));cache.publish(s);}done=true;});
 unsigned coherent=0;while(!done.load()||coherent==0){auto s=cache.read();if(s.sample<=1)continue;++coherent;assert(s.sampled_ns==int64_t(s.sample));for(int j=0;j<20;++j){assert(s.position[j]==float(s.sample));assert(s.speed[j]==-float(s.sample));}}writer.join();
 std::vector<long long> times;times.reserve(20000);for(int i=0;i<20000;++i){auto t=steady_ns();cache.publish(o);times.push_back(steady_ns()-t);}std::sort(times.begin(),times.end());
 std::cout<<"PASS gate, source deadline, old sessions, takeover, continuous targets, pause, monotonic clock, RT freeze/WKC/Fault, resource mask and coherent SPSC; publish ns median="<<times[10000]<<" p99="<<times[19800]<<" max="<<times.back()<<"\n";
}

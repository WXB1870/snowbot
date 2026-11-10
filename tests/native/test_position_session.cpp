#include "servo_control/position_watchdog.h"
#include "servo_control/position_follower.h"
#include "servo_control/position_clock.h"
#include "position_session_contract.h"
#include <cassert>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
using namespace position_session;
#ifdef POSITION_PDO_OUTPUT_TEST
void check_position_pdo_output();
#endif
// One continuous watchdog crosses TTL; healthy frames, PAUSE and deselection
// cannot release a prior fault. Only a newly admitted explicit takeover can.
// The position band is finite: R3 lower 1 degree, all other bounds 0.1 degree.
void check_joint_limit_allowance(){
 assert(std::abs(joint_limit_allowance-std::acos(-1.)/1800.)<1e-18);
 std::array<double,joint_count> q{};
 for(unsigned j=0;j<joint_count;++j)q[j]=(nominal_lower[j]+nominal_upper[j])*.5;
 for(unsigned i=0;i<joint_count;++i){
  const double saved=q[i],lower_allowance=i==9?std::acos(-1.)/180.:joint_limit_allowance;
  assert(std::abs(lower[i]-(nominal_lower[i]-lower_allowance))<1e-15);
  assert(std::abs(upper[i]-(nominal_upper[i]+joint_limit_allowance))<1e-15);
  q[i]=nominal_lower[i]-lower_allowance;assert(in_limits(q));
  q[i]-=1e-8;assert(!in_limits(q));
  q[i]=nominal_upper[i]+joint_limit_allowance;assert(in_limits(q));
  q[i]+=1e-8;assert(!in_limits(q));
  q[i]=std::numeric_limits<double>::quiet_NaN();assert(!in_limits(q));q[i]=saved;
 }
 q[8]=0.00023968449386302382;assert(in_limits(q));
 q[9]=-0.5822895169258118;assert(in_limits(q)); // Recorded startup rejection.
 q[9]=nominal_lower[9]-std::acos(-1.)/180.-1e-8;assert(!in_limits(q));
}
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

// Ordinary target expiry under measured_hold captures once, without upgrading
// EXPIRED into a successful PAUSE; the local PDO/drive/RT protections remain.
void check_target_expiry_hold(){
 for(auto policy:{1,2}){
  Observation o;o.sample=1;o.valid_mask=0xfffff;o.wkc=o.expected_wkc=67;
  o.status.fill(0x27);o.position.fill(.12f);o.sampled_ns=o.attempted_ns=1000000000;
  Output p;p.selected=true;p.policy=policy;p.operation=1;p.admission_generation=1;
  p.session=7;p.sequence=1;p.deadline=1200000000;p.trajectory=true;p.reference_mode=2;
  p.rt_deadline=1210000000;p.q.fill(.2f);
  Watchdog w;w.evaluate(p,o,1000000000);assert(!w.disabled);
  for(int64_t t=1200000000;t<=3200000000;t+=10000000){
   ++o.sample;o.sampled_ns=o.attempted_ns=t;
   w.evaluate(p,o,t);assert(w.reason==EXPIRED);
   assert(w.disabled==(policy==1));
   if(policy==2){assert(w.output.q[0]==.12f&&w.output.v[0]==0);o.position[0]=.125f;}
  }
  if(policy==2){o.status[0]|=8;w.evaluate(p,o,3200000000);assert(w.disabled&&w.reason==DRIVE_FAULT);}
 }
 std::cout<<"PASS target expiry: fixed measured hold versus disable; no fake PAUSE success; drive fault retained\n";
}

namespace {
constexpr int64_t tracker_ros=10000000000LL,tracker_steady=1000000000LL,ms=1000000;
struct TrackerObservation {
 struct {struct {int32_t sec=0;uint32_t nanosec=0;} stamp;} header;
 uint64_t boot_id=1,pdo_sample=1;double pdo_age=0;
 int wkc=67,expected_wkc=67;uint32_t valid_mask=0xfffff;
 std::array<uint16_t,20> status{};std::array<double,20> position{},speed{};
};
void receive_tracker(Tracker& t,TrackerObservation& m,uint64_t sequence,int64_t ros,int64_t steady,int64_t delay=0){
 m.pdo_sample=sequence;const auto source=ros-delay;
 m.header.stamp.sec=int32_t(source/1000000000);m.header.stamp.nanosec=uint32_t(source%1000000000);
 t.observe(m,ros,steady);
}
Tracker ready_tracker(){
 Tracker t;TrackerObservation m;
 receive_tracker(t,m,1,tracker_ros-ms,tracker_steady-ms);receive_tracker(t,m,2,tracker_ros,tracker_steady);
 assert(t.fresh(tracker_ros,tracker_steady));return t;
}
void check_ros_feedback_budget(){
 static_assert(ros_feedback_timeout_ns==100*ms);
 for(const auto delay:{21*ms,41*ms,53*ms,92*ms,98*ms,100*ms,101*ms}){
  const bool expected=delay<=100*ms;TrackerObservation m;Tracker receipt,progress;
  receive_tracker(receipt,m,1,tracker_ros-ms,tracker_steady-ms,delay);
  receive_tracker(receipt,m,2,tracker_ros,tracker_steady,delay);
  assert((receipt.advances==2)==expected && receipt.fresh(tracker_ros,tracker_steady)==expected);
  receive_tracker(progress,m,1,tracker_ros,tracker_steady);
  receive_tracker(progress,m,2,tracker_ros+delay,tracker_steady+delay);
  assert((progress.advances==2)==expected && progress.fresh(tracker_ros+delay,tracker_steady+delay)==expected);
  auto no_progress=ready_tracker();no_progress.source=tracker_ros+delay;
  assert(no_progress.fresh(tracker_ros+delay,tracker_steady+delay)==expected);
  auto combined=ready_tracker();combined.age=.001;combined.source=tracker_ros-(delay-ms);
  assert(combined.fresh(tracker_ros,tracker_steady)==expected);
 }
 for(const double age:{.02,.020000001,.021,-.001,std::numeric_limits<double>::quiet_NaN()}){
  Tracker t;TrackerObservation m;m.pdo_age=age;
  receive_tracker(t,m,1,tracker_ros-ms,tracker_steady-ms);receive_tracker(t,m,2,tracker_ros,tracker_steady);
  assert(t.fresh(tracker_ros,tracker_steady)==(age==.02));
 }
 for(unsigned failure=0;failure<9;++failure){
  auto t=ready_tracker();TrackerObservation m;uint64_t sequence=3;int64_t delay=0;
  if(failure==0)m.wkc=66;
  if(failure==1)m.expected_wkc=m.wkc=0;
  if(failure==2)m.status[0]=8;
  if(failure==3)m.valid_mask&=~1u;
  if(failure==4)m.position[0]=std::numeric_limits<double>::quiet_NaN();
  if(failure==5)m.speed[18]=std::numeric_limits<double>::infinity();
  if(failure==6)sequence=1;
  if(failure==7)sequence=0;
  if(failure==8)delay=-ms;
  receive_tracker(t,m,sequence,tracker_ros+ms,tracker_steady+ms,delay);
  assert(!t.fresh(tracker_ros+ms,tracker_steady+ms));
 }
 auto t=ready_tracker();assert(!t.fresh(tracker_ros-1,tracker_steady) && !t.fresh(tracker_ros,tracker_steady-1));
 TrackerObservation m;receive_tracker(t,m,2,tracker_ros+101*ms,tracker_steady+101*ms);
 assert(!t.fresh(tracker_ros+101*ms,tracker_steady+101*ms)); // Duplicate PDO cannot renew progress.
 Meta command{1,1,tracker_ros,tracker_ros+400*ms,1};assert(fresh(command,tracker_ros));
 assert(!fresh(command,tracker_ros+400*ms));
 command.expires=command.stamp+maximum_target_ttl+1;assert(!fresh(command,tracker_ros));
 std::cout<<"PASS ROS feedback 100ms boundaries; raw PDO 20ms, WKC/Fault/validity/sequence/clocks and source deadline preserved\n";
}
} // namespace

#ifdef POSITION_HOLD_RECEIVE_TEST
void check_hold_receive_window(){
 constexpr int64_t ms=1000000;
 for(const std::string scenario:{"recover_one","recover_two","third","partial","drive_fault","lost_enable",
                                 "sensor","drift","speed","shutdown","new_motion","following","unconfirmed","rt_expired"}){
  Watchdog w;ObservationCache cache;Observation o;Output p;
  int64_t now=1000000000;
  p.selected=p.trajectory=p.impedance=p.impedance_mode=true;
  p.policy=2;p.operation=2;p.trajectory_phase=4;p.admission_generation=3;p.session=9;p.sequence=2;
  p.nominal_hold=scenario!="following";p.deadline=now+200*ms;p.rt_deadline=now+20*ms;
  if(scenario=="following"){p.operation=0;p.trajectory_phase=2;}
  p.hold_position_tolerance=.25;p.hold_velocity_tolerance=.01;
  p.q.fill(.17f);p.kp.fill(1);p.kd.fill(.2f);p.torque.fill(2);
  auto receive=[&](int wkc){
   ++o.attempt;o.attempted_ns=now;o.wkc=wkc;o.expected_wkc=67;
   o.valid_mask=0;o.sensor_valid_mask=0;
   if(wkc==67){++o.sample;o.sampled_ns=now;o.valid_mask=0xfffff;
    o.sensor_valid_mask=3;o.sensor_sampled_ns.fill(now);}
   w.report_hold(o,controlled_mask);o.reference_valid=o.hold_valid;
   cache.publish(o,w.finish_hold_receive(o));
  };
  o.status.fill(0x27);receive(67);w.evaluate(p,o,now);receive(67);
  const auto last=cache.read();const auto before=w.output;
  if(scenario=="unconfirmed"){o.hold_valid=false;w.finish_hold_receive(o);}
  ++p.sample_time_ns;p.q.fill(.3f);p.torque.fill(7);p.kp.fill(4);p.kd.fill(.5f);p.v.fill(.02f);
  now+=ms;receive(scenario=="partial"?40:-1);
  if(scenario=="shutdown"){p.shutdown=true;p.shutdown_deadline=now+100*ms;}
  if(scenario=="new_motion"){p.operation=1;++p.admission_generation;++p.sequence;p.nominal_hold=false;}
  if(scenario=="rt_expired")now=p.rt_deadline;
  w.evaluate(p,o,now);
  if(scenario=="partial" || scenario=="shutdown" || scenario=="new_motion" ||
     scenario=="following" || scenario=="unconfirmed" || scenario=="rt_expired"){
   assert(w.disabled);continue;
  }
  auto frozen=[&]{
   auto retained=cache.read();
   assert(retained.awaiting_hold_rx && retained.sample==last.sample && retained.sampled_ns==last.sampled_ns);
   assert(o.wkc==-1 && o.valid_mask==0 && !o.hold_valid);
   assert(!w.disabled && w.reason==PAUSED && w.output.q==before.q && w.output.v==before.v &&
          w.output.kp==before.kp && w.output.kd==before.kd && w.output.torque==before.torque &&
          w.output.sample_time_ns==before.sample_time_ns && w.output.rt_deadline==before.rt_deadline);
  };
  frozen();
  if(scenario!="recover_one"){now+=ms;receive(-1);w.evaluate(p,o,now);frozen();}
  now+=ms;
  if(scenario=="drive_fault")o.status[0]|=8;
  if(scenario=="lost_enable")o.status[0]=0x23;
  if(scenario=="drift")o.position[0]=.26f;
  if(scenario=="speed")o.speed[0]=.02f;
  if(scenario=="sensor")o.wrench[0]=std::numeric_limits<float>::quiet_NaN();
  receive(scenario=="third"?-1:67);w.evaluate(p,o,now);
  assert(!cache.read().awaiting_hold_rx);
  if(scenario=="recover_one" || scenario=="recover_two"){
   assert(!w.disabled && w.reason==PAUSED && o.sample==last.sample+1);
   now+=ms;receive(-1);w.evaluate(p,o,now);assert(!w.disabled); // new burst
  }else{
   assert(w.disabled);if(scenario=="drive_fault")assert(w.reason==DRIVE_FAULT);now+=ms;receive(67);w.evaluate(p,o,now);assert(w.disabled);
  }
 }
 std::cout<<"PASS confirmed hold: one/two missing frames freeze output, third latches, independent faults remain active\n";
}
#endif
int main(){
#ifdef POSITION_HOLD_RECEIVE_TEST
 check_hold_receive_window();
#endif
 check_ros_feedback_budget();
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
 Output selected;selected.selected=true;selected.policy=2;selected.session=90;selected.sequence=1;selected.operation=1;selected.deadline=1200000000;
 Watchdog isolation;isolation.evaluate(selected,o,1000000000);
 for(unsigned i=0;i<20;++i){assert(isolation.motor_enabled(false,true,i)==bool(controlled_mask&(1u<<i)));assert(!isolation.motor_enabled(false,false,i));}
 assert(position_slot(14)==18&&position_slot(15)==19&&controlled_mask==0xc7f7f);
 // Independent full-revolution expectations catch a shared wrong encoder scale.
 assert(csp_counts(6.28318530717958647692)==262144);
 assert(csp_speed_counts(6.28318530717958647692)==65536);
 assert(std::abs(csp_radians(262144)-6.28318530717958647692)<1e-12);
 assert(std::abs(csp_radians(csp_counts(.1234))-.1234)<=.5/csp_counts_per_rad);
 Interval clock;clock.reset(1000000000);assert(std::abs(clock.tick(1001000000)-.001)<1e-12);assert(clock.tick(1001000000)<0);
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
 check_target_expiry_hold();
 check_joint_limit_allowance();
 check_latched_failures();
 std::atomic<bool> done{false};std::thread writer([&]{Observation s;for(uint64_t i=1;i<=200000;++i){s.sample=i;s.sampled_ns=i;s.position.fill(float(i));s.speed.fill(-float(i));cache.publish(s);}done=true;});
 unsigned coherent=0;while(!done.load()||coherent==0){auto s=cache.read();if(s.sample<=1)continue;++coherent;assert(s.sampled_ns==int64_t(s.sample));for(int j=0;j<20;++j){assert(s.position[j]==float(s.sample));assert(s.speed[j]==-float(s.sample));}}writer.join();
 std::vector<long long> times;times.reserve(20000);for(int i=0;i<20000;++i){auto t=steady_ns();cache.publish(o);times.push_back(steady_ns()-t);}std::sort(times.begin(),times.end());
 std::cout<<"PASS gate, source deadline, old sessions, takeover, continuous targets, pause, monotonic clock, RT freeze/WKC/Fault, resource mask and coherent SPSC; publish ns median="<<times[10000]<<" p99="<<times[19800]<<" max="<<times.back()<<"\n";
}

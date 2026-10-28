// Actual submit/dispatch/poll/step, OutputCache and Watchdog; clocks and feedback
// are injected memory fixtures. This does not measure ROS or EtherCAT scheduling.
struct CycleHarness {
 ServoNode node;position_session::OutputCache cache;position_session::Watchdog watchdog;
 int64_t next_cycle=1000000000LL;uint64_t sequence=0,session=801;
 std::array<double,16> measured{};
 CycleHarness(){
  using namespace position_session;cycle_ros_offset_ns=0;cycle_now_ns=next_cycle;active_cycle_cache=&cache;
  snapshot=Observation{};snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);
  snapshot.position[0]=.015;snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;
  for(unsigned i=0;i<16;++i)measured[i]=snapshot.position[position_slot(i)];
  node.position_reference_mode_=2;node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_,[](auto& result){
   inspect_entered=true;while(inspect_hold.load())std::this_thread::yield();
   if(auto error=inspect_servo_command(*result.packet,result.origin.get(),result.context,result.command))throw std::invalid_argument(error);
  });node.feedback();
 }
 Packet::SharedPtr command(uint8_t operation=0,double displacement=.1){
  node.feedback();auto p=std::make_shared<Packet>();const auto now=node.now().nanoseconds();
  p->meta.session=session;p->meta.sequence=++sequence;p->meta.operation=operation;p->meta.reference_mode=2;
  p->meta.source_stamp_ns=now;p->meta.expires_at_ns=now+(operation==2?2000000000LL:200000000LL);
  p->meta.feedback_boot_id=node.position_feedback_.boot;p->meta.feedback_sample=node.position_feedback_.sample;p->meta.middle_instance_id=32;
  for(unsigned i=0;i<7;++i){p->command.l_position[i]=measured[i];p->command.r_position[i]=measured[i+7];p->command.l_kp[i]=p->command.r_kp[i]=40;p->command.l_kd[i]=p->command.r_kd[i]=2;}
  p->command.l_position[0]+=displacement;p->waist_position={measured[14],measured[15]};
  p->trajectory.version=2;p->trajectory.segment_id=p->meta.sequence;p->trajectory.max_velocity.fill(.25);p->trajectory.max_acceleration.fill(.5);p->trajectory.max_jerk.fill(162);return p;
 }
 position_session::Output tick(){
  next_cycle+=1000000;cycle_now_ns=next_cycle;node.feedback();
  periodic_allocations=periodic_deletions=0;count_periodic_memory=true;
  node.position_poll();node.position_step();auto packet=cache.read();
  watchdog.evaluate(packet,snapshot,cycle_clock());snapshot.attempted_ns=cycle_clock();watchdog.report_hold(snapshot,position_session::controlled_mask);
  count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);
  // Feedback is an independent memory device input, never rewritten to target/reference.
  for(unsigned i=0;i<16;++i)assert(snapshot.position[position_session::position_slot(i)]==measured[i]);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));return packet;
 }
 void settle(){const auto limit=next_cycle+150000000;do{tick();assert(next_cycle<limit);}while(node.position_admission_->busy()||node.position_target_pending_);}
 void interrupt(){node.position_interrupt();}
};
void check_cycle_stream(int period,uint8_t policy,int64_t ros_offset=0){
 using namespace position_session;CycleHarness h;h.node.position_policy_=policy;auto take=h.command(1);h.node.position_submit(take);h.settle();assert(h.node.position_gate_.last().sequence==1);
 bool moved=false;double largest_speed=0;
 for(int tick=0;tick<400;++tick){
  if(tick%period==0){auto target=h.command(0);h.node.position_submit(target);}
  const auto packet=h.tick();assert(!h.watchdog.disabled);moved|=std::abs(packet.reference_position[0]-h.measured[0])>1e-5;largest_speed=std::max(largest_speed,std::abs(packet.reference_speed[0]));
 }
 assert(moved&&largest_speed>.03);cycle_ros_offset_ns=ros_offset;auto pause=h.command(2);h.node.position_submit(pause);
 const auto expiry=pause->meta.expires_at_ns;bool braking=false,held=false;std::array<double,16> fixed{};bool have_fixed=false;int64_t stop_deadline=0;
 for(int tick=0;tick<2200;++tick){
  const auto packet=h.tick();assert(!h.watchdog.disabled);
  if(packet.sequence==pause->meta.sequence){assert(packet.source_stamp_ns==pause->meta.source_stamp_ns&&packet.expires_at_ns==expiry);if(!stop_deadline)stop_deadline=packet.deadline;assert(packet.deadline==stop_deadline);braking|=packet.trajectory_phase==3;
   if(packet.trajectory_phase==4){if(ros_offset)assert(h.node.position_gate_.deadline()!=packet.deadline);held=true;if(!have_fixed){fixed=packet.reference_position;have_fixed=true;}assert(packet.reference_position==fixed);}
  }
  if(cycle_now_ns<expiry&&held)assert(h.watchdog.reason==PAUSED);
 }
 assert(braking&&held&&h.watchdog.reason==PAUSED&&h.watchdog.captured);
 assert(h.node.position_local_hold_&&h.watchdog.nominal_pause&&h.watchdog.output.nominal_hold);
 auto expired_copy=pause;h.node.position_submit(expired_copy);h.tick();
 assert(!h.node.position_cycle_reason_&&h.watchdog.reason==PAUSED);
 assert(snapshot.hold_valid);assert(std::abs(h.watchdog.hold[0]-h.measured[0])>1e-5); // Reference hold acknowledgement is not physical arrival.
 for(auto cause:{FEEDBACK_INVALID,DRIVE_FAULT,CONTROL_SOURCE_CONFLICT,CYCLE_OR_TRACKING_ERROR}){
  auto guard=h.watchdog;auto input=guard.output;auto feedback=snapshot;auto now=cycle_clock();
  feedback.sampled_ns=feedback.attempted_ns=now;input.rt_deadline=now+20000000;
  if(cause==FEEDBACK_INVALID)feedback.wkc=61;
  else if(cause==DRIVE_FAULT)feedback.status[0]|=8;
  else if(cause==CONTROL_SOURCE_CONFLICT)input.failure_reason=cause;
  else input.rt_deadline=now;
  guard.evaluate(input,feedback,now);guard.report_hold(feedback,controlled_mask);
  assert(guard.disabled&&guard.reason==cause&&!feedback.hold_valid);
  feedback=snapshot;feedback.sampled_ns=feedback.attempted_ns=now+1000000;input=h.watchdog.output;input.rt_deadline=now+20000000;
  guard.evaluate(input,feedback,now+1000000);assert(guard.disabled&&guard.reason==cause);
 }
 h.interrupt();for(unsigned i=0;i<25;++i)h.tick();
 assert(h.watchdog.disabled&&h.watchdog.reason==CONTROL_SOURCE_CONFLICT);
 ++h.session;h.sequence=0;auto retake=h.command(1,0.);h.node.position_submit(retake);h.settle();
 assert(h.node.position_gate_.last().session==h.session&&!h.watchdog.disabled&&!h.watchdog.nominal_pause);
 std::cout<<"PASS actual position step/cache/watchdog "<<1000/period<<"Hz clock offset "<<ros_offset<<"ns: coupled nominal output, fixed 2s PAUSE source, local powered hold after expiry, independent measured feedback, zero counted periodic new/delete\n";
}
void check_cycle_expiry_fault_revoke(){
 using namespace position_session;
 for(uint8_t policy:{1,2}){CycleHarness h;h.node.position_policy_=policy;h.node.position_submit(h.command(1));h.settle();
  for(unsigned i=0;i<40;++i)h.tick();
  const auto deadline=h.node.position_gate_.deadline();
  while(cycle_now_ns<deadline+30000000)h.tick();
  assert(h.watchdog.reason==EXPIRED&&h.watchdog.disabled==(policy==1));
  if(policy==2){assert(h.watchdog.captured);for(unsigned i=0;i<16;++i)assert(std::abs(h.watchdog.output.q[i]-h.measured[i])<1e-6);}
 }
 for(const bool fault:{false,true}){CycleHarness h;h.node.position_submit(h.command(1));h.settle();for(unsigned i=0;i<25;++i)h.tick();
  auto old=h.command(0);inspect_hold=true;inspect_entered=false;h.node.position_submit(old);while(!inspect_entered.load())std::this_thread::yield();
  if(fault)snapshot.status[0]|=8;
  h.interrupt();inspect_hold=false;
  for(unsigned i=0;i<35;++i)h.tick();
  const auto admitted=h.node.position_gate_.last().sequence;
  assert(admitted!=old->meta.sequence&&!h.node.position_gate_.live(cycle_now_ns)&&h.watchdog.disabled);
  if(fault)assert(h.watchdog.reason==DRIVE_FAULT);else assert(h.watchdog.reason==CYCLE_OR_TRACKING_ERROR);
  snapshot.status[0]=0x27;h.node.position_submit(old);for(unsigned i=0;i<30;++i)h.tick();
  assert(h.node.position_gate_.last().sequence==admitted&&h.watchdog.disabled);
 }
 std::cout<<"PASS actual step/cache/watchdog source expiry captures measured pose; fault/revocation plus late worker/replay cannot reactivate output; fault/cycle reasons remain latched\n";
}
void check_cycle_powered_retake(){
 using namespace position_session;CycleHarness h;h.node.position_policy_=1;
 h.node.position_submit(h.command(1));h.settle();
 for(unsigned i=0;i<400;++i){if(i%20==0)h.node.position_submit(h.command(0));h.tick();}
 h.node.position_submit(h.command(2));for(unsigned i=0;i<2300;++i)h.tick();
 assert(h.watchdog.reason==PAUSED&&!h.watchdog.disabled&&snapshot.hold_valid);
 const auto held=h.watchdog.output.reference_position;
 assert(std::abs(held[0]-h.measured[0])>.002);
 // New ownership may change the goal, never jump the already-powered reference
 // back to a displaced measurement. Compare the old measured-seed behavior too.
 TargetSnapshot old_seed;assert(h.node.position_trajectory_.snapshot(cycle_now_ns,cycle_now_ns,1,10000000,h.measured,old_seed));
 assert(std::abs(old_seed.q[0]-held[0])>.002);
 ++h.session;h.sequence=0;h.node.position_submit(h.command(1,0.));
 auto previous=held;bool adopted=false;
 for(unsigned i=0;i<100;++i){
  const auto output=h.tick();assert(!h.watchdog.disabled);
  for(unsigned j=0;j<16;++j)assert(std::abs(output.reference_position[j]-previous[j])<.0003);
  previous=output.reference_position;adopted|=output.session==h.session;
 }
 assert(adopted);h.node.position_submit(h.command(2));
 for(unsigned i=0;i<2300;++i)h.tick();
 assert(h.watchdog.reason==PAUSED&&!h.watchdog.disabled&&snapshot.hold_valid);
 std::cout<<"PASS powered hold -> squeeze retake -> PAUSE: offset measurement never jumps reference, no expiry disable\n";
}
void check_cycle_fault_diagnostics(){
 using namespace position_session;
 auto hold=[](CycleHarness& h){
  h.node.position_submit(h.command(1,0.));h.settle();
  h.node.position_submit(h.command(2,0.));h.settle();
  for(unsigned i=0;i<35;++i)h.tick();
  assert(h.watchdog.reason==PAUSED&&!h.watchdog.disabled&&snapshot.hold_valid);
 };
 // A fixed, healthy hold still obeys the unchanged main-loop 10 ms bound.
 for(int gap_ms:{9,11}){
  CycleHarness h;hold(h);const auto enables=enable_requests;
  h.next_cycle+=(gap_ms-1)*1000000LL;h.tick();
  assert(enable_requests==enables);
  assert(h.watchdog.disabled==(gap_ms==11));
  if(gap_ms==11){
   assert(h.watchdog.reason==CYCLE_OR_TRACKING_ERROR&&!snapshot.hold_valid);
#ifdef POSITION_CYCLE_DIAGNOSTICS
   const auto fault=snapshot.cycle_fault;
   assert(fault.origin==1&&fault.checks==1&&fault.axis==-1&&fault.dt_s>.010&&fault.dt_s<.012);
   assert(fault.session==h.session&&fault.sequence==h.sequence&&fault.pdo_sample>0);
   for(unsigned i=0;i<4;++i)h.tick();
   assert(snapshot.cycle_fault.detected_ns==fault.detected_ns);
#endif
  }
 }
 // Stop ROS sampling but keep independent PDO/RT progress healthy: the RT
 // reference deadline can fail first. A later main-loop error must not rewrite it.
 {
  CycleHarness h;hold(h);const auto packet=h.cache.read();
  h.next_cycle+=23000000;cycle_now_ns=h.next_cycle;h.node.feedback();
  periodic_allocations=periodic_deletions=0;count_periodic_memory=true;
  h.watchdog.evaluate(packet,snapshot,cycle_clock());h.watchdog.report_hold(snapshot,controlled_mask);
  count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);
  assert(packet.failure_reason==0&&h.watchdog.reason==CYCLE_OR_TRACKING_ERROR&&h.watchdog.disabled);
#ifdef POSITION_CYCLE_DIAGNOSTICS
  const auto first=snapshot.cycle_fault;
  assert(first.origin==2&&first.checks==0&&first.detected_ns-first.rt_deadline_ns>=3000000);
#endif
  h.tick();assert(h.node.position_output_.failure_reason==CYCLE_OR_TRACKING_ERROR);
#ifdef POSITION_CYCLE_DIAGNOSTICS
  assert(h.node.position_output_.cycle_fault.checks==1);
  assert(snapshot.cycle_fault.origin==2&&snapshot.cycle_fault.detected_ns==first.detected_ns);
#endif
  ++h.session;h.sequence=0;h.node.position_submit(h.command(1,0.));h.settle();
  assert(!h.watchdog.disabled);
#ifdef POSITION_CYCLE_DIAGNOSTICS
  assert(!snapshot.cycle_fault.detected_ns&&!h.node.position_output_.cycle_fault.detected_ns);
#endif
 }
 // The target-stream tracking guard allows up to 0.25 rad on either side;
 // larger error still latches the actual axis and disables the owned group.
 for(unsigned axis:{0u,7u,14u,15u})for(double error:{-.251,-.25,-.249,.249,.25,.250166999,.251}){
  CycleHarness h;
  // Binary-exact starting pose keeps the exact 0.25 boundary independent of
  // float PDO rounding; the injected measurement is not copied from a target.
  h.measured[axis]=.125;snapshot.position[position_slot(axis)]=.125f;
  hold(h);h.measured[axis]=float(.125+error);snapshot.position[position_slot(axis)]=float(h.measured[axis]);h.tick();
  const bool exceeded=std::abs(error)>.25;
  assert(h.watchdog.disabled==exceeded);
  if(!exceeded){assert(h.watchdog.reason==PAUSED&&snapshot.hold_valid);continue;}
  assert(h.watchdog.reason==CYCLE_OR_TRACKING_ERROR&&!snapshot.hold_valid);
#ifdef POSITION_CYCLE_DIAGNOSTICS
  assert(snapshot.cycle_fault.origin==1&&snapshot.cycle_fault.checks==16&&snapshot.cycle_fault.axis==int16_t(axis));
  assert(std::abs(snapshot.cycle_fault.reference-snapshot.cycle_fault.measured)>.25);
#endif
 }
 // No active trajectory is a sampling error, not an interval/tracking error.
 {
  CycleHarness h;auto p=h.command(1,0.);auto t=cycle_clock();
  h.node.position_gate_.commit(meta(p->meta),t,t);h.node.position_interval_.reset(t);
  auto& output=h.node.position_output_;output.selected=true;output.policy=2;output.session=h.session;output.sequence=h.sequence;output.deadline=h.node.position_gate_.deadline();
  h.tick();assert(h.watchdog.reason==CYCLE_OR_TRACKING_ERROR&&h.watchdog.disabled);
#ifdef POSITION_CYCLE_DIAGNOSTICS
  assert(snapshot.cycle_fault.origin==1&&snapshot.cycle_fault.checks==2);
#endif
 }
 std::cout<<"PASS unchanged 9/11ms hold boundary, independent 23ms RT expiry, first-failure latch/recovery, tracking/sampling distinctions and zero periodic allocation\n";
}

int main(){check_cycle_fault_diagnostics();check_cycle_stream(20,1);check_cycle_stream(10,2);check_cycle_stream(20,1,10000);check_cycle_stream(10,2,-10000);check_cycle_expiry_fault_revoke();check_cycle_powered_retake();}

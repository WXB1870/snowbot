// Actual submit/dispatch/poll/step, OutputCache and Watchdog; clocks and feedback
// are injected memory fixtures. This does not measure ROS or EtherCAT scheduling.
struct CycleHarness {
 ServoNode node;position_session::OutputCache cache;position_session::Watchdog watchdog;
 int64_t next_cycle=1000000000LL;uint64_t sequence=0,session=801;
 std::array<double,16> measured{};
 CycleHarness(){
  using namespace position_session;cycle_now_ns=next_cycle;active_cycle_cache=&cache;
  snapshot=Observation{};snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);
  snapshot.position[0]=.015;snapshot.position[1]=.2;snapshot.position[9]=-.2;snapshot.position[18]=.12;snapshot.position[19]=-.08;
  for(unsigned i=0;i<16;++i)measured[i]=snapshot.position[position_slot(i)];
  node.position_reference_mode_=2;node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_,[](auto& result){
   inspect_entered=true;while(inspect_hold.load())std::this_thread::yield();
   if(auto error=inspect_servo_command(*result.packet,result.origin.get(),result.context,result.command))throw std::invalid_argument(error);
  });node.feedback();
 }
 Packet::SharedPtr command(uint8_t operation=0,double displacement=.1){
  node.feedback();auto p=std::make_shared<Packet>();const auto now=cycle_clock();
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
void check_cycle_stream(int period){
 using namespace position_session;CycleHarness h;auto take=h.command(1);h.node.position_submit(take);h.settle();assert(h.node.position_gate_.last().sequence==1);
 bool moved=false;double largest_speed=0;
 for(int tick=0;tick<400;++tick){
  if(tick%period==0){auto target=h.command(0);h.node.position_submit(target);}
  const auto packet=h.tick();assert(!h.watchdog.disabled);moved|=std::abs(packet.reference_position[0]-h.measured[0])>1e-5;largest_speed=std::max(largest_speed,std::abs(packet.reference_speed[0]));
 }
 assert(moved&&largest_speed>.03);auto pause=h.command(2);h.node.position_submit(pause);
 const auto expiry=pause->meta.expires_at_ns;bool braking=false,held=false;std::array<double,16> fixed{};bool have_fixed=false;int64_t stop_deadline=0;
 for(int tick=0;tick<2200;++tick){
  const auto packet=h.tick();assert(!h.watchdog.disabled);
  if(packet.sequence==pause->meta.sequence){assert(packet.source_stamp_ns==pause->meta.source_stamp_ns&&packet.expires_at_ns==expiry);if(!stop_deadline)stop_deadline=packet.deadline;assert(packet.deadline==stop_deadline);braking|=packet.trajectory_phase==3;
   if(packet.trajectory_phase==4){held=true;if(!have_fixed){fixed=packet.reference_position;have_fixed=true;}assert(packet.reference_position==fixed);}
  }
  if(cycle_now_ns<expiry&&held)assert(h.watchdog.reason==PAUSED);
 }
 assert(braking&&held&&h.watchdog.reason==EXPIRED&&h.watchdog.captured);
 assert(snapshot.hold_valid);assert(std::abs(h.watchdog.hold[0]-h.measured[0])>1e-5); // Reference hold acknowledgement is not physical arrival.
 std::cout<<"PASS actual position step/cache/watchdog "<<1000/period<<"Hz: coupled nominal output, fixed 2s PAUSE source/hold, expiry reason, independent measured feedback, zero counted periodic new/delete\n";
}
void check_cycle_expiry_fault_revoke(){
 using namespace position_session;
 {CycleHarness h;h.node.position_submit(h.command(1));h.settle();
  for(unsigned i=0;i<40;++i)h.tick();
  const auto deadline=h.node.position_gate_.deadline();
  while(cycle_now_ns<deadline+30000000)h.tick();
  assert(h.watchdog.reason==EXPIRED&&!h.watchdog.disabled&&h.watchdog.captured);
  for(unsigned i=0;i<16;++i)assert(std::abs(h.watchdog.output.q[i]-h.measured[i])<1e-6);
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
int main(){check_cycle_stream(20);check_cycle_stream(10);check_cycle_expiry_fault_revoke();}

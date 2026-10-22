void check_target_production(){
 using namespace position_session;
 ServoNode node;node.position_reference_mode_=2;
 node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_,[](auto& result){
  inspect_entered=true;while(inspect_hold.load())std::this_thread::yield();
  if(auto error=inspect_servo_command(*result.packet,result.origin.get(),result.context,result.command))throw std::invalid_argument(error);
 });
 auto packet=[&](uint64_t sequence,uint8_t operation=0,uint64_t session=71){
  node.feedback();auto p=std::make_shared<Packet>();const auto t=steady_ns();
  p->meta.session=session;p->meta.sequence=sequence;p->meta.operation=operation;p->meta.reference_mode=2;
  p->meta.source_stamp_ns=t;p->meta.expires_at_ns=t+200000000;p->meta.feedback_boot_id=node.position_feedback_.boot;p->meta.feedback_sample=node.position_feedback_.sample;p->meta.middle_instance_id=32;
  for(unsigned i=0;i<7;++i){p->command.l_position[i]=node.position_feedback_.q[i];p->command.r_position[i]=node.position_feedback_.q[i+7];p->command.l_kp[i]=p->command.r_kp[i]=40;p->command.l_kd[i]=p->command.r_kd[i]=2;}
  p->command.l_position[0]+=.002;p->waist_position={node.position_feedback_.q[14],node.position_feedback_.q[15]};
  p->trajectory.version=2;p->trajectory.segment_id=sequence;p->trajectory.max_velocity.fill(.25);p->trajectory.max_acceleration.fill(.5);p->trajectory.max_jerk.fill(162);return p;
 };
 auto tick=[&](){node.feedback();count_periodic_memory=true;node.position_poll();count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);};
 auto drain=[&](){const auto until=steady_ns()+1000000000;do{tick();assert(steady_ns()<until);std::this_thread::sleep_for(std::chrono::milliseconds(1));}while(node.position_admission_->busy()||node.position_target_pending_);};
 auto take=packet(1,1);node.position_submit(take);drain();assert(node.position_gate_.last().sequence==1&&node.position_gate_.last().session==71);
 AdmissionReceipt receipt;assert(node.position_receipts_.pop(receipt)&&receipt.accepted);
 // Same-source derived auxiliary frames update gains/torque without replanning or renewal.
 const auto generation=node.position_gate_.generation();const auto deadline=node.position_gate_.deadline();
 const auto sample_time=steady_ns()+10000000;assert(node.position_trajectory_.sample(sample_time));
 const auto nominal_q=node.position_trajectory_.q,nominal_v=node.position_trajectory_.v,nominal_a=node.position_trajectory_.a;
 auto derived=std::make_shared<Packet>(*take);derived->frame_sequence=2;derived->command.l_kp[0]=41;derived->command.l_torque_grav_comp[0]=.123;
 node.position_submit(derived);drain();assert(node.position_v2_.frame==2&&node.position_output_.kp[0]==41&&std::abs(node.position_output_.torque[0]-.123f)<1e-8f);
 assert(node.position_gate_.generation()==generation&&node.position_gate_.deadline()==deadline&&node.position_gate_.last().sequence==1);
 assert(node.position_trajectory_.sample(sample_time)&&node.position_trajectory_.q==nominal_q&&node.position_trajectory_.v==nominal_v&&node.position_trajectory_.a==nominal_a);
 assert(!node.position_receipts_.pop(receipt));
 // Invalid target is rejected on fixed storage without periodic allocation/deletion.
 auto invalid=packet(2);invalid->command.l_position[0]=NAN;
 count_periodic_memory=true;node.position_submit(invalid);count_periodic_memory=false;
 assert(!periodic_allocations&&!periodic_deletions&&node.position_gate_.generation()==generation);
 assert(std::string(node.position_cycle_reason_)=="joint_limit");
 // Continuous input updates pending only: never cancels the ordinary worker result.
 auto first=packet(2);inspect_hold=true;inspect_entered=false;node.position_submit(first);while(!inspect_entered.load())std::this_thread::yield();
 node.position_submit(derived);assert(!node.position_target_pending_);
 auto newest=packet(3);for(uint64_t seq=3;seq<32;++seq){newest=packet(seq);count_periodic_memory=true;node.position_submit(newest);count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);}
 assert(node.position_target_pending_==newest&&node.position_admission_source_.sequence==2);inspect_hold=false;drain();
 assert(node.position_gate_.last().sequence==31);bool saw_first=false,saw_latest=false;
 while(node.position_receipts_.pop(receipt)){saw_first|=receipt.accepted&&receipt.sequence==2;saw_latest|=receipt.accepted&&receipt.sequence==31;}
 assert(saw_first&&saw_latest);
 // A single compute tail retries its original source without TTL renewal; next inputs recover.
 uint64_t seq=32;
 for(const int delay:{12,25,100}){
  auto delayed=packet(seq++);const auto expires=delayed->meta.expires_at_ns;inspect_hold=true;inspect_entered=false;node.position_submit(delayed);while(!inspect_entered.load())std::this_thread::yield();
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));tick();inspect_hold=false;drain();
  assert(node.position_gate_.last().sequence==delayed->meta.sequence&&node.position_gate_.last().expires==expires);
  auto recover=packet(seq++);node.position_submit(recover);drain();assert(node.position_gate_.last().sequence==recover->meta.sequence);
 }
 // PAUSE supersedes an in-flight ordinary generation. New ordinary targets cannot replace it.
 auto ordinary=packet(seq++);inspect_hold=true;inspect_entered=false;node.position_submit(ordinary);while(!inspect_entered.load())std::this_thread::yield();
 auto pause=packet(seq++,2);node.position_submit(pause);const auto barrier=node.position_target_barrier_;
 for(unsigned i=0;i<12;++i)node.position_submit(packet(seq++));
 assert(node.position_target_pending_==pause&&node.position_target_barrier_==barrier);inspect_hold=false;drain();
 assert(node.position_gate_.last().operation==2&&node.position_gate_.last().sequence==pause->meta.sequence);
 auto pause_derived=std::make_shared<Packet>(*pause);pause_derived->frame_sequence=2;pause_derived->command.l_torque_grav_comp[0]=.234;
 const auto pause_generation=node.position_gate_.generation();node.position_submit(pause_derived);drain();
 assert(node.position_target_barrier_==barrier&&node.position_gate_.generation()==pause_generation&&node.position_v2_.frame==2&&std::abs(node.position_output_.torque[0]-.234f)<1e-8f);
 // A late PAUSE while already in flight remains prior to later ordinary input.
 auto restart=packet(1,1,72);node.position_submit(restart);drain();
 inspect_hold=true;inspect_entered=false;auto late_pause=packet(2,2,72);node.position_submit(late_pause);while(!inspect_entered.load())std::this_thread::yield();
 for(unsigned i=3;i<20;++i)node.position_submit(packet(i,0,72));
 std::this_thread::sleep_for(std::chrono::milliseconds(25));tick();inspect_hold=false;drain();
 assert(node.position_gate_.last().operation==2&&node.position_gate_.last().sequence==2&&node.position_gate_.last().expires==late_pause->meta.expires_at_ns);
 // Revocation rejects a completed result instead of reactivating its reference.
 auto replacement=packet(1,1,73);inspect_hold=true;inspect_entered=false;node.position_submit(replacement);while(!inspect_entered.load())std::this_thread::yield();node.position_gate_.interrupt();inspect_hold=false;drain();
 assert(node.position_gate_.last().session==72&&!node.position_gate_.live(steady_ns()));
 // Exhaust the fixed packet retirement ring while the worker is deliberately held.
 auto overflow_take=packet(1,1,74);node.position_submit(overflow_take);drain();
 inspect_hold=true;inspect_entered=false;auto overflow_inflight=packet(2,0,74);node.position_submit(overflow_inflight);while(!inspect_entered.load())std::this_thread::yield();
 unsigned merged_count=0;
 for(uint64_t n=3;n<100;++n){auto update=packet(n,0,74);count_periodic_memory=true;node.position_submit(update);count_periodic_memory=false;
  assert(!periodic_allocations&&!periodic_deletions);++merged_count;
  if(!node.position_gate_.live(steady_ns()))break;
 }
 assert(merged_count>=63&&!node.position_gate_.live(steady_ns())&&std::string(node.position_cycle_reason_)=="target_retirement_full");
 inspect_hold=false;drain();assert(!node.position_gate_.live(steady_ns()));
 assert(!periodic_allocations&&!periodic_deletions);
 std::cout<<"PASS actual target submit/dispatch/poll: derived auxiliary updates without replan/TTL renewal, bounded latest merge, no ordinary starvation, 12/25/100ms single-tail retry, PAUSE priority including late PAUSE, original expiry, revocation, fixed-capacity/error handling, zero periodic new/delete\n";
}

void check_bootstrap_interrupts(){
 using namespace position_session;
 for(unsigned scenario=0;scenario<6;++scenario){
  ServoNode node;node.position_reference_mode_=2;
  node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_,[](auto& result){inspect_entered=true;while(inspect_hold.load())std::this_thread::yield();if(auto e=inspect_servo_command(*result.packet,result.origin.get(),result.context,result.command))throw std::invalid_argument(e);});
  auto make=[&](uint64_t session,uint64_t sequence,uint8_t operation){node.feedback();auto p=std::make_shared<Packet>();const auto now=steady_ns();p->meta.session=session;p->meta.sequence=sequence;p->meta.operation=operation;p->meta.reference_mode=2;p->meta.source_stamp_ns=now;p->meta.expires_at_ns=now+(operation==2?2000000000LL:200000000LL);p->meta.feedback_boot_id=node.position_feedback_.boot;p->meta.feedback_sample=node.position_feedback_.sample;p->meta.middle_instance_id=32;
   for(unsigned i=0;i<7;++i){p->command.l_position[i]=node.position_feedback_.q[i];p->command.r_position[i]=node.position_feedback_.q[i+7];p->command.l_kp[i]=p->command.r_kp[i]=40;p->command.l_kd[i]=p->command.r_kd[i]=2;}p->waist_position={node.position_feedback_.q[14],node.position_feedback_.q[15]};p->trajectory.version=2;p->trajectory.segment_id=sequence;p->trajectory.max_velocity.fill(.25);p->trajectory.max_acceleration.fill(.5);p->trajectory.max_jerk.fill(162);return p;};
  auto drain=[&](){const auto until=steady_ns()+1000000000;do{node.feedback();periodic_allocations=periodic_deletions=0;count_periodic_memory=true;node.position_poll();count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);assert(steady_ns()<until);std::this_thread::sleep_for(std::chrono::milliseconds(1));}while(node.position_admission_->busy()||node.position_target_pending_);};
  uint64_t session=900+scenario*3;
  if(scenario==1){node.position_submit(make(session-1,1,1));drain();node.position_interrupt();}
  const auto enabled_before=enable_requests;auto take=make(session,1,1);inspect_hold=true;inspect_entered=false;node.position_submit(take);while(!inspect_entered.load())std::this_thread::yield();
  if(scenario<=1){auto pause=make(session,2,2);const auto before=node.position_target_barrier_;count_periodic_memory=true;node.position_submit(pause);count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);assert(node.position_target_barrier_>before&&!node.position_target_takeover_valid_);assert(std::string(node.position_cycle_reason_)=="takeover_canceled_before_activation");}
  else if(scenario==2){count_periodic_memory=true;node.position_interrupt();count_periodic_memory=false;assert(!periodic_allocations&&!periodic_deletions);}
  else if(scenario==3){auto wrong=make(session,2,2);wrong->meta.lease_id="wrong-lease";const auto before=node.position_target_barrier_;node.position_submit(wrong);assert(node.position_target_barrier_==before&&node.position_target_takeover_valid_);}
  else if(scenario==4)node.position_initialized_=false;
  else std::this_thread::sleep_for(std::chrono::milliseconds(210));
  inspect_hold=false;drain();
  if(scenario==3){assert(node.position_gate_.last().session==session&&enable_requests==enabled_before+16);continue;}
  assert(node.position_gate_.last().session!=session&&enable_requests==enabled_before&&!node.position_target_takeover_valid_);
  node.position_submit(take);drain();assert(node.position_gate_.last().session!=session&&enable_requests==enabled_before);
  node.position_initialized_=true;node.position_submit(make(session+1,1,1));drain();assert(node.position_gate_.last().session==session+1&&enable_requests==enabled_before+16);
 }
 std::cout<<"PASS actual bootstrap first/recovery in-flight PAUSE and helper interruption retire old takeover without enables; wrong authority cannot cancel; terminal rejection/expiry clears pending identity; fresh-session recovery and zero counted periodic new/delete\n";
}

int main(){
 using namespace position_session;
 snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;
 snapshot.status.fill(0x27);snapshot.output_valid=snapshot.reference_valid=true;
 snapshot.position[1]=.2f;snapshot.position[9]=-.2f;snapshot.position[3]=-.916625f;
 ServoNode node;node.shutdown_mode_=true;
#ifdef POSITION_SHUTDOWN_IMPEDANCE
 node.session_execution_mode_="impedance";
#endif
 node.position_reference_mode_=2;node.position_local_hold_=true;node.feedback();
 node.position_interval_.reset(steady_ns());
 auto& packet=node.position_output_;packet.selected=true;packet.policy=2;
 packet.kp.fill(1);packet.kd.fill(.2f);packet.torque.fill(6);
 TargetSnapshot seed;seed.valid=true;
 for(unsigned i=0;i<joint_count;++i)seed.q[i]=snapshot.position[position_slot(i)];
 packet.reference_position=seed.q;packet.reference_speed=seed.v;packet.reference_acceleration=seed.a;
 seed.effective_steady=steady_ns()+50000000;
 node.position_shutdown_.begin(seed,seed.q,.25,.5,162.,steady_ns(),8.);
 // A finite local shutdown can leave an expired client TTL without extending it.
 packet.deadline=steady_ns()-1000000;const auto original_deadline=packet.deadline;
 while(steady_ns()<seed.effective_steady){
  node.feedback();assert(!node.position_shutdown_tick());
  assert(applied.kp[3]==1&&applied.kd[3]==.2f&&applied.torque[3]==6);
  assert(std::abs(applied.q[3]-seed.q[3])<1e-7&&!applied.shutdown_disable);
  assert(enable_requests==0&&applied.deadline==original_deadline);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }
 node.feedback();assert(!node.position_shutdown_tick());
 assert(node.position_shutdown_.sampling());
#ifdef POSITION_SHUTDOWN_IMPEDANCE
 assert(shutdown_controls>0);
#endif
 assert(std::abs(applied.reference_position[3]-seed.q[3])<1e-12);
 assert(applied.q[3]==snapshot.position[3]&&applied.kp[3]==1&&applied.torque[3]==6);
 snapshot.wkc=28;node.feedback();assert(!node.position_shutdown_tick());
 assert(applied.shutdown_disable&&node.position_shutdown_.failed()&&enable_requests==0);
 snapshot.wkc=67;snapshot.shutdown_disabled=true;node.feedback();
 assert(node.position_shutdown_tick()&&node.shutdown_exit_code_==1);
 // Never re-enable a base which is idle or has not completed startup.
 ServoNode idle;idle.shutdown_mode_=true;idle.position_startup_complete_=false;
 snapshot.shutdown_disabled=false;idle.feedback();assert(!idle.position_shutdown_tick());
 assert(applied.shutdown_disable&&enable_requests==0);
 snapshot.shutdown_disabled=true;idle.feedback();assert(idle.position_shutdown_tick());
 // Exercise production entry with the actual 10ms preparation setting.
 ServoNode entry;entry.shutdown_mode_=true;entry.position_reference_mode_=2;
#ifdef POSITION_SHUTDOWN_IMPEDANCE
 entry.session_execution_mode_="impedance";
#endif
 snapshot.shutdown_disabled=false;snapshot.session_reason=OK;entry.feedback();
 bindu::execution::JointLimits limits;
 for(unsigned i=0;i<joint_count;++i)limits.push_back({lower[i],upper[i],.25,.5,162.});
 bindu::execution::JointState rest;
 rest.q.assign(seed.q.begin(),seed.q.end());rest.v.assign(joint_count,0.);rest.a=rest.j=rest.v;
 TrajectoryQueue::Prepared prepared;
 prepared.segment=std::make_unique<const bindu::execution::PreparedSegment>(
  bindu::execution::PreparedSegment::target(limits,rest,rest.q,0.,8.,8.));
 auto now=steady_ns();prepared.source={99,1,now,now+200000000,1};prepared.effective=now+3000000;
 prepared.q=seed.q;prepared.velocity.fill(.25);prepared.acceleration.fill(.5);prepared.jerk.fill(162.);prepared.admission=1;
 assert(!entry.position_trajectory_.commit(prepared,now,now,seed.q,seed.v));
 entry.position_interval_.reset(now);entry.position_output_=packet;entry.position_output_.shutdown=false;
 entry.position_output_.shutdown_disable=false;entry.position_output_.failure_reason=0;
 entry.position_output_.deadline=now+200000000;entry.position_output_.reference_position=seed.q;
 for(unsigned i=0;i<7;++i){lArm_shutdownPos[i]=seed.q[i];rArm_shutdownPos[i]=seed.q[i+7];}
 do{
  entry.feedback();assert(!entry.position_shutdown_tick());
  if(entry.position_shutdown_.phase==Shutdown::Phase::Planning)std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }while(entry.position_shutdown_.phase==Shutdown::Phase::Planning);
 assert(entry.position_shutdown_.sampling()&&!entry.position_shutdown_.failed()&&enable_requests==0);
 snapshot.status[0]=0x231;entry.feedback();assert(!entry.position_shutdown_tick());
 assert(applied.shutdown_disable&&entry.position_shutdown_.failed());
 std::cout<<"PASS actual servo shutdown: continuous first output, retained mode/controller, no enable or client TTL renewal, WKC failure and fresh disable ACK\n";
}

int main(){
 using namespace position_session;
 snapshot.valid_mask=0xfffff;snapshot.wkc=snapshot.expected_wkc=67;snapshot.status.fill(0x27);
 snapshot.output_valid=snapshot.reference_valid=true;snapshot.position[1]=.2f;snapshot.position[9]=-.2f;snapshot.position[3]=-.916625f;
 ServoNode node;node.shutdown_mode_=true;node.position_reference_mode_=2;
#ifdef POSITION_SHUTDOWN_IMPEDANCE
 node.session_execution_mode_="impedance";
#endif
 node.position_admission_=std::make_unique<TrajectoryAdmission<Packet>>(node.position_trajectory_);
 std::array<double,joint_count> q{},v{};bindu::execution::JointLimits limits;
 for(unsigned i=0;i<joint_count;++i){q[i]=snapshot.position[position_slot(i)];limits.push_back({lower[i],upper[i],.25,.5,162.});}
 bindu::execution::JointState rest;rest.q.assign(q.begin(),q.end());rest.v.assign(joint_count,0.);rest.a=rest.j=rest.v;
 TrajectoryQueue::Prepared prepared;prepared.q=q;prepared.admission=1;
 prepared.segment=std::make_unique<const bindu::execution::PreparedSegment>(bindu::execution::PreparedSegment::target(limits,rest,rest.q,0.,8.,8.));
 auto now=steady_ns();prepared.source={99,1,now,now+200000000,1};prepared.effective=now+3000000;
 assert(!node.position_trajectory_.commit(prepared,now,now,q,v));node.position_interval_.reset(now);
 auto& packet=node.position_output_;packet.selected=true;packet.policy=2;packet.kp.fill(1);packet.kd.fill(.2f);packet.torque.fill(6);
 packet.deadline=now+200000000;packet.reference_position=q;const auto original_deadline=packet.deadline;
 for(unsigned i=0;i<7;++i){lArm_shutdownPos[i]=q[i];rArm_shutdownPos[i]=q[i+7];}
 bool returned=false,disabled=false,done=false;const auto began=now;
 do{
  node.feedback();snapshot.speed[0]=(steady_ns()-began<300000000)?.1f:0.f;
  done=node.position_shutdown_tick();
  assert(enable_requests==0&&applied.deadline==original_deadline);
  returned|=node.shutdown_phase_==ServoNode::ShutdownReturning;
  if(!applied.shutdown_disable){assert(std::abs(applied.q[3]-q[3])<1e-7&&applied.kp[3]==1&&applied.kd[3]==.2f&&applied.torque[3]==6);}
  else{assert(returned);disabled=true;snapshot.shutdown_disabled=true;}
  if(snapshot.speed[0]>.01)assert(!returned);
  assert(steady_ns()-began<3000000000LL);std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }while(!done);
 assert(returned&&disabled&&node.shutdown_exit_code_==0);
#ifdef POSITION_SHUTDOWN_IMPEDANCE
 assert(shutdown_controls>0);
#endif
 // The watchdog alone owns PDO/fault/enable checks; shutdown cannot clear it.
 Watchdog guard;snapshot.status[0]=0x231;snapshot.shutdown_disabled=false;
 packet.shutdown_disable=false;packet.shutdown_deadline=steady_ns()+1000000000;
 snapshot.sampled_ns=steady_ns();guard.evaluate(packet,snapshot,steady_ns());assert(guard.disabled&&guard.reason==DRIVE_FAULT);
 snapshot.status[0]=0x27;packet.operation=1;packet.admission_generation=99;
 guard.evaluate(packet,snapshot,steady_ns());assert(guard.disabled);
 ServoNode failed;failed.shutdown_phase_=ServoNode::ShutdownPlanning;failed.shutdown_deadline_=steady_ns()+8000000000LL;
 snapshot.output_valid=false;failed.feedback();assert(!failed.position_shutdown_tick()&&applied.shutdown_disable);
 snapshot.shutdown_disabled=true;snapshot.sampled_ns=steady_ns()-21000000;
 assert(!failed.position_shutdown_tick());failed.feedback();assert(failed.position_shutdown_tick()&&failed.shutdown_exit_code_==1);
 ServoNode idle;idle.position_startup_complete_=false;snapshot.shutdown_disabled=false;idle.feedback();
 assert(!idle.position_shutdown_tick()&&enable_requests==0);snapshot.shutdown_disabled=true;idle.feedback();assert(idle.position_shutdown_tick());
 std::cout<<"PASS actual shutdown: shared planner, continuous first frame, measured stop/home, unchanged gains/masks/TTL, watchdog latch and fresh disable ACK\n";
}

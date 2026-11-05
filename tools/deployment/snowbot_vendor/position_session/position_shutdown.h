#pragma once
#include "position_trajectory.h"

namespace position_session {
// Servo-local, finite shutdown action. Planning stays off the sampling thread;
// the existing session packet/watchdog remains the only PDO command writer.
class Shutdown {
 using Segment=bindu::execution::PreparedSegment;
 struct Plan { Segment brake,home; int64_t start=0,completed=0; std::string error; };
 struct Job {
  TargetSnapshot seed; std::array<double,joint_count> goal{};
  double velocity=0.,acceleration=0.,jerk=0.,timeout=0.;
 } job_;
 std::atomic<bool> stopping_{false},ready_{false};
 std::atomic<Plan*> completed_{nullptr};
 std::thread worker_;
 Plan* plan_=nullptr;
 int64_t phase_start_=0,settled_=0,deadline_=0;
 uint64_t confirmed_sample_=0;
 bool failed_=false;
 const char* reason_=nullptr;
 void work(){
  while(!stopping_.load(std::memory_order_acquire)){
   if(!ready_.exchange(false,std::memory_order_acq_rel)){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;
   }
   auto plan=new Plan;plan->start=job_.seed.effective_steady;
   try{
    bindu::execution::JointLimits limits;
    for(unsigned i=0;i<joint_count;++i)
     limits.push_back({lower[i],upper[i],job_.velocity,job_.acceleration,job_.jerk});
    bindu::execution::JointState seed;
    seed.q.assign(job_.seed.q.begin(),job_.seed.q.end());
    seed.v.assign(job_.seed.v.begin(),job_.seed.v.end());
    seed.a.assign(job_.seed.a.begin(),job_.seed.a.end());seed.j.assign(joint_count,0.);
    plan->brake=Segment::stop(limits,seed,0.,job_.timeout);
    auto rest=plan->brake.sample(plan->brake.end());
    rest.v.assign(joint_count,0.);rest.a.assign(joint_count,0.);rest.j.assign(joint_count,0.);
    plan->home=Segment::target(limits,rest,
     std::vector<double>(job_.goal.begin(),job_.goal.end()),0.,job_.timeout,job_.timeout);
   }catch(const std::exception& e){plan->error=e.what();}
   plan->completed=steady_ns();completed_.store(plan,std::memory_order_release);
  }
 }
 bool reached(const Observation& obs,const std::array<double,joint_count>& target,double speed)const{
  for(unsigned i=0;i<joint_count;++i){const auto slot=position_slot(i);
   if(std::abs(obs.position[slot]-target[i])>(i<14?.02:.01) ||
      std::abs(obs.speed[slot])>speed)return false;
  }
  return true;
 }
 bool stable(int64_t now,const Observation& obs,const std::array<double,joint_count>& target,double speed){
  if(!reached(obs,target,speed)){settled_=0;confirmed_sample_=obs.sample;return false;}
  if(obs.sample<=confirmed_sample_)return false;
  confirmed_sample_=obs.sample;
  if(!settled_)settled_=now;
  return now-settled_>=500000000LL;
 }
public:
 enum class Phase { Idle,Planning,Braking,StopHold,Returning,HomeHold,Disabling,Done };
 Phase phase=Phase::Idle;
 Shutdown():worker_([this]{work();}){}
 ~Shutdown(){stopping_.store(true,std::memory_order_release);worker_.join();delete plan_;delete completed_.exchange(nullptr);}
 Shutdown(const Shutdown&)=delete;Shutdown& operator=(const Shutdown&)=delete;
 bool failed()const{return failed_;}
 int64_t deadline()const{return deadline_;}
 const char* reason()const{return reason_?reason_:"shutdown_complete";}
 bool sampling()const{return phase>=Phase::Braking && phase<=Phase::HomeHold;}
 void begin(const TargetSnapshot& seed,const std::array<double,joint_count>& goal,
   double velocity,double acceleration,double jerk,int64_t now,double timeout){
  job_={seed,goal,velocity,acceleration,jerk,timeout};
  phase=Phase::Planning;deadline_=now+int64_t(timeout*1e9);
  ready_.store(true,std::memory_order_release);
 }
 void disable(int64_t now,const char* error=nullptr){
  if(error){failed_=true;reason_=error;}
  if(phase!=Phase::Disabling){phase=Phase::Disabling;deadline_=now+8000000000LL;}
 }
 // False while the old timeline must still be sampled before the dated switch.
 bool poll(int64_t now){
  if(phase!=Phase::Planning)return true;
  if(auto result=completed_.exchange(nullptr,std::memory_order_acq_rel)){
   plan_=result;
   if(!plan_->error.empty()){disable(now,plan_->error.c_str());return true;}
   if(plan_->completed>plan_->start){disable(now,"shutdown_plan_late");return true;}
  }
  if(!plan_ && now>=job_.seed.effective_steady){disable(now,"shutdown_plan_late");return true;}
  if(now>=deadline_){disable(now,"shutdown_plan_timeout");return true;}
  if(!plan_ || now<plan_->start)return false;
  phase=Phase::Braking;phase_start_=plan_->start;
  deadline_=phase_start_+int64_t((plan_->brake.end()+job_.timeout)*1e9);
  return true;
 }
 void tick(int64_t now,const Observation& obs,Output& output,double speed){
  if(phase==Phase::Done)return;
  bool operating=true;
  for(unsigned i=0;i<joint_count;++i)if((obs.status[position_slot(i)]&0x6f)!=0x27)operating=false;
  if(phase!=Phase::Disabling && (!measured(obs,now) ||
     !operating || (fault_mask(obs)&controlled_mask) || obs.session_reason>=FEEDBACK_INVALID))
   disable(now,"shutdown_feedback_or_latched_fault");
  if(phase==Phase::Disabling){
   output.shutdown=true;output.shutdown_disable=true;output.shutdown_deadline=deadline_;
   if(obs.shutdown_disabled && measured(obs,now,0xfffff)){phase=Phase::Done;return;}
   if(now>=deadline_){failed_=true;reason_="shutdown_disable_unconfirmed";phase=Phase::Done;}
   return;
  }
  if(now>=deadline_){disable(now,"shutdown_settle_timeout");tick(now,obs,output,speed);return;}
  if(!sampling())return;
  const auto& segment=(phase==Phase::Braking||phase==Phase::StopHold)?plan_->brake:plan_->home;
  std::array<double,joint_count> j{};
  const double elapsed=std::min((now-phase_start_)*1e-9,segment.end());
  if(!segment.sample_into(elapsed,output.reference_position.data(),output.reference_speed.data(),
      output.reference_acceleration.data(),j.data(),joint_count)){
   disable(now,"shutdown_sample_failed");tick(now,obs,output,speed);return;
  }
  output.shutdown=true;output.shutdown_disable=false;output.shutdown_deadline=deadline_;
  output.operation=0;output.nominal_hold=false;output.trajectory=true;
  output.trajectory_phase=(phase==Phase::Braking?3:phase==Phase::Returning?2:4);
  if(elapsed>=segment.end()){
   if(phase==Phase::Braking)phase=Phase::StopHold;
   if(phase==Phase::Returning)phase=Phase::HomeHold;
   if(stable(now,obs,output.reference_position,speed)){
    if(phase==Phase::StopHold){
     phase=Phase::Returning;phase_start_=now;settled_=0;
     deadline_=now+int64_t((plan_->home.end()+job_.timeout)*1e9);
    }else disable(now);
   }
  }
 }
};
} // namespace position_session

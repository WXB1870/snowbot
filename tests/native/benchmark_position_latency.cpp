// Offline deterministic scheduling model using production generation/commit/sample.
#include "position_trajectory.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
using namespace position_session;
using namespace bindu::execution;
struct Wire {
 unsigned version=2;uint64_t segment_id=0;int64_t effective_at_ns=0;
 std::vector<double> piece_start,duration,source_duration,source_elapsed,controls;
 std::vector<unsigned char> control_count;
 std::array<double,16> max_velocity{},max_acceleration{},max_jerk{};
};
int main(int argc,char** argv){
 if(argc!=6)return 2;
 const int period=std::stoi(argv[1]),jitter=std::stoi(argv[2]),budget=std::stoi(argv[3]),delay=std::stoi(argv[4]),ack=std::stoi(argv[5]);
 TrajectoryQueue queue;JointLimits limits;std::array<double,16> measured{},speed{},goal{};
 measured[1]=.2;measured[8]=-.2;measured[14]=.12;measured[15]=-.08;goal=measured;
 for(unsigned i=0;i<16;++i)limits.push_back({lower[i],upper[i],.25,.5,162});
 const int64_t origin=1000000000;uint64_t sequence=0,accepted=0,rejected=0,merged=0,inputs=0;
 int next=0,ready=-1,ack_until=0;bool has_pending=false,has_reference=false,pause_sent=false;
 Meta source{},pending_source{};std::unique_ptr<TrajectoryQueue::Prepared> pending;
 uint64_t sampled_id=0;
 double sum_error=0,max_error=0,axis0_error=0,axis7_error=0,waist_error=0,max_v=0,max_a=0,max_j=0,continuity_q=0,continuity_v=0,continuity_a=0;
 size_t error_count=0,expired_samples=0,constant_samples=0,constant_rest_samples=0,constant_restarts=0;double constant_sum_v=0,constant_sum_v2=0,constant_min_v=1,constant_max_v=-1;bool constant_was_rest=false;int first_nonzero_input=-1;double reverse_ms=-1,first_motion_ms=-1,stop_ms=-1;
 std::cout<<std::setprecision(12);
 for(int tick=0;tick<5000;++tick){
  const int64_t now=origin+tick*1000000LL;
  if(tick==next && tick<4000){
   ++inputs;auto target=measured;const double t=tick*.001;
   // Micro-motion, constant-speed ramp, acceleration, reversal, non-collinear change.
   if(t<.5)target[0]+=.0005*t/.5;
   else if(t<1.5)target[0]+=.0005+.02*(t-.5);
   else if(t<2.5)target[0]+=.0205-.02*(t-1.5);
   else {target[0]+=.0005+.008*std::sin((t-2.5)*2);target[7]+=.006*std::sin((t-2.5)*3);target[14]+=.003*(t-2.5);}
   if(has_pending)++merged;goal=target;has_pending=true;
   pending_source={1,++sequence,now,now+200000000,has_reference?uint8_t(0):uint8_t(1)};
   if(first_nonzero_input<0&&std::abs(goal[0]-measured[0])>1e-12)first_nonzero_input=tick;
   next+=period+(jitter?((inputs%4==0)?4:(inputs%4==1?-3:inputs%4==2?2:-3)):0);
  }
  if(tick==4000){has_pending=true;pause_sent=true;
   pending_source={1,++sequence,now,now+2000000000LL,2};
  }
  if(pending && tick>=ready){
   const char* error=queue.commit(*pending,now,now,measured,speed);
   if(error){++rejected;std::cout<<"REJECT "<<tick<<' '<<error<<'\n';}
   else {++accepted;has_reference=true;
    std::cout<<"ACCEPT "<<source.sequence<<' '<<(pending->effective-source.stamp)*1e-6<<' '<<(pending->effective-now)*1e-6<<' '<<unsigned(source.operation)<<'\n';}
   pending.reset();queue.reclaim();ack_until=tick+ack;
  }
  if(has_pending && !pending && tick>=ack_until){
   has_pending=false;source=pending_source;
   const auto began=std::chrono::steady_clock::now();
   try{
    TargetSnapshot snapshot;if(!queue.snapshot(now,now,source.operation,budget*1000000LL,measured,snapshot))throw std::runtime_error("snapshot");
    Wire w;w.version=2;w.segment_id=source.sequence;w.max_velocity.fill(.25);w.max_acceleration.fill(.5);w.max_jerk.fill(162);
    pending=TrajectoryQueue::prepare_target(w,source,snapshot,goal);
    const auto generation=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    std::cout<<"GENERATE "<<generation<<'\n';
    ready=tick+std::max(1,int(std::ceil(generation)))+delay;
   }catch(const std::exception& e){++rejected;pending.reset();std::cout<<"REJECT "<<tick<<' '<<e.what()<<'\n';}
  }
  if(has_reference){
   const auto old_q=queue.q,old_v=queue.v,old_a=queue.a;const auto old_id=sampled_id;
   queue.sample(now);
   sampled_id=queue.id();
   if(old_id && sampled_id!=old_id){
    // Compare one-sided polynomial derivatives at the exact shared switch.
    // q/v/a finite differences below additionally guard actual sampled continuity.
    for(unsigned i=0;i<16;++i){continuity_q=std::max(continuity_q,std::abs(queue.q[i]-old_q[i]));continuity_v=std::max(continuity_v,std::abs(queue.v[i]-old_v[i]));continuity_a=std::max(continuity_a,std::abs(queue.a[i]-old_a[i]));}
   }
   if(now>=queue.deadline()){++expired_samples;continue;}
   if(tick>=800&&tick<1450){const double v=queue.v[0];const bool rest=std::abs(v)<1e-5;++constant_samples;constant_rest_samples+=rest;constant_restarts+=!rest&&constant_was_rest;constant_was_rest=rest;constant_sum_v+=v;constant_sum_v2+=v*v;constant_min_v=std::min(constant_min_v,v);constant_max_v=std::max(constant_max_v,v);}
   for(unsigned i=0;i<16;++i){max_v=std::max(max_v,std::abs(queue.v[i]));max_a=std::max(max_a,std::abs(queue.a[i]));max_j=std::max(max_j,std::abs(queue.j[i]));}
   if(tick<4000)for(unsigned i=0;i<16;++i){const double error=std::abs(queue.q[i]-goal[i]);sum_error+=error*error;max_error=std::max(max_error,error);++error_count;if(i==0)axis0_error+=error*error;if(i==7)axis7_error+=error*error;if(i==14)waist_error+=error*error;}
   if(first_motion_ms<0 && std::abs(queue.q[0]-measured[0])>1e-10)first_motion_ms=tick;
   if(tick>=1500 && reverse_ms<0 && queue.v[0]<-1e-7)reverse_ms=tick-1500;
   if(tick>=4000 && pause_sent && stop_ms<0 && queue.phase(now)==4)stop_ms=tick-4000;
  }
 }
 std::cout<<"SUMMARY "<<inputs<<' '<<accepted<<' '<<rejected<<' '<<merged<<' '<<std::sqrt(sum_error/std::max(size_t(1),error_count))<<' '<<max_error<<' '<<reverse_ms<<' '<<first_motion_ms<<' '<<stop_ms<<' '<<max_v<<' '<<max_a<<' '<<max_j<<' '<<continuity_q<<' '<<continuity_v<<' '<<continuity_a<<' '<<expired_samples<<' '<<(first_motion_ms<0?-1:first_motion_ms-first_nonzero_input)<<' '<<constant_rest_samples<<' '<<constant_restarts<<' '<<constant_min_v<<' '<<constant_max_v<<' '<<(constant_samples?std::sqrt(std::max(0.,constant_sum_v2/constant_samples-std::pow(constant_sum_v/constant_samples,2))):0.)<<' '<<std::sqrt(axis0_error/std::max(1.,error_count/16.))<<' '<<std::sqrt(axis7_error/std::max(1.,error_count/16.))<<' '<<std::sqrt(waist_error/std::max(1.,error_count/16.))<<'\n';
 return 0;
}

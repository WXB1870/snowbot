#pragma once
#include "servo_control/position_follower.h"
#include "bindu_execution_native/interpolation.hpp"
#if __has_include("bindu_execution_native/prediction_model.hpp")
#include "bindu_execution_native/prediction_model.hpp"
#define SNOWBOT_RIGID_PREDICTION 1
#endif
#ifndef SNOWBOT_ARM_REFERENCE_MARGIN
#define SNOWBOT_ARM_REFERENCE_MARGIN 0.0
#endif
#include <memory>
#include <stdexcept>
#include <atomic>
#include <string>
#include <thread>
#include <cstring>
#include <functional>
#include "servo_control/position_watchdog.h"
#include "servo_control/position_clock.h"
namespace position_session {
// Source strings are bounded once on the non-periodic worker. The periodic
// receipt/identity path copies only fixed storage and never owns a ROS string.
struct FixedText {
 std::array<char,129> bytes{};uint16_t size=0;
 bool assign(const std::string& text)noexcept{
  if(text.size()>128)return false;
  size=static_cast<uint16_t>(text.size());std::copy(text.begin(),text.end(),bytes.begin());bytes[size]=0;return true;
 }
 void reason(const char* text)noexcept{size=0;while(text[size] && size<128){bytes[size]=text[size];++size;}bytes[size]=0;}
 const char* c_str()const noexcept{return bytes.data();}
 bool operator==(const FixedText& b)const noexcept{return size==b.size&&std::equal(bytes.begin(),bytes.begin()+size,b.bytes.begin());}
 bool operator==(const char* text)const noexcept{return std::strcmp(bytes.data(),text)==0;}
};
struct SourceIdentity {
 FixedText command,lease,profile;uint64_t epoch=0;
 bool authority(const SourceIdentity& b)const noexcept{return lease==b.lease&&profile==b.profile&&epoch==b.epoch;}
};
struct AdmissionContext {
 uint64_t generation=0,instance=0,frame=0,barrier=0;Meta last{};SourceIdentity identity{};
 std::array<double,joint_count> target{};FixedText publisher;
};
struct ServoCommand {
 Meta source{};SourceIdentity identity{};uint64_t frame=0,feedback_boot=0,feedback_sample=0,middle_instance=0;
 std::array<double,joint_count> target{};
 std::array<float,joint_count> kp{},kd{},torque{};
 bool derived=false,target_stream=false;
};
struct RejectedTrajectory {
 Meta source{};FixedText reason;bool active=false;
 void reject(const Meta& m,const char* why)noexcept{source=m;reason.reason(why);active=true;}
 bool matches(const Meta& m)const noexcept{return active&&source==m;}
 bool blocks(const Meta& m)const noexcept{return active&&source.session==m.session&&m.sequence<=source.sequence;}
 void clear()noexcept{active=false;}
};
template<class Packet> const char* inspect_servo_command(const Packet& packet,const Packet* origin,
 const AdmissionContext& context,ServoCommand& result)noexcept{
 result.source=meta(packet.meta);result.frame=packet.frame_sequence;result.target_stream=packet.meta.reference_mode==2;
 result.feedback_boot=packet.meta.feedback_boot_id;result.feedback_sample=packet.meta.feedback_sample;
 result.middle_instance=packet.meta.middle_instance_id;result.identity.epoch=packet.meta.epoch;
 if(!((packet.meta.reference_mode==1&&packet.trajectory.version==1)||(packet.meta.reference_mode==2&&(packet.trajectory.version==2||packet.trajectory.version==3)))||packet.command.if_use_debugmod||!packet.command.if_topic_point_position)return "reference_mode_conflict";
 if(!result.identity.command.assign(packet.meta.source_command_id)||!result.identity.lease.assign(packet.meta.lease_id)||
    !result.identity.profile.assign(packet.meta.profile_hash)||!result.identity.command.size||!result.identity.lease.size||!result.identity.profile.size)return "source_identity_layout";
 for(unsigned i=0;i<7;++i){
  result.target[i]=packet.command.l_position[i];result.target[i+7]=packet.command.r_position[i];
  const double values[]={packet.command.l_kp[i],packet.command.l_kd[i],packet.command.r_kp[i],packet.command.r_kd[i],packet.command.l_torque_grav_comp[i],packet.command.r_torque_grav_comp[i]};
  for(double value:values)if(!std::isfinite(value)||!std::isfinite(float(value)))return "nonfinite_gains";
  if(values[0]<0||values[1]<0||values[2]<0||values[3]<0)return "negative_gains";
  result.kp[i]=float(values[0]);result.kd[i]=float(values[1]);result.kp[i+7]=float(values[2]);result.kd[i+7]=float(values[3]);
  result.torque[i]=float(values[4]);result.torque[i+7]=float(values[5]);
 }
 for(unsigned i=0;i<2;++i)result.target[i+14]=packet.waist_position[i];
 if(!in_limits(result.target))return "joint_limit";
 result.derived=result.source==context.last;
 if(result.source.operation!=1 && !result.identity.authority(context.identity))return "source_authority_changed";
 if(result.derived && (!origin || result.target!=context.target || packet.meta!=origin->meta || packet.trajectory!=origin->trajectory))return "source_target_changed";
 return nullptr;
}

// Fixed-size history travels with the committed segment, never pending input.
struct TargetTrend {
 std::array<double,joint_count> goal{},velocity{};
 int64_t stamp=0;uint64_t session=0;bool rigid=false;std::array<double,joint_count> grasp{};
};
// Snapshot at one future switch instant, taken by the sampling owner. No segment
// pointer escapes; background generation cannot race timeline reclamation.
struct TargetSnapshot {
 TargetTrend trend{};
 std::array<double,joint_count> q{},v{},a{};
 int64_t effective_ros=0,effective_steady=0;uint64_t version=0;bool valid=false,held_takeover=false,inner_reference=false;
};
// The original servo thread owns the bounded timeline and samples it. A separate
// admission worker imports/certifies immutable segments and reclaims old ones.
// Sampling/commit do not plan, allocate, free trajectories, log or renew TTL.
struct PredictionTrace {
 bool rigid=false,accepted=false;double gain=0.;int attempts=0,projection_failures=0,geometry_rejections=0;
 const char* reason="ordinary_trend";
};
class TrajectoryQueue {
 using Segment=bindu::execution::PreparedSegment;
 struct Entry {TargetTrend trend{};std::unique_ptr<const Segment> segment;Meta source{};uint64_t id=0,admission=0;int64_t effective=0,start=0,deadline=0;bool target_stream=false,inner_reference=false;};
 std::array<Entry,8> entries_{};unsigned count_=0;const Entry* selected_=nullptr;int64_t clock_offset_=0;uint64_t version_=0;
 static_assert(std::atomic<const Segment*>::is_always_lock_free,"lock-free retirement required");
 std::array<std::atomic<const Segment*>,8> retired_{};
 std::array<double,joint_count> velocity_{},acceleration_{},jerk_{};
 const Entry* at(int64_t now)const noexcept {
  if(!count_)return nullptr;
  const Entry* e=&entries_[0];for(unsigned i=1;i<count_;++i)if(entries_[i].start<=now)e=&entries_[i];return e;
 }
 static void require(bool value,const char* text){if(!value)throw std::invalid_argument(text);}
public:
 struct Prepared {
  PredictionTrace prediction{};TargetTrend trend{};
  std::unique_ptr<const Segment> segment;Meta source{};uint64_t id=0,admission=0;int64_t effective=0;uint64_t version=0;bool target_stream=false,held_takeover=false,inner_reference=false;
  std::array<double,joint_count> velocity{},acceleration{},jerk{},q{},v{},a{};
 };
 std::array<double,joint_count> q{},v{},a{},j{};
 ~TrajectoryQueue(){reclaim();}
 // Only the admission worker calls this while the original servo loop runs.
 void reclaim()noexcept{for(auto& slot:retired_)delete slot.exchange(nullptr,std::memory_order_acq_rel);}
 template<class Wire> static std::unique_ptr<Prepared> prepare(const Wire& wire,const Meta& source,
   double max_v=.25,double max_a=.5,double max_j=162.) {
  using namespace bindu::execution;
  require(wire.version==1 && wire.segment_id==source.sequence,"trajectory_version_or_identity");
  const auto pieces=wire.duration.size();
  require(pieces>0 && pieces<=64 && wire.piece_start.size()==pieces && wire.source_duration.size()==pieces &&
          wire.source_elapsed.size()==pieces && wire.control_count.size()==pieces*joint_count &&
          wire.controls.size()<=64*joint_count*6,"trajectory_layout");
  auto result=std::make_unique<Prepared>();JointLimits limits;limits.reserve(joint_count);
  for(unsigned i=0;i<joint_count;++i){
   const double speed=wire.max_velocity[i],accel=wire.max_acceleration[i],jerk=wire.max_jerk[i];
   require(std::isfinite(speed)&&std::isfinite(accel)&&std::isfinite(jerk)&&speed>0&&accel>0&&jerk>0&&
           speed<=max_v&&accel<=max_a&&jerk<=max_j,"trajectory_dynamic_limits");
   limits.push_back({lower[i],upper[i],speed,accel,jerk});
   result->velocity[i]=speed;result->acceleration[i]=accel;result->jerk[i]=jerk;
  }
  std::vector<CurveDescriptor> description;description.reserve(pieces);size_t offset=0;
  for(unsigned p=0;p<pieces;++p){
   CurveDescriptor d;d.start=wire.piece_start[p];d.duration=wire.duration[p];d.source_duration=wire.source_duration[p];d.source_elapsed=wire.source_elapsed[p];
   for(unsigned i=0;i<joint_count;++i){const unsigned n=wire.control_count[p*joint_count+i];
    require((n==4||n==6)&&offset+n<=wire.controls.size(),"trajectory_controls");
    d.controls.emplace_back(wire.controls.begin()+offset,wire.controls.begin()+offset+n);offset+=n;
   }
   description.push_back(std::move(d));
  }
  require(offset==wire.controls.size(),"trajectory_trailing_controls");
  result->segment=std::make_unique<const Segment>(Segment::from_description(description,limits));
  const auto& next=*result->segment;require(next.axes()==joint_count && std::abs(next.start())<1e-9,"trajectory_axes_or_origin");
  const auto first=next.sample(next.start()),last=next.sample(next.end());
  for(unsigned i=0;i<joint_count;++i){require(std::abs(last.v[i])<1e-8&&std::abs(last.a[i])<1e-8,"trajectory_endpoint_not_rest");result->q[i]=first.q[i];result->v[i]=first.v[i];result->a[i]=first.a[i];}
  result->source=source;result->id=wire.segment_id;result->effective=wire.effective_at_ns;return result;
 }
 // The worker uses a dated q/v/a seed, never a moving measured seed. Ordinary
 // targets cannot invalidate this snapshot; only a committed switch changes it.
 bool snapshot(int64_t ros,int64_t steady,uint8_t operation,int64_t budget,
   const std::array<double,joint_count>& measured,TargetSnapshot& out,bool held_takeover=false)const noexcept {
  out={};if(budget<2000000||budget>50000000)return false;
  out.effective_ros=ros+budget;out.effective_steady=steady+budget;out.version=version_;
  if(operation==1){
   if(!held_takeover){out.q=measured;out.valid=true;return true;}
   const auto* entry=at(out.effective_steady);
   if(!entry||entry->source.operation!=2||(out.effective_steady-entry->start)*1e-9<entry->segment->end())return false;
   std::array<double,joint_count> j{};
   out.valid=entry->segment->sample_into((out.effective_steady-entry->start)*1e-9,
     out.q.data(),out.v.data(),out.a.data(),j.data(),joint_count);
   out.inner_reference=entry->inner_reference;
   if(out.inner_reference)out.trend.grasp=entry->trend.grasp;
   out.held_takeover=out.valid;return out.valid;
  }
  out.effective_ros=out.effective_steady-clock_offset_;
  const auto* entry=at(out.effective_steady);if(!entry||out.effective_steady>=entry->deadline)return false;
  out.inner_reference=entry->inner_reference;
  if(entry->target_stream&&entry->source.operation!=2)out.trend=entry->trend;
  if(out.inner_reference)out.trend.grasp=entry->trend.grasp;
  std::array<double,joint_count> j{};
  out.valid=entry->segment->sample_into((out.effective_steady-entry->start)*1e-9,
    out.q.data(),out.v.data(),out.a.data(),j.data(),joint_count);return out.valid;
 }
 template<class Wire> static std::unique_ptr<Prepared> prepare_target(const Wire& wire,const Meta& source,
   const TargetSnapshot& snapshot,const std::array<double,joint_count>& goal,
   double max_v=.25,double max_a=.5,double max_j=162.) {
  using namespace bindu::execution;
  require((wire.version==2||wire.version==3)&&wire.segment_id==source.sequence&&wire.effective_at_ns==0,"target_version_or_identity");
  require(source.operation!=2||wire.version==2,"rigid_pause_requires_original_stop_recipe");
  require(wire.duration.empty()&&wire.piece_start.empty()&&wire.source_duration.empty()&&wire.source_elapsed.empty()&&
    wire.control_count.empty()&&wire.controls.empty(),"target_contains_curve");
  require(snapshot.valid&&snapshot.effective_ros<source.expires,"target_snapshot_or_expired");
  auto result=std::make_unique<Prepared>();JointLimits limits;limits.reserve(joint_count);
  // A new ordinary takeover (explicit preparation/recovery) can leave the
  // tool scope. Continuations and PAUSE inherit its exact reference reserve.
  result->inner_reference=wire.version==3 || (source.operation!=1&&snapshot.inner_reference);
  if(result->inner_reference)result->trend.grasp=snapshot.inner_reference?snapshot.trend.grasp:snapshot.q;
  for(unsigned i=0;i<joint_count;++i){
   const double v=wire.max_velocity[i],a=wire.max_acceleration[i],j=wire.max_jerk[i];
   require(std::isfinite(v)&&std::isfinite(a)&&std::isfinite(j)&&v>0&&a>0&&j>0&&v<=max_v&&a<=max_a&&j<=max_j,"trajectory_dynamic_limits");
   const double margin=result->inner_reference&&i<14 ? SNOWBOT_ARM_REFERENCE_MARGIN : 0.;
   require(std::isfinite(margin)&&margin>=0.&&lower[i]+margin<upper[i]-margin,"reference_reserve_invalid");
   limits.push_back({lower[i]+margin,upper[i]-margin,v,a,j});result->velocity[i]=v;result->acceleration[i]=a;result->jerk[i]=j;
  }
  JointState seed;seed.q.assign(snapshot.q.begin(),snapshot.q.end());seed.v.assign(snapshot.v.begin(),snapshot.v.end());seed.a.assign(snapshot.a.begin(),snapshot.a.end());seed.j.assign(joint_count,0.);
  std::vector<double> hint(joint_count,0.);
  if(source.operation!=2){
   result->trend.goal=goal;result->trend.stamp=source.stamp;result->trend.session=source.session;
   const double dt=(source.stamp-snapshot.trend.stamp)*1e-9;
   if(source.operation==0&&snapshot.trend.session==source.session&&snapshot.trend.rigid==(wire.version==3)&&dt>=.005&&dt<=.1){
    // Derive only from committed goals at their original source times. Reserve speed
    // headroom for catch-up; suppress timing noise in the derivative, not q.
    for(unsigned i=0;i<joint_count;++i){
     const double raw=std::clamp((goal[i]-snapshot.trend.goal[i])/dt,-.8*limits[i].velocity,.8*limits[i].velocity);
     hint[i]=goal[i]==snapshot.trend.goal[i]?0.:snapshot.trend.velocity[i]+dt/(dt+.04)*(raw-snapshot.trend.velocity[i]);
    }
   }
   std::copy(hint.begin(),hint.end(),result->trend.velocity.begin());
  }
  Segment segment;
  if(source.operation==2){segment=Segment::stop(limits,seed,0.,(source.expires-snapshot.effective_ros)*1e-9);result->prediction.reason="pause_stop";}
  else if(wire.version==3){
#ifdef SNOWBOT_RIGID_PREDICTION
   result->trend.rigid=true;
   // A PAUSE/new lease refreshes time and input origins, not the held relation.
   // Only a new ordinary takeover explicitly leaves this tool scope.
   rigid::Model model(prediction_model::nodes,prediction_model::left,prediction_model::right,prediction_model::dofs);
   require(model.dofs==int(joint_count),"rigid_model_joint_layout");rigid::Decision decision;
   const std::vector<double> grasp(result->trend.grasp.begin(),result->trend.grasp.end());
   segment=rigid::constrained(model,limits,seed,std::vector<double>(goal.begin(),goal.end()),hint,model.evaluate(rigid::vec(grasp)),0.,"geometry",1.,decision);
   result->prediction={true,bool(decision.accepted),decision.gain,decision.attempts,decision.projection_failures,decision.guard_rejections,decision.reason};
#else
   throw std::invalid_argument("rigid_prediction_model_not_built");
#endif
  }else segment=Segment::tracking_target(limits,seed,std::vector<double>(goal.begin(),goal.end()),hint,0.,30.,5.);
  result->segment=std::make_unique<const Segment>(segment);result->source=source;result->id=source.sequence;
  result->effective=snapshot.effective_ros;result->q=snapshot.q;result->v=snapshot.v;result->a=snapshot.a;
  result->target_stream=true;result->version=snapshot.version;result->held_takeover=snapshot.held_takeover;return result;
 }
 // Bounded, allocation-free commit on the original servo thread. Ownership is
 // transferred only after every check; retired segments are deleted by worker.
 const char* commit(Prepared& input,int64_t ros,int64_t steady,
   const std::array<double,joint_count>& measured_q,const std::array<double,joint_count>& measured_v)noexcept {
  const auto& source=input.source;
  if(input.target_stream&&input.version!=version_)return "target_snapshot_changed";
  if(input.target_stream&&input.effective<ros+1000000)return "target_generation_late";
  if(input.effective<ros+(input.target_stream?1000000:2000000) || input.effective>ros+150000000 || input.effective>=source.expires)return "trajectory_late_or_expired";
  const int64_t offset=source.operation==1?steady-ros:clock_offset_;
  if(source.operation!=1 && std::abs((steady-ros)-offset)>2000000)return "trajectory_clock_changed";
  const int64_t start=input.effective+offset,deadline=source.expires+offset;
  if(source.operation==2 && !(input.segment->end()*1e9<double(deadline-start)))return "trajectory_stop_exceeds_deadline";
  if(source.operation==1){
   std::array<double,joint_count> held{},hv{},ha{},hj{};
   if(input.held_takeover){
    const auto* old=at(start);
    if(!old||old->source.operation!=2||(start-old->start)*1e-9<old->segment->end()||
       !old->segment->sample_into((start-old->start)*1e-9,held.data(),hv.data(),ha.data(),hj.data(),joint_count))return "trajectory_hold_takeover";
   }
   for(unsigned i=0;i<joint_count;++i){
    if(std::abs(measured_v[i])>.01||std::abs(input.q[i]-measured_q[i])>(input.held_takeover?.25:.002)||
       std::abs(input.v[i])>=1e-8||std::abs(input.a[i])>=1e-8)return "trajectory_takeover_seed";
    if(input.held_takeover&&std::abs(input.q[i]-held[i])>=1e-6)return "trajectory_hold_discontinuous";
   }
  }else{
   if(!count_ || start<=entries_[count_-1].start || start>=entries_[count_-1].deadline)return "trajectory_order_or_takeover";
   for(unsigned i=0;i<joint_count;++i)if(input.velocity[i]!=velocity_[i]||input.acceleration[i]!=acceleration_[i]||input.jerk[i]!=jerk_[i])return "trajectory_limits_changed";
   const auto* old=at(start);std::array<double,joint_count> pq{},pv{},pa{},pj{};
   if(!old->segment->sample_into((start-old->start)*1e-9,pq.data(),pv.data(),pa.data(),pj.data(),joint_count))return "trajectory_previous_sample";
   for(unsigned i=0;i<joint_count;++i)if(std::abs(pq[i]-input.q[i])>=1e-6||std::abs(pv[i]-input.v[i])>=1e-6||std::abs(pa[i]-input.a[i])>=1e-5)return "trajectory_discontinuous";
  }
  unsigned remove=source.operation==1?count_:0;
  if(source.operation!=1)while(remove+1<count_&&entries_[remove+1].start<=steady)++remove;
  const unsigned kept=count_-remove;if(kept>=entries_.size())return "trajectory_queue_full";
  unsigned available=0;for(auto& slot:retired_)available+=slot.load(std::memory_order_acquire)==nullptr;
  if(available<remove)return "trajectory_reclamation_pending";
  for(unsigned i=0;i<remove;++i)for(auto& slot:retired_){const Segment* empty=nullptr;
   if(slot.compare_exchange_strong(empty,entries_[i].segment.get(),std::memory_order_release,std::memory_order_relaxed)){entries_[i].segment.release();break;}}
  for(unsigned i=0;i<kept;++i)if(remove)entries_[i]=std::move(entries_[i+remove]);
  count_=kept;auto& entry=entries_[count_++];entry.segment=std::move(input.segment);entry.source=source;
  entry.inner_reference=input.inner_reference;entry.trend=input.trend;entry.target_stream=input.target_stream;entry.id=input.id;entry.admission=input.admission;entry.effective=input.effective;entry.start=start;entry.deadline=deadline;
  velocity_=input.velocity;acceleration_=input.acceleration;jerk_=input.jerk;selected_=nullptr;clock_offset_=offset;++version_;return nullptr;
 }
 // Non-periodic convenience for native fixtures and offline callers.
 template<class Wire> void accept(const Wire& wire,const Meta& source,int64_t ros,int64_t steady,
   const std::array<double,joint_count>& measured_q,const std::array<double,joint_count>& measured_v,
   double max_v=.25,double max_a=.5,double max_j=162.){
  auto prepared=prepare(wire,source,max_v,max_a,max_j);prepared->admission=source.sequence;const auto error=commit(*prepared,ros,steady,measured_q,measured_v);
  reclaim();if(error)throw std::invalid_argument(error);
 }
 bool sample(int64_t now)noexcept {
  selected_=at(now);return selected_&&selected_->segment->sample_into(std::max(0.,(now-selected_->start)*1e-9),q.data(),v.data(),a.data(),j.data(),joint_count);
 }
 const Meta& source()const noexcept{return selected_->source;}
 uint64_t id()const noexcept{return selected_->id;}
 uint64_t admission()const noexcept{return selected_->admission;}
 int64_t deadline()const noexcept{return selected_->deadline;}
 uint8_t phase(int64_t now)const noexcept{
  if(now<selected_->start)return 1;
  if(source().operation!=2)return selected_->target_stream&&(now-selected_->start)*1e-9>=selected_->segment->end()?5:2;
  return (now-selected_->start)*1e-9>=selected_->segment->end()?4:3;
 }
};
// This is the production periodic admission entry: fixed identity/freshness
// rechecks and queue ownership transfer only. Parsing/graph checks stay worker-side.
struct ServoAdmissionState {
 SourceIdentity identity{};std::array<double,joint_count> target{};uint64_t frame=0;
 RejectedTrajectory rejected{};
 AdmissionContext context(const Gate& gate,uint64_t instance,const std::string& publisher)const noexcept{
  AdmissionContext result;result.generation=gate.generation();result.instance=instance;result.frame=frame;
  result.last=gate.last();result.identity=identity;result.target=target;result.publisher.assign(publisher);return result;
 }
 const char* commit(const ServoCommand& command,TrajectoryQueue::Prepared* prepared,const AdmissionContext& validated,
   uint64_t instance,bool startup,bool mode_free,uint8_t policy,const Tracker& feedback,int64_t ros,int64_t steady,
   Gate& gate,TrajectoryQueue& trajectory,Output& output,Interval& interval)noexcept{
  const auto& source=command.source;
  if(validated.instance!=instance || validated.generation!=gate.generation())return "admission_generation_changed";
  if(!startup || !mode_free)return "startup_shutdown_or_mode_conflict";
  if(!policy)return "stop_policy_unconfirmed";
  if(!feedback.fresh(ros,steady))return "feedback_or_drive_invalid";
  if(command.feedback_boot!=feedback.boot || command.feedback_sample<feedback.ready_sample || command.feedback_sample>feedback.sample || source.stamp<feedback.ready_ros)return "feedback_baseline_mismatch";
  if(rejected.blocks(source))return rejected.reason.c_str();
  const bool derived=source==gate.last();
  if(derived!=command.derived)return "admission_source_changed";
  if(derived){
   if(!gate.live(steady)||!fresh(source,ros,command.target_stream?maximum_target_pause_ttl:maximum_ttl,command.target_stream?maximum_target_ttl:maximum_ttl)||command.frame<=frame)return "duplicate_or_expired_frame";
  }else{
   if(source.operation!=1 && !command.identity.authority(identity))return "source_authority_changed";
   if(!gate.accepts(source,ros,steady,command.target_stream?maximum_target_pause_ttl:maximum_ttl,command.target_stream?maximum_target_ttl:maximum_ttl))return gate.rejection;
   if(!prepared || !(prepared->source==source))return "prepared_source_mismatch";
   prepared->admission=gate.generation()+1;
   if(const auto error=trajectory.commit(*prepared,ros,steady,feedback.q,feedback.v))return error;
   gate.commit(source,ros,steady);interval.admit(steady,source.operation==1);
   identity=command.identity;target=command.target;rejected.clear();
  }
  frame=command.frame;output.kp=command.kp;output.kd=command.kd;output.torque=command.torque;
  output.policy=policy;output.failure_reason=0;output.selected=true;return nullptr;
 }
};

// One optional non-RT importer, one in-flight packet, fixed SPSC handoff.
// The original servo thread never locks or waits for import/reclamation.
template<class Packet> class TrajectoryAdmission {
public:
 struct Result {typename Packet::SharedPtr packet,origin,retired_packet;AdmissionContext context{};ServoCommand command{};std::unique_ptr<TrajectoryQueue::Prepared> prepared;TargetSnapshot snapshot{};std::string error;double prepare_us=0.;};
private:
 TrajectoryQueue& queue_;typename Packet::SharedPtr input_,origin_;AdmissionContext context_{};std::array<double,3> limits_{};std::function<void(Result&)> inspect_;std::function<void(const Result&)> report_;TargetSnapshot snapshot_{};
 std::atomic<bool> stopping_{false},ready_{false},busy_{false};
 static_assert(std::atomic<Result*>::is_always_lock_free,"lock-free admission handoff required");
 std::atomic<Result*> completed_{nullptr},returned_{nullptr};
 std::array<typename Packet::SharedPtr,64> deferred_{};std::atomic<unsigned> deferred_read_{0},deferred_write_{0};
 std::thread worker_;
 void reclaim_packets()noexcept{auto read=deferred_read_.load(std::memory_order_relaxed);const auto write=deferred_write_.load(std::memory_order_acquire);
  while(read!=write){deferred_[read].reset();read=(read+1)%deferred_.size();deferred_read_.store(read,std::memory_order_release);}}

 void run(){
  while(!stopping_.load(std::memory_order_acquire)){
   reclaim_packets();
   if(auto result=returned_.exchange(nullptr,std::memory_order_acq_rel)){
    delete result;queue_.reclaim();busy_.store(false,std::memory_order_release);
   }
   if(!ready_.exchange(false,std::memory_order_acq_rel)){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;
   }
   auto result=new Result;result->packet=std::move(input_);result->origin=std::move(origin_);result->context=context_;result->snapshot=snapshot_;
   const auto began=std::chrono::steady_clock::now();
   try{if(inspect_)inspect_(*result);if(!result->command.derived){if(result->packet->meta.reference_mode==2)result->prepared=TrajectoryQueue::prepare_target(result->packet->trajectory,meta(result->packet->meta),result->snapshot,result->command.target,limits_[0],limits_[1],limits_[2]);else result->prepared=TrajectoryQueue::prepare(result->packet->trajectory,meta(result->packet->meta),limits_[0],limits_[1],limits_[2]);}}
   catch(const std::exception& e){result->error=e.what();}catch(...){result->error="trajectory_import_exception";}
   result->prepare_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-began).count();
   // Observation-only callback; its failure cannot alter control acceptance.
   if(report_)try{report_(*result);}catch(...){}
   completed_.store(result,std::memory_order_release);
  }
  delete completed_.exchange(nullptr);delete returned_.exchange(nullptr);reclaim_packets();queue_.reclaim();
 }
public:
 explicit TrajectoryAdmission(TrajectoryQueue& queue,std::function<void(Result&)> inspect={},std::function<void(const Result&)> report={}):queue_(queue),inspect_(std::move(inspect)),report_(std::move(report)),worker_([this]{run();}){}
 ~TrajectoryAdmission(){stopping_.store(true,std::memory_order_release);worker_.join();}
 bool submit(typename Packet::SharedPtr packet,double velocity,double acceleration,double jerk,const AdmissionContext& context={},typename Packet::SharedPtr origin={},const TargetSnapshot& snapshot={})noexcept{
  if(busy_.exchange(true,std::memory_order_acq_rel))return false;
  input_=std::move(packet);origin_=std::move(origin);context_=context;snapshot_=snapshot;limits_={velocity,acceleration,jerk};ready_.store(true,std::memory_order_release);return true;
 }
 bool defer_packet(typename Packet::SharedPtr& packet)noexcept{
  if(!packet)return true;
  const auto write=deferred_write_.load(std::memory_order_relaxed);const auto next=(write+1)%deferred_.size();
  if(next==deferred_read_.load(std::memory_order_acquire))return false;
  deferred_[write]=std::move(packet);deferred_write_.store(next,std::memory_order_release);return true;
 }
 bool busy()const noexcept{return busy_.load(std::memory_order_acquire);}
 Result* poll()noexcept{return completed_.exchange(nullptr,std::memory_order_acq_rel);}
 void release(Result* result)noexcept{returned_.store(result,std::memory_order_release);}
};
} // namespace position_session

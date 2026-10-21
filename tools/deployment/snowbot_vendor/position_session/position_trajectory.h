#pragma once
#include "servo_control/position_follower.h"
#include "bindu_execution_native/interpolation.hpp"
#include <memory>
#include <stdexcept>
#include <atomic>
#include <string>
#include <thread>
namespace position_session {
// Keep a failed source visible across repeated relay frames and slower state publication.
struct RejectedTrajectory {
 Meta source{};std::string reason;bool active=false;
 void reject(const Meta& m,const std::string& why){source=m;reason=why;active=true;}
 bool matches(const Meta& m)const noexcept{return active&&source==m;}
 bool blocks(const Meta& m)const noexcept{return active&&source.session==m.session&&m.sequence<=source.sequence;}
 void clear()noexcept{active=false;}
};
// The original servo thread owns the bounded timeline and samples it. A separate
// admission worker imports/certifies immutable segments and reclaims old ones.
// Sampling/commit do not plan, allocate, free trajectories, log or renew TTL.
class TrajectoryQueue {
 using Segment=bindu::execution::PreparedSegment;
 struct Entry {std::unique_ptr<const Segment> segment;Meta source{};uint64_t id=0;int64_t effective=0,start=0,deadline=0;};
 std::array<Entry,8> entries_{};unsigned count_=0;const Entry* selected_=nullptr;int64_t clock_offset_=0;
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
  std::unique_ptr<const Segment> segment;Meta source{};uint64_t id=0;int64_t effective=0;
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
 // Bounded, allocation-free commit on the original servo thread. Ownership is
 // transferred only after every check; retired segments are deleted by worker.
 const char* commit(Prepared& input,int64_t ros,int64_t steady,
   const std::array<double,joint_count>& measured_q,const std::array<double,joint_count>& measured_v)noexcept {
  const auto& source=input.source;
  if(input.effective<ros+2000000 || input.effective>ros+150000000 || input.effective>=source.expires)return "trajectory_late_or_expired";
  const int64_t offset=source.operation==1?steady-ros:clock_offset_;
  if(source.operation!=1 && std::abs((steady-ros)-offset)>2000000)return "trajectory_clock_changed";
  const int64_t start=input.effective+offset,deadline=source.expires+offset;
  if(source.operation==2 && !(input.segment->end()*1e9<double(deadline-start)))return "trajectory_stop_exceeds_deadline";
  if(source.operation==1){
   for(unsigned i=0;i<joint_count;++i)if(std::abs(measured_v[i])>.01||std::abs(input.q[i]-measured_q[i])>.002||std::abs(input.v[i])>=1e-8||std::abs(input.a[i])>=1e-8)return "trajectory_takeover_seed";
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
  entry.id=input.id;entry.effective=input.effective;entry.start=start;entry.deadline=deadline;
  velocity_=input.velocity;acceleration_=input.acceleration;jerk_=input.jerk;selected_=nullptr;clock_offset_=offset;return nullptr;
 }
 // Non-periodic convenience for native fixtures and offline callers.
 template<class Wire> void accept(const Wire& wire,const Meta& source,int64_t ros,int64_t steady,
   const std::array<double,joint_count>& measured_q,const std::array<double,joint_count>& measured_v,
   double max_v=.25,double max_a=.5,double max_j=162.){
  auto prepared=prepare(wire,source,max_v,max_a,max_j);const auto error=commit(*prepared,ros,steady,measured_q,measured_v);
  reclaim();if(error)throw std::invalid_argument(error);
 }
 bool sample(int64_t now)noexcept {
  selected_=at(now);return selected_&&selected_->segment->sample_into(std::max(0.,(now-selected_->start)*1e-9),q.data(),v.data(),a.data(),j.data(),joint_count);
 }
 const Meta& source()const noexcept{return selected_->source;}
 uint64_t id()const noexcept{return selected_->id;}
 int64_t deadline()const noexcept{return selected_->deadline;}
 uint8_t phase(int64_t now)const noexcept{
  if(now<selected_->start)return 1;
  if(source().operation!=2)return 2;
  return (now-selected_->start)*1e-9>=selected_->segment->end()?4:3;
 }
};
// One optional non-RT importer, one in-flight packet, fixed SPSC handoff.
// The original servo thread never locks or waits for import/reclamation.
template<class Packet> class TrajectoryAdmission {
public:
 struct Result {typename Packet::SharedPtr packet,retired_packet;std::unique_ptr<TrajectoryQueue::Prepared> prepared;std::string error;};
private:
 TrajectoryQueue& queue_;typename Packet::SharedPtr input_;std::array<double,3> limits_{};
 std::atomic<bool> stopping_{false},ready_{false},busy_{false};
 static_assert(std::atomic<Result*>::is_always_lock_free,"lock-free admission handoff required");
 std::atomic<Result*> completed_{nullptr},returned_{nullptr};std::thread worker_;
 void run(){
  while(!stopping_.load(std::memory_order_acquire)){
   if(auto result=returned_.exchange(nullptr,std::memory_order_acq_rel)){
    delete result;queue_.reclaim();busy_.store(false,std::memory_order_release);
   }
   if(!ready_.exchange(false,std::memory_order_acq_rel)){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;
   }
   auto result=new Result;result->packet=std::move(input_);
   try{result->prepared=TrajectoryQueue::prepare(result->packet->trajectory,meta(result->packet->meta),limits_[0],limits_[1],limits_[2]);}
   catch(const std::exception& e){result->error=e.what();}catch(...){result->error="trajectory_import_exception";}
   completed_.store(result,std::memory_order_release);
  }
  delete completed_.exchange(nullptr);delete returned_.exchange(nullptr);queue_.reclaim();
 }
public:
 explicit TrajectoryAdmission(TrajectoryQueue& queue):queue_(queue),worker_([this]{run();}){}
 ~TrajectoryAdmission(){stopping_.store(true,std::memory_order_release);worker_.join();}
 bool submit(typename Packet::SharedPtr packet,double velocity,double acceleration,double jerk)noexcept{
  if(busy_.exchange(true,std::memory_order_acq_rel))return false;
  input_=std::move(packet);limits_={velocity,acceleration,jerk};ready_.store(true,std::memory_order_release);return true;
 }
 bool busy()const noexcept{return busy_.load(std::memory_order_acquire);}
 Result* poll()noexcept{return completed_.exchange(nullptr,std::memory_order_acq_rel);}
 void release(Result* result)noexcept{returned_.store(result,std::memory_order_release);}
};
} // namespace position_session

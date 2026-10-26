#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <chrono>
namespace position_session {
constexpr int64_t maximum_ttl=200000000;
// Pure position target streams tolerate bounded source gaps; legacy stays 200 ms.
constexpr int64_t maximum_target_ttl=300000000;
// A target-stream PAUSE is a separate stop command, never a motion renewal.
constexpr int64_t maximum_target_pause_ttl=2000000000;
// ROS delivery and progress budget; the raw PDO age limit remains 20 ms.
constexpr int64_t ros_feedback_timeout_ns=40000000;
// Independent position contract: L1..7, R1..7, waist1..2, radians.
constexpr unsigned joint_count=16;
constexpr uint32_t controlled_mask=0xc7f7f;
constexpr unsigned position_slot(unsigned i){return i<7?i:i<14?i+1:i+4;}
// Dual-encoder outer-ring position and motor speed use different resolutions.
constexpr double csp_counts_per_rad=262144.0/6.28318530717958647692;
constexpr double csp_speed_counts_per_rad=65536.0/6.28318530717958647692;
inline int32_t csp_counts(double q){return int32_t(std::round(q*csp_counts_per_rad));}
inline double csp_radians(int32_t q){return q/csp_counts_per_rad;}
inline int32_t csp_speed_counts(double v){return int32_t(std::round(v*csp_speed_counts_per_rad));}
inline double csp_speed_radians(int32_t v){return v/csp_speed_counts_per_rad;}
struct Meta {
 uint64_t session=0,sequence=0;int64_t stamp=0,expires=0;uint8_t operation=0;
 bool operator==(const Meta& b)const{return session==b.session && sequence==b.sequence && stamp==b.stamp && expires==b.expires && operation==b.operation;}
};
template<class M> Meta meta(const M& m){return {m.session,m.sequence,m.source_stamp_ns,m.expires_at_ns,m.operation};}
inline bool fresh(const Meta& m,int64_t now,int64_t pause_limit=maximum_ttl,int64_t motion_limit=maximum_ttl){return m.session && m.sequence && m.stamp>0 && m.expires>m.stamp && m.expires-m.stamp<=(m.operation==2?pause_limit:motion_limit) && now>=m.stamp && now<m.expires && m.operation<=2;}
class Gate {
 bool interrupted_=false;uint64_t generation_=0;Meta last_{};int64_t deadline_=0;uint64_t session_watermark_=0;
public:
 const char* rejection="";
 void interrupt(){if(selected()){if(!interrupted_)++generation_;interrupted_=true;deadline_=0;}}
 // Session identities increase at the source (wire 3 / target wire 5).
 // A constant-size watermark also retires cancelled, unadopted takeovers.
 // Never evict old identities: restamping or resequencing cannot revive them.
 void invalidate_pending(const Meta& pending){
  ++generation_;interrupted_=true;deadline_=0;
  session_watermark_=std::max(session_watermark_,pending.session);
 }
 uint64_t generation()const{return generation_;}
 const Meta& last()const{return last_;}
 bool selected()const{return last_.session!=0;}
 bool live(int64_t now)const{return selected() && now>0 && now<deadline_;}
 bool following(int64_t now)const{return live(now) && last_.operation!=2;}
 int64_t deadline()const{return deadline_;}
 bool accepts(const Meta& m,int64_t ros,int64_t steady,int64_t pause_limit=maximum_ttl,int64_t motion_limit=maximum_ttl) {
  rejection="";
  if(interrupted_ && m.session==last_.session){rejection="new_session_required";return false;}
  if(!fresh(m,ros,pause_limit,motion_limit) || steady<=0){rejection="expired_or_invalid";return false;}
  if(m.session!=last_.session){
   if(m.operation!=1 || following(steady)){
    rejection="session_conflict_or_retired";return false;}
   if(m.session<=session_watermark_){rejection="session_replayed_or_out_of_order";return false;}
  }else if(m.sequence<=last_.sequence || m.stamp<last_.stamp){rejection="duplicate_or_out_of_order";return false;}
  else if(m.operation==0 && (!live(steady) || last_.operation==2)){rejection="takeover_required";return false;}
  return true;
 }
 void commit(const Meta& m,int64_t ros,int64_t steady){
  session_watermark_=std::max(session_watermark_,m.session);
  interrupted_=false;last_=m;deadline_=steady+(m.expires-ros);++generation_;
 }
};
struct Tracker {
 uint64_t received_messages=0;unsigned advances=0;uint64_t ready_sample=0;int64_t ready_ros=0;uint64_t boot=0,sample=0;int64_t progress=0,received=0,source=0;double age=-1;
 uint32_t valid=0,faults=0;int wkc=0,expected=0;std::array<double,joint_count> q{},v{};
 template<class M> void observe(const M& m,int64_t ros,int64_t steady){
  ++received_messages;
  const auto stamp=int64_t(m.header.stamp.sec)*1000000000+m.header.stamp.nanosec;
  valid=0;faults=0;wkc=m.wkc;expected=m.expected_wkc;age=m.pdo_age;
  if(!m.boot_id || !m.pdo_sample || !std::isfinite(age) || age<0 || age>.02 || stamp<=0 || ros<stamp || ros-stamp>ros_feedback_timeout_ns){advances=0;ready_sample=0;ready_ros=0;return;}
  if(boot!=m.boot_id){boot=m.boot_id;sample=0;progress=0;advances=0;ready_sample=0;ready_ros=0;}
  if(m.pdo_sample<sample){advances=0;ready_sample=0;ready_ros=0;return;}
  if(m.pdo_sample>sample){advances=progress>0 && steady>=progress && steady-progress<=ros_feedback_timeout_ns ? std::min(2u,advances+1):1;sample=m.pdo_sample;progress=steady;}
  received=steady;source=stamp;valid=m.valid_mask;
  for(unsigned i=0;i<20;++i)if(m.status[i]&8)faults|=1u<<i;
  for(unsigned i=0;i<joint_count;++i){const auto slot=position_slot(i);q[i]=m.position[slot];v[i]=m.speed[slot];if(!std::isfinite(q[i]) || !std::isfinite(v[i]))valid&=~(1u<<slot);}
  if(wkc!=expected || (valid&controlled_mask)!=controlled_mask || (faults&controlled_mask)){advances=0;ready_sample=0;ready_ros=0;}
  if(advances<2){ready_sample=0;ready_ros=0;}
  else if(!ready_sample){ready_sample=sample;ready_ros=ros;}
 }
 bool fresh(int64_t ros,int64_t steady)const{
  return advances>=2 && expected>0 && wkc==expected && (valid&controlled_mask)==controlled_mask && !(faults&controlled_mask) &&
   progress>0 && steady>=progress && steady-progress<=ros_feedback_timeout_ns && ros>=source && ros-source+int64_t(age*1e9)<=ros_feedback_timeout_ns;
 }
};
} // namespace position_session

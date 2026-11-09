// Pure-memory dependency doubles; actual Traj_temp_outp is inserted by validator.
#include <array>
#include <cassert>
#include <iostream>
#include <vector>
constexpr int SINGLEARMDOF=7;
namespace KDL {struct JntArray {std::array<double,7> q{};int rows()const{return 7;}double operator()(int i)const{return q[i];}};}
struct ctlOutp {double posDes=0,velDes=0,grav_TorDes=0,interia_TorDes=0,feedforward_TorDes=0;bool ifReachEnd=false;};
struct outpWrap {std::vector<double>posDes,velDes,gravComp_TorDes,InteriaComp_TorDes,FeedForward_TorDes,KpDes,KdDes;std::vector<bool>ifComToEnd;};
struct ArmPD {std::vector<double> lArmKp,rArmKp,lArmKd,rArmKd;};
struct TorCompWrap {std::vector<double>lArm_Comp_grav,rArm_comp_grav,lArm_Comp_InteriaComb,rArm_Comp_InteriaComb,lArm_feedForward,rArm_feedForward;};
struct modCtl {struct{bool ifUseGravComp=true;int GravCompInpMod=1;}gravComp_ctl;int transMod=1;std::array<int,7>base_rigiditiCtl_mod_l{},base_rigiditiCtl_mod_r{};};
struct Track {unsigned calls=0;std::vector<std::vector<ctlOutp>> result;Track(){result.resize(2);for(auto& arm:result){arm.resize(7);for(unsigned i=0;i<7;++i){arm[i].posDes=.01*i;arm[i].velDes=.001*i;}}}std::vector<std::vector<ctlOutp>> solveResOutp(const std::vector<double>&,const std::vector<double>&){++calls;return result;}std::vector<std::vector<ctlOutp>> IKsolveResOutp(const std::vector<double>&a,const std::vector<double>&b){return solveResOutp(a,b);}};
struct PD {unsigned calls=0;ArmPD PDInteOutp(int mode,int rigidity){++calls;return {std::vector<double>(7,mode+rigidity+20.),std::vector<double>(7,mode+rigidity+30.),std::vector<double>(7,2.),std::vector<double>(7,3.)};}};
struct Gravity {unsigned calls=0;TorCompWrap GravCompOutp(const std::vector<double>&lq,const std::vector<double>&lv,const std::vector<double>&lt,const std::vector<double>&rq,const std::vector<double>&rv,const std::vector<double>&rt){++calls;TorCompWrap out;for(unsigned i=0;i<7;++i){out.lArm_Comp_grav.push_back(lq[i]+lt[i]);out.rArm_comp_grav.push_back(rq[i]+rt[i]);out.lArm_Comp_InteriaComb.push_back(lv[i]);out.rArm_Comp_InteriaComb.push_back(rv[i]);out.lArm_feedForward.push_back(.3);out.rArm_feedForward.push_back(.4);}return out;}
 TorCompWrap onlyGravComp(const std::vector<double>&a,const std::vector<double>&b,const std::vector<double>&c,const std::vector<double>&d){return GravCompOutp(a,b,std::vector<double>(7),c,d,std::vector<double>(7));}};
struct ArmCtlOptInterface {
 std::array<double,7>currentConstant_l{},currentConstant_r{};bool ifOutpUseGravComp=true;int GravCompInpChoose=1,outpMod=1;
 std::array<int,7>rigidityMod_l{},rigidityMod_r{};Track FrameSpace_track,JntSpace_track;PD mArmPDInte;Gravity GravComp;KDL::JntArray stPos_l,stPos_r;outpWrap lArmLst_outpWrap,rArmLst_outpWrap;
 ArmCtlOptInterface(){currentConstant_l.fill(2);currentConstant_r.fill(3);lArmLst_outpWrap.gravComp_TorDes.assign(7,0);lArmLst_outpWrap.InteriaComp_TorDes.assign(7,0);lArmLst_outpWrap.FeedForward_TorDes.assign(7,0);rArmLst_outpWrap=lArmLst_outpWrap;}
 std::vector<outpWrap> Traj_temp_outp(KDL::JntArray,KDL::JntArray,KDL::JntArray,KDL::JntArray,std::vector<double>,std::vector<double>
#ifdef TARGET_AUXILIARY
 ,const std::vector<outpWrap>* reference_only=nullptr,const modCtl* auxiliary_config=nullptr
#endif
 );
};
// PRODUCTION_FUNCTION
#ifdef TARGET_AUXILIARY
#include "position_session_contract.h"
#include "position_session_ros.h"
#include <memory>
#include <string>
#include <functional>
#include <type_traits>
namespace snowbot_ctl_msgs { namespace msg {
struct MidNodeCtlMsgSend {bool if_streaming_joint_position=true;int ctl_trans_mod=3;};
struct SessionMeta {
 uint64_t session=7,sequence=1,feedback_boot_id=3,feedback_sample=2,middle_instance_id=32;
 int64_t source_stamp_ns=1999000000,expires_at_ns=2399000000;
 uint8_t operation=1,reference_mode=2;
};
struct PositionSessionUpper {
 using SharedPtr=std::shared_ptr<PositionSessionUpper>;
 SessionMeta meta;MidNodeCtlMsgSend command;int trajectory=37;std::array<double,2> waist_position{.12,-.08};
};
}}
struct ArmCtlNode {
 struct Endpoint {
  std::string node_name()const{return "robot_upperMonitor";}
  std::string node_namespace()const{return "/";}
  std::string topic_type()const{return "snowbot_ctl_msgs/msg/PositionSessionUpper";}
 };
 bool source_known=true;
 position_session::CommandGraph position_command_graph_;
 std::vector<Endpoint> get_publishers_info_by_topic(const char*)const{return source_known?std::vector<Endpoint>{{}}:std::vector<Endpoint>{};}
 unsigned count_publishers(const char*)const{return 0;}
 position_session::Gate position_gate_;
 std::string position_rejection_,position_wait_reason_;
 bool position_dispatch_=false,ifInitTemp=false,checkTemp=false,ifReInit_get=true;
 unsigned ready_checks=0,native_dispatches=0;uint64_t position_instance_id_=32;
 int64_t position_nominal_bootstrap_expires_=0;
 uint64_t position_command_received_=0,position_command_session_=0,position_command_sequence_=0;
 int64_t position_command_expires_=0;
 bool feedback_ready=true;unsigned immediate_forwards=0;
 void position_log_interrupt(const char*){}
 void msg_publish(){assert(position_gate_.last().operation==2);++immediate_forwards;}
 std::vector<double> lArm_stPos_get,rArm_stPos_get;
 std::array<double,2> position_waist_target_{};
 snowbot_ctl_msgs::msg::SessionMeta position_meta_;
 std::function<void(snowbot_ctl_msgs::msg::PositionSessionUpper::SharedPtr)> session_input;
 std::function<void(std::shared_ptr<snowbot_ctl_msgs::msg::MidNodeCtlMsgSend>)> legacy_input;
 int position_sub_=0,upperMonitorMsg_get=0;
 template<class Message,class Callback>int create_subscription(const char*,int,Callback callback){
  if constexpr(std::is_same_v<Message,snowbot_ctl_msgs::msg::PositionSessionUpper>)session_input=callback;
  else legacy_input=callback;
  return 0;
 }
 ArmCtlNode(){position_meta_.session=0;
  // PRODUCTION_MIDDLE_INPUTS
 }
 int position_trajectory_=0;
 snowbot_ctl_msgs::msg::PositionSessionUpper::SharedPtr position_target_packet_;
 struct Feedback {
  uint64_t boot=3,ready_sample=1,sample=2;int64_t ready_ros=1900000000;
  std::array<double,16> q{};
  bool fresh(int64_t,int64_t)const{return true;}
 } position_feedback_;
 struct Clock {int64_t nanoseconds()const{return 2000000000;}};
 Clock now()const{return {};}
 bool position_ready(){++ready_checks;return feedback_ready;}
 void upperMonitor_msg_callback(const std::shared_ptr<snowbot_ctl_msgs::msg::MidNodeCtlMsgSend>&){assert(position_dispatch_);++native_dispatches;}
 void position_callback(const snowbot_ctl_msgs::msg::PositionSessionUpper::SharedPtr packet);
};
// PRODUCTION_MIDDLE_CALLBACK
void check_middle_session(){
 using Packet=snowbot_ctl_msgs::msg::PositionSessionUpper;
 ArmCtlNode startup;
 assert(startup.session_input && !startup.legacy_input && startup.native_dispatches==0);
 startup.session_input(std::make_shared<Packet>());
 assert(startup.native_dispatches==1 && startup.ifReInit_get);
 ArmCtlNode discovering;discovering.source_known=false;
 discovering.position_callback(std::make_shared<Packet>());
 assert(discovering.position_rejection_.empty()&&!discovering.position_gate_.selected()&&discovering.native_dispatches==0);
 for(uint8_t mode:{0,1}){
  ArmCtlNode node;auto packet=std::make_shared<Packet>();packet->meta.reference_mode=mode;
  node.position_callback(packet);
  assert(node.position_rejection_=="reference_mode_conflict");
  assert(!node.position_gate_.selected() && node.ready_checks==0 && node.native_dispatches==0);
 }
 ArmCtlNode node;auto packet=std::make_shared<Packet>();
 node.position_callback(packet);
 assert(node.position_rejection_.empty() && node.native_dispatches==1 && node.ifReInit_get);
 assert(node.position_meta_.reference_mode==2 && node.position_trajectory_==37);
 assert(node.position_nominal_bootstrap_expires_==packet->meta.expires_at_ns);
 assert(node.position_waist_target_==packet->waist_position);
 packet=std::make_shared<Packet>(*packet);packet->meta.operation=0;++packet->meta.sequence;
 node.position_callback(packet);
 assert(node.position_rejection_.empty() && node.native_dispatches==2 && !node.ifReInit_get);
 packet=std::make_shared<Packet>(*packet);packet->meta.operation=2;++packet->meta.sequence;
 packet->meta.expires_at_ns=packet->meta.source_stamp_ns+2000000000;
 node.position_feedback_.q.fill(.5);node.position_callback(packet);
 assert(node.position_rejection_.empty() && node.native_dispatches==3 && !node.ifReInit_get);
 assert(node.immediate_forwards==1);
 assert(node.position_gate_.last().expires==packet->meta.expires_at_ns);
 assert(node.position_waist_target_==packet->waist_position);
 assert(node.lArm_stPos_get.front()==0. && node.rArm_stPos_get.front()==0.);
 ArmCtlNode expired;packet=std::make_shared<Packet>();packet->meta.expires_at_ns=1999999999;
 expired.position_callback(packet);
 assert(!expired.position_gate_.selected() && expired.native_dispatches==0);
 for(bool stale:{false,true}){
  ArmCtlNode blocked;blocked.position_callback(std::make_shared<Packet>());
  auto pause=std::make_shared<Packet>();pause->meta.operation=2;++pause->meta.sequence;
  if(stale)blocked.feedback_ready=false;
  else pause->meta.expires_at_ns=1999999999;
  blocked.position_callback(pause);
  assert(blocked.immediate_forwards==0 && blocked.native_dispatches==1);
 }
 std::cerr<<"PASS actual middle session callback rejects retired modes and forwards v3 takeover/target/PAUSE with original deadlines and waist targets\n";
}
#endif
void equal(const std::vector<outpWrap>& a,const std::vector<outpWrap>& b){for(unsigned arm=0;arm<2;++arm){assert(a[arm].posDes==b[arm].posDes);assert(a[arm].velDes==b[arm].velDes);assert(a[arm].KpDes==b[arm].KpDes);assert(a[arm].KdDes==b[arm].KdDes);assert(a[arm].gravComp_TorDes==b[arm].gravComp_TorDes);assert(a[arm].InteriaComp_TorDes==b[arm].InteriaComp_TorDes);assert(a[arm].FeedForward_TorDes==b[arm].FeedForward_TorDes);}}
int main(){
 KDL::JntArray pos,vel;for(unsigned i=0;i<7;++i){pos.q[i]=.02*i;vel.q[i]=.003*i;}
 for(int mode=0;mode<=5;++mode)for(int gravity=1;gravity<=3;++gravity){ArmCtlOptInterface node;node.outpMod=mode;node.GravCompInpChoose=gravity;auto result=node.Traj_temp_outp(pos,pos,vel,vel,std::vector<double>(7,.1),std::vector<double>(7,.2));
  std::cout<<mode<<' '<<gravity<<' '<<node.JntSpace_track.calls<<' '<<node.FrameSpace_track.calls<<' '<<node.mArmPDInte.calls<<' '<<node.GravComp.calls;
  for(const auto& arm:result)for(const auto*values:{&arm.posDes,&arm.velDes,&arm.KpDes,&arm.KdDes,&arm.gravComp_TorDes,&arm.InteriaComp_TorDes,&arm.FeedForward_TorDes})for(double value:*values)std::cout<<' '<<value;
  std::cout<<'\n';
 }
#ifdef TARGET_AUXILIARY
 check_middle_session();
 for(int gravity=1;gravity<=3;++gravity){ArmCtlOptInterface legacy,target;legacy.outpMod=1;legacy.GravCompInpChoose=gravity;auto expected=legacy.Traj_temp_outp(pos,pos,vel,vel,std::vector<double>(7,.1),std::vector<double>(7,.2));
  std::vector<outpWrap> reference(2);for(auto& arm:reference)for(unsigned i=0;i<7;++i){arm.posDes.push_back(.01*i);arm.velDes.push_back(.001*i);}
  modCtl config;config.gravComp_ctl.GravCompInpMod=gravity;
  auto actual=target.Traj_temp_outp(pos,pos,vel,vel,std::vector<double>(7,.1),std::vector<double>(7,.2),&reference,&config);
  equal(expected,actual);assert(target.FrameSpace_track.calls==0&&target.JntSpace_track.calls==0&&target.mArmPDInte.calls==1&&target.GravComp.calls==1);
 }
 std::cerr<<"PASS actual middle Traj_temp_outp target reference bypasses both interpolators and preserves PD/gravity wrapper calculations; default outputs available for bytewise differential\n";
#endif
}

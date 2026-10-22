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
void equal(const std::vector<outpWrap>& a,const std::vector<outpWrap>& b){for(unsigned arm=0;arm<2;++arm){assert(a[arm].posDes==b[arm].posDes);assert(a[arm].velDes==b[arm].velDes);assert(a[arm].KpDes==b[arm].KpDes);assert(a[arm].KdDes==b[arm].KdDes);assert(a[arm].gravComp_TorDes==b[arm].gravComp_TorDes);assert(a[arm].InteriaComp_TorDes==b[arm].InteriaComp_TorDes);assert(a[arm].FeedForward_TorDes==b[arm].FeedForward_TorDes);}}
int main(){
 KDL::JntArray pos,vel;for(unsigned i=0;i<7;++i){pos.q[i]=.02*i;vel.q[i]=.003*i;}
 for(int mode=0;mode<=5;++mode)for(int gravity=1;gravity<=3;++gravity){ArmCtlOptInterface node;node.outpMod=mode;node.GravCompInpChoose=gravity;auto result=node.Traj_temp_outp(pos,pos,vel,vel,std::vector<double>(7,.1),std::vector<double>(7,.2));
  std::cout<<mode<<' '<<gravity<<' '<<node.JntSpace_track.calls<<' '<<node.FrameSpace_track.calls<<' '<<node.mArmPDInte.calls<<' '<<node.GravComp.calls;
  for(const auto& arm:result)for(const auto*values:{&arm.posDes,&arm.velDes,&arm.KpDes,&arm.KdDes,&arm.gravComp_TorDes,&arm.InteriaComp_TorDes,&arm.FeedForward_TorDes})for(double value:*values)std::cout<<' '<<value;
  std::cout<<'\n';
 }
#ifdef TARGET_AUXILIARY
 for(int gravity=1;gravity<=3;++gravity){ArmCtlOptInterface legacy,target;legacy.outpMod=1;legacy.GravCompInpChoose=gravity;auto expected=legacy.Traj_temp_outp(pos,pos,vel,vel,std::vector<double>(7,.1),std::vector<double>(7,.2));
  std::vector<outpWrap> reference(2);for(auto& arm:reference)for(unsigned i=0;i<7;++i){arm.posDes.push_back(.01*i);arm.velDes.push_back(.001*i);}
  modCtl config;config.gravComp_ctl.GravCompInpMod=gravity;
  auto actual=target.Traj_temp_outp(pos,pos,vel,vel,std::vector<double>(7,.1),std::vector<double>(7,.2),&reference,&config);
  equal(expected,actual);assert(target.FrameSpace_track.calls==0&&target.JntSpace_track.calls==0&&target.mArmPDInte.calls==1&&target.GravComp.calls==1);
 }
 std::cerr<<"PASS actual middle Traj_temp_outp target reference bypasses both interpolators and preserves PD/gravity wrapper calculations; default outputs available for bytewise differential\n";
#endif
}

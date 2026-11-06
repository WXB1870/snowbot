
#include "impedance/cartesian_impedance_controller.h"
#include <cassert>
#include <filesystem>
#include <iostream>
using namespace snowbot_impedance;
void check_joint(const std::string& side,const char* urdf){
 const auto yaml=std::filesystem::path(urdf).parent_path().parent_path().parent_path()/"ctl_msgs/config/impedance_params.yaml";
 const auto imported=ImpedanceConfig::fromYaml(yaml.string(),side,urdf);
 assert(imported.mode==ImpedanceMode::JOINT&&imported.joint_stiffness[0]==1200.&&imported.joint_stiffness[6]==400.);
 assert(imported.joint_damping[0]==120.&&imported.joint_damping[6]==40.);
 auto node=std::make_shared<rclcpp::Node>();auto cfg=ImpedanceConfig::makeDefault(side,urdf);
 cfg.session_reference_only=true;cfg.mode=ImpedanceMode::JOINT;
 cfg.filter_params_pose=cfg.filter_params_nullspace=1.;cfg.cmd_lpf_alpha=1.;
 cfg.kt_gravity.assign(7,0.);cfg.lugre_friction.enable.assign(7,0.);
 cfg.joint_stiffness.assign(7,20.);cfg.joint_damping.assign(7,2.);
 cfg.joint_vel_filter_coeff.assign(7,30.);cfg.effort_limit_per_joint.assign(7,1.);
 CartesianImpedanceController ctl;assert(ctl.init(node,cfg));assert(node->subscriptions==0);
 VectorXd q=VectorXd::Zero(ctl.getNqFull()),dq=q,zero=q;Vector6d wrench=Vector6d::Zero();
 const auto index=ctl.getControlledQIdx()[0];q[index]=.1;
 ctl.enable(q,dq,wrench);assert(ctl.setNominalReference(q,dq));ctl.update(q,dq,zero,wrench,.002);
 assert(ctl.getEffortCommands().norm()<1e-12);
 auto target=q;target[index]+=.01;
 for(int tick=1;tick<=2;++tick){
  assert(ctl.setNominalReference(target,zero));ctl.update(q,dq,zero,wrench,.002);
  const double expected=20.*.01+2.*.01*30.*std::exp(-30.*.002*tick);
  assert(std::abs(ctl.getEffortCommands()[0]-expected)<1e-10);
  assert(std::abs(ctl.getQDNullspaceFiltered()[0]-target[index])<1e-12);
 }
 q[index]+=.005;dq[ctl.getControlledVIdx()[0]]=.02;
 assert(ctl.setNominalReference(target,zero));ctl.update(q,dq,zero,wrench,.002);
 const double expected=20.*.005+2.*(.01*30.*std::exp(-30.*.006)-.02);
 assert(std::abs(ctl.getEffortCommands()[0]-expected)<1e-10);
 dq.setZero();ctl.enable(q,dq,wrench);assert(ctl.setNominalReference(q,dq));ctl.update(q,dq,zero,wrench,.002);
 assert(ctl.getEffortCommands().norm()<1e-12);
 for(double sign:{-1.,1.}){
  ctl.enable(q,dq,wrench);target=q;target[index]+=sign*.2;
  assert(ctl.setNominalReference(target,zero));ctl.update(q,dq,zero,wrench,.002);
  assert(std::abs(ctl.getEffortCommands()[0]-sign)<1e-12);
 }
 std::cout<<side<<": JOINT nominal reference, filtered velocity, measured damping, enable reset and torque limits passed\n";
}
int main(int argc,char**argv){assert(argc==2);for(const std::string side:{"left","right"}){
 check_joint(side,argv[1]);
 auto node=std::make_shared<rclcpp::Node>();auto cfg=ImpedanceConfig::makeDefault(side,argv[1]);cfg.session_reference_only=true;cfg.filter_params_pose=cfg.filter_params_nullspace=1.;cfg.tcp_to_ee={0,0,.192,0,0,0,1};
 CartesianImpedanceController ctl;assert(ctl.init(node,cfg));assert(node->subscriptions==0);
 VectorXd q=VectorXd::Zero(ctl.getNqFull()),dq=q,zero=q;Vector6d wrench=Vector6d::Zero();
 q[ctl.getJointQIdx("body_joint_1")]=.12;q[ctl.getJointQIdx("body_joint_2")]=-.08;
 dq[ctl.getJointVIdx("body_joint_1")]=.02;dq[ctl.getJointVIdx("body_joint_2")]=-.03;
 for(auto i:ctl.getControlledQIdx())q[i]=.1;
 ctl.enable(q,dq,wrench);assert(ctl.setNominalReference(q,dq));ctl.update(q,dq,zero,wrench,.001);
 assert(ctl.getActualTwist().norm()>1e-5);assert(ctl.getPoseDeviation().norm()<1e-10);assert(ctl.getTwistDeviation().norm()<1e-10);
 const auto nominal_pose=ctl.getTcpPoseInBase();auto measured=q;measured[ctl.getControlledQIdx()[0]]+=.03;
 for(int tick=0;tick<100;++tick){assert(ctl.setNominalReference(q,zero));ctl.update(measured,zero,zero,wrench,.001);assert(ctl.getPoseDeviation().norm()>1e-4);assert(ctl.getEffortCommands().allFinite());}
 for(unsigned i=0;i<7;++i)assert(std::abs(ctl.getQDNullspaceFiltered()[i]-q[ctl.getControlledQIdx()[i]])<1e-12);
 assert((ctl.getTcpPoseInBase().translation()-ctl.getPoseDeviation().head<3>()-nominal_pose.translation()).norm()<1e-10);
 auto invalid=q;invalid[0]=NAN;assert(!ctl.setNominalReference(invalid,zero));assert(!ctl.setNominalReference(VectorXd::Zero(1),zero));
 std::cout<<side<<": nominal FK/TCP/nullspace, waist twist, fixed reference under displacement and invalid reference passed\n";
}}

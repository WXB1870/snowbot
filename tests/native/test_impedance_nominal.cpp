
#include "impedance/cartesian_impedance_controller.h"
#include <cassert>
#include <iostream>
using namespace snowbot_impedance;
int main(int argc,char**argv){assert(argc==2);for(const std::string side:{"left","right"}){
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

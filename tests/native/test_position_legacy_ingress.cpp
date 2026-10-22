
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cassert>
#include <iostream>
constexpr int LARMDOF=7,RARMDOF=7;
struct Time { double t; double seconds()const{return t;} };Time operator-(Time a,Time b){return {a.t-b.t};}
#define RCLCPP_WARN(logger, format, age) ((void)(age))
int enable_writes=0;
void EtherCAT_setSingleMotorEnable(int,bool){++enable_writes;}
void EtherCAT_setMotorEnable(bool){++enable_writes;}
namespace snowbot_ctl_msgs { namespace msg {
struct CtlInformation {using SharedPtr=std::shared_ptr<CtlInformation>;struct {Time stamp;} header;
 double temp_time=0;int runin_mod=0,if_topic_point_position=0,if_runin_renew=0,if_use_debugmod=0,if_useret_to_start_runin=0,motor_enable=0;
 std::array<double,7> base_rigidity_ctl_mode_l{};
std::array<double,7> base_rigidity_ctl_mode_r{};
std::array<double,7> l_kd{};
std::array<double,7> l_kp{};
std::array<double,7> l_motor_enable{};
std::array<double,7> l_position{};
std::array<double,7> l_position_at_runin_st{};
std::array<double,7> l_speed{};
std::array<double,7> l_torque_feedforward{};
std::array<double,7> l_torque_grav_comp{};
std::array<double,7> l_torque_interia_comp{};
std::array<double,7> r_kd{};
std::array<double,7> r_kp{};
std::array<double,7> r_motor_enable{};
std::array<double,7> r_position{};
std::array<double,7> r_position_at_runin_st{};
std::array<double,7> r_speed{};
std::array<double,7> r_torque_feedforward{};
std::array<double,7> r_torque_grav_comp{};
std::array<double,7> r_torque_interia_comp{};
};}}
struct ServoNode {
 int position_reference_mode_=2; bool shutdown_mode_=false; std::string position_rejection_;
 struct Gate {bool chosen=false;bool selected()const{return chosen;}} position_gate_;
 double tempTime_upperMonitor=-99;
 int runin_mod=-99,if_topic_point_position=-99,if_runin_renew=-99,msg_get_ifUseDebugMod=-99,if_useret_to_start_runin=-99;
 bool ifRunin=false,has_received_command_=false;
 std::array<double,7> lArm_Kd_receive{};
std::array<double,7> lArm_Kp_receive{};
std::array<double,7> lArm_pos_receive{};
std::array<double,7> lArm_torComp_feedforward_receive{};
std::array<double,7> lArm_torComp_grav_receive{};
std::array<double,7> lArm_torComp_interia_receive{};
std::array<double,7> lArm_transMod{};
std::array<double,7> lArm_vel_receive{};
std::array<double,7> l_posReceive_atRuninSt{};
std::array<double,7> rArm_Kd_receive{};
std::array<double,7> rArm_Kp_receive{};
std::array<double,7> rArm_pos_receive{};
std::array<double,7> rArm_torComp_feedforward_receive{};
std::array<double,7> rArm_torComp_grav_receive{};
std::array<double,7> rArm_torComp_interia_receive{};
std::array<double,7> rArm_transMod{};
std::array<double,7> rArm_vel_receive{};
std::array<double,7> r_posReceive_atRuninSt{};
 Time now(){return {100.};} int get_logger(){return 0;}
 void message_callback(const snowbot_ctl_msgs::msg::CtlInformation::SharedPtr msg);
};
// PRODUCTION_LEGACY_CALLBACK
int main(){
 auto m=std::make_shared<snowbot_ctl_msgs::msg::CtlInformation>();m->motor_enable=1;m->temp_time=.02;
 m->base_rigidity_ctl_mode_l.fill(1.25);
m->base_rigidity_ctl_mode_r.fill(1.25);
m->l_kd.fill(1.25);
m->l_kp.fill(1.25);
m->l_motor_enable.fill(1.25);
m->l_position.fill(1.25);
m->l_position_at_runin_st.fill(1.25);
m->l_speed.fill(1.25);
m->l_torque_feedforward.fill(1.25);
m->l_torque_grav_comp.fill(1.25);
m->l_torque_interia_comp.fill(1.25);
m->r_kd.fill(1.25);
m->r_kp.fill(1.25);
m->r_motor_enable.fill(1.25);
m->r_position.fill(1.25);
m->r_position_at_runin_st.fill(1.25);
m->r_speed.fill(1.25);
m->r_torque_feedforward.fill(1.25);
m->r_torque_grav_comp.fill(1.25);
m->r_torque_interia_comp.fill(1.25);
 ServoNode s;
 // Fresh and 5.29-second queued legacy packets, before/after session selection.
 for(bool selected:{false,true})for(double age:{0.,.001,5.288261}){
  s.position_gate_.chosen=selected;m->header.stamp={100.-age};
  for(int i=0;i<400;i++)s.message_callback(m);
 }
#ifdef OLD_CODE
 assert(enable_writes>0 && s.has_received_command_ && s.ifRunin);
 std::cout<<"old callback reproduces queued legacy enable/target writes\n";
#else
 assert(enable_writes==0 && !s.has_received_command_ && !s.ifRunin);
 assert(s.tempTime_upperMonitor==-99 && s.runin_mod==-99);
 for(auto x:s.lArm_Kd_receive)assert(x==0.);
for(auto x:s.lArm_Kp_receive)assert(x==0.);
for(auto x:s.lArm_pos_receive)assert(x==0.);
for(auto x:s.lArm_torComp_feedforward_receive)assert(x==0.);
for(auto x:s.lArm_torComp_grav_receive)assert(x==0.);
for(auto x:s.lArm_torComp_interia_receive)assert(x==0.);
for(auto x:s.lArm_transMod)assert(x==0.);
for(auto x:s.lArm_vel_receive)assert(x==0.);
for(auto x:s.l_posReceive_atRuninSt)assert(x==0.);
for(auto x:s.rArm_Kd_receive)assert(x==0.);
for(auto x:s.rArm_Kp_receive)assert(x==0.);
for(auto x:s.rArm_pos_receive)assert(x==0.);
for(auto x:s.rArm_torComp_feedforward_receive)assert(x==0.);
for(auto x:s.rArm_torComp_grav_receive)assert(x==0.);
for(auto x:s.rArm_torComp_interia_receive)assert(x==0.);
for(auto x:s.rArm_transMod)assert(x==0.);
for(auto x:s.rArm_vel_receive)assert(x==0.);
for(auto x:s.r_posReceive_atRuninSt)assert(x==0.);
 assert(s.position_rejection_=="legacy_source_disabled_for_target_stream");
 // Unsupported old channels stay isolated even during shutdown; internal homing is separate.
 s.shutdown_mode_=true;s.message_callback(m);assert(enable_writes==0);
 // Non-target-stream mode's existing behavior is preserved.
 ServoNode legacy;legacy.position_reference_mode_=0;m->header.stamp={100.};
 legacy.message_callback(m);legacy.message_callback(m);assert(enable_writes>0 && legacy.ifRunin);
 std::cout<<"new callback: 2400 legacy packets blocked without enable/target/gain/interpolation writes; other mode preserved\n";
#endif
}

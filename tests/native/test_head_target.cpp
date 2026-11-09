// Actual prepared callbacks with memory-only feedback and command sinks.
#include "position_head.h"
#include <array>
#include <cassert>
#include <memory>
#include <string>
#include <iostream>
constexpr int HEAD_MOTORS=2;
double ros_now=100.;
std::array<double,2> measured{{.2,-.1}}, output=measured;
int enables=0;
bool reset_busy=false;
bool EtherCAT_driveResetBusy(){return reset_busy;}
void EtherCAT_setMotorEnable(bool){++enables;}
void EtherCAT_setSingleMotorEnable(int,bool){++enables;}
void getMotorFb_head(int i,float& p,float& v,float& c,uint16_t& s){p=measured[i];v=c=0;s=0x27;}
void setHeadMotorCmd(int i,double p,double,double,double,double){output[i]=p;}
struct Observation {unsigned valid_mask=0xfffff;int wkc=67,expected_wkc=67;
 int64_t sampled_ns=position_session::HeadTarget::steady();std::array<unsigned,20> status{};};
Observation obs;
Observation EtherCAT_positionObservation(){obs.sampled_ns=position_session::HeadTarget::steady();return obs;}
#define RCLCPP_INFO_THROTTLE(...) ((void)0)
namespace std_msgs {namespace msg {
struct Float64MultiArray {using SharedPtr=std::shared_ptr<Float64MultiArray>;std::vector<double> data;};}}
struct Writer {std::string node_name()const{return "pico_control";}
 std::string topic_type()const{return "std_msgs/msg/Float64MultiArray";}};
struct Time {double seconds()const{return ros_now;}};
struct ServoNode {
 bool position_startup_complete_=true,shutdown_mode_=false,h_inited=false,h_ifRunin=false;
 int h_step=0,h_winlen=0;double h_time_last=0.;
 double h_pos_curr[2]{},h_pos_start[2]{},h_pos_target[2]{};
 position_session::HeadTarget head_target_;
 bool conflict=false;
 int count_publishers(const char*)const{return 0;}
 Time now()const{return {};}
 std::vector<Writer> get_publishers_info_by_topic(const char*){return conflict?std::vector<Writer>(2):std::vector<Writer>(1);}
 void head_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
 void head_runin();
};
// PRODUCTION_HEAD_METHODS
int main(){
 using position_session::HeadTarget;
 HeadTarget gate;
 std::vector<double> packet{.21,-.09,100.,.25,100.,0.};
 assert(gate.accept(packet,100.,1000000000));
 assert(!gate.accept(packet,100.01,1010000000)); // replay
 packet[4]=100.01;packet[0]=.792;assert(!gate.accept(packet,100.01,1010000000));
 packet[0]=.21;packet[2]=101.;assert(!gate.accept(packet,100.01,1010000000));
 packet[2]=100.;assert(!gate.accept(packet,100.25,1250000000));
 assert(!gate.expired(100.1,1100000000));
 assert(gate.expired(100.25,1100000000));
 assert(gate.expired(100.1,1250000000)); // steady expiry despite ROS lag
 assert(gate.expired(99.,1100000000));
 assert(gate.expired(100.1,999999999));
 auto msg=std::make_shared<std_msgs::msg::Float64MultiArray>();
 msg->data={.21,-.09,100.,.25,100.,0.};
 ServoNode s;s.head_cmd_callback(msg);
 assert(s.h_inited && s.h_ifRunin && enables==3);
 s.head_runin();const auto before=output;
 // Expiry freezes the native reference instead of completing the old goal.
 ros_now=100.251;s.head_runin();
 assert(!s.h_ifRunin && output==before);
 msg->data={0.,0.};s.head_cmd_callback(msg);assert(enables==3); // retired legacy
 ros_now=100.26;msg->data={.22,-.08,100.26,.25,100.26,0.};s.head_cmd_callback(msg);
 assert(s.h_ifRunin);s.head_runin();const auto moving=output;
 ros_now=100.261;msg->data={.24,-.08,100.261,.25,100.261,0.};s.head_cmd_callback(msg);
 double distance=0.;for(int i=0;i<2;++i)distance=std::max(distance,std::abs(s.h_pos_target[i]-s.h_pos_start[i]));
 assert(1.5*distance/(s.h_winlen*.001)<=HeadTarget::max_speed+1e-9);
 ros_now=100.27;msg->data={0.,0.,100.27,.25,100.27,1.};s.head_cmd_callback(msg);s.head_runin();
 assert(!s.h_ifRunin && output==moving);
 const auto enable_count=enables;
 ros_now=100.28;msg->data={.22,-.08,100.28,.25,100.28,0.};
 s.conflict=true;s.head_cmd_callback(msg);assert(enables==enable_count);
 s.conflict=false;obs.status[16]=8;s.head_cmd_callback(msg);assert(enables==enable_count);
 reset_busy=true;msg->data[4]=100.28;s.head_cmd_callback(msg);assert(enables==enable_count);
 std::cout<<"head targets: native expiry, stop, replay, limits, source conflict and drive fault passed\n";
}

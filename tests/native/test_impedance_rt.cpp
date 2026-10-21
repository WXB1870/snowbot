#include "servo_control/position_watchdog.h"
#include <cassert>
#include <array>
#include <iostream>
using namespace position_session;
void sensors(){
 Observation obs;bool complete=true;
 struct Sensor {float Fx=1,Fy=2,Fz=3,Mx=4,My=5,Mz=6;};
 Sensor left,right;std::array<Sensor*,2>s_tx_{&left,&right};
 auto sample=[&](){
 // SENSOR_CAPTURE
 };
 sample();assert(obs.sensor_valid_mask==3&&obs.wrench[0]==1&&obs.wrench[11]==6);
 s_tx_[1]=nullptr;sample();assert(obs.sensor_valid_mask==1);
 s_tx_[1]=&right;right.Fx=NAN;sample();assert(obs.sensor_valid_mask==1);
 right.Fx=1;complete=false;sample();assert(obs.sensor_valid_mask==0);
}
int main(){
 sensors();constexpr int EN_STEP=5;
 struct PT {float target_position=99,target_speed=99,KP=99,KD=99,target_torque=99;};
 struct CSP {int32_t position=99,velocity=99;int16_t torque=99;};
 std::array<PT,20> pt;std::array<CSP,20> csp;
 std::array<PT*,20> m_rx_{};std::array<CSP*,20> csp_rx_{};std::array<bool,20>is_csp_motor_{};
 std::array<int,20>enable_step;enable_step.fill(EN_STEP*6);
 for(unsigned m=0;m<20;++m){if(m>=16){is_csp_motor_[m]=true;csp_rx_[m]=&csp[m];}else m_rx_[m]=&pt[m];}
 Output session_packet;session_packet.selected=true;session_packet.impedance=true;
 session_packet.session=10;session_packet.sequence=1;session_packet.operation=1;session_packet.policy=2;session_packet.deadline=1200000000;
 session_packet.q.fill(.1f);session_packet.kp.fill(1);session_packet.kd.fill(.2f);session_packet.torque.fill(3);
 session_packet.hold_kp.fill(40);session_packet.hold_kd.fill(2);session_packet.hold_torque.fill(4);
 Observation obs;obs.sample=1;obs.sampled_ns=obs.attempted_ns=1000000000;obs.wkc=obs.expected_wkc=67;obs.valid_mask=0xfffff;obs.sensor_valid_mask=3;obs.status.fill(0x27);obs.position.fill(.2f);
 Watchdog session_watchdog_;session_watchdog_.evaluate(session_packet,obs,1000000000);
 auto write=[&](){
 // FINAL_PDO_WRITE
 return position_written;
 };
 assert(write()==controlled_mask);assert(pt[0].KP==1&&pt[0].KD==.2f&&pt[0].target_torque==3);assert(csp[18].position==csp_counts(.1f));
 assert(pt[7].KP==99&&pt[15].KP==99&&csp[16].position==99&&csp[17].position==99);
 obs.sampled_ns=1200000000;session_watchdog_.evaluate(session_packet,obs,1200000000);assert(!session_watchdog_.disabled);
 assert(write()==controlled_mask);assert(pt[0].KP==40&&pt[0].KD==2&&pt[0].target_torque==4&&pt[0].target_position==.2f);
 // RT notices missing force samples without waiting for a ROS callback.
 ++session_packet.sequence;session_packet.deadline=1400000000;obs.sensor_valid_mask=1;
 session_watchdog_.evaluate(session_packet,obs,1200000000);assert(session_watchdog_.disabled&&session_watchdog_.reason==IMPEDANCE_INVALID);assert(write()==0);
 std::cout<<"PASS actual sensor PDO capture and final PT/CSP writes: validity, torque precedence, RT fallback and resource isolation\n";
}

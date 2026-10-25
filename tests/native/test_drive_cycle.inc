#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include "driver_test_access.h"
#define EN_STEP 5
// @ACTUAL_CYCLE@
// @ACTUAL_DIAGNOSTICS@
EtherCATDriver::EtherCATDriver(){
 std::fill_n(m_rx_,TOTAL_SLAVES,nullptr);std::fill_n(m_tx_,TOTAL_SLAVES,nullptr);
 std::fill_n(csp_rx_,TOTAL_SLAVES,nullptr);std::fill_n(csp_tx_,TOTAL_SLAVES,nullptr);
 inOP_=true;cycle_count_=0;
}
EtherCATDriver::~EtherCATDriver(){}
// Run the actual pre-send copy and post-receive capture with memory PDOs.
// Deliberately change outgoing buffers after capture: a reconstructed command
// or a later buffer read must not masquerade as the pre-send sample.
void check_csp_snapshot(){
 std::array<csp_rx_t,4> rx{};std::array<csp_tx_t,4> tx{};
 std::array<csp_rx_t*,20> csp_rx_{};std::array<csp_tx_t*,20> csp_tx_{};
 std::array<motor_tx_pdo_t*,20> m_tx_{};std::array<bool,20> is_csp_motor_{};
 position_session::Observation observation_rt_;std::atomic<int> wkc_{67};int expectedWKC_=67;
 uint64_t timing_over_1ms_=0;
 for(unsigned i=0;i<4;++i){unsigned m=16+i;is_csp_motor_[m]=true;csp_rx_[m]=&rx[i];csp_tx_[m]=&tx[i];
  rx[i].position=2147483647-int(i);rx[i].velocity=-2147483647+int(i);rx[i].torque=-32768;
  rx[i].controlword=0x1f;rx[i].ModeOp=8;
  tx[i].position=-2147483647+int(i);tx[i].velocity=2147483647-int(i);tx[i].ModeOp=i==2?9:8;
  tx[i].statusword=0x1237;tx[i].torque=32767;
 }
 auto collect=[&]{
  const auto io_start=std::chrono::steady_clock::now();
  // @CSP_CAPTURE@
  for(auto& value:rx)--value.position; // Not the values that were captured.
  const auto io_end=std::chrono::steady_clock::now();
  // @CSP_RECEIVE@
 };
 collect();const auto first=observation_rt_;
 assert(first.csp.command_mask==0xf0000&&first.csp.feedback_mask==0xf0000);
 assert(first.csp.send_attempt_ns>0&&first.csp.send_attempt_ns<=first.attempted_ns);
 assert(first.csp.target_position[2]==2147483645&&first.csp.target_position[2]!=rx[2].position);
 assert(first.csp.target_velocity[3]==-2147483644&&first.csp.target_torque[2]==-32768);
 assert(first.csp.controlword[2]==0x1f&&first.csp.mode_command[2]==8&&first.csp.mode_display[2]==9);
 assert(first.csp.actual_position[2]==tx[2].position&&first.csp.actual_velocity[2]==tx[2].velocity);
 assert(first.torque_raw[18]==32767&&first.status[18]==0x1237);
 // Bad WKC never validates stale incoming mode/encoder data, but still reports
 // the distinct outgoing attempt. No synthetic mode 8 or zero feedback.
 wkc_=66;tx[2].ModeOp=-1;collect();
 assert(observation_rt_.sample==first.sample&&observation_rt_.attempt==first.attempt+1);
 assert(observation_rt_.csp.command_mask==0xf0000&&observation_rt_.csp.feedback_mask==0);
 assert(observation_rt_.valid_mask==0&&observation_rt_.csp.mode_display[2]==-1);
 // A missing PDO pointer and a non-CSP slot must not inherit old availability.
 wkc_=67;csp_rx_[18]=nullptr;csp_tx_[19]=nullptr;is_csp_motor_[16]=false;collect();
 assert(observation_rt_.csp.command_mask==0xa0000&&observation_rt_.csp.feedback_mask==0x60000);
 assert(observation_rt_.csp.mode_command[2]==0&&observation_rt_.csp.mode_display[3]==0);
 assert(first.csp.mode_display[2]==9); // Previous copies remain detached.
}
int main(){
 check_csp_snapshot();
 EtherCATDriver d;
 std::array<motor_rx_pdo_t,16> rx{};std::array<motor_tx_pdo_t,16> tx{};
 std::array<csp_rx_t,4> crx{};std::array<csp_tx_t,4> ctx{};
 for(unsigned i=0;i<20;++i){
  if(i<16){d.m_rx_[i]=&rx[i];d.m_tx_[i]=&tx[i];rx[i].KP=51;rx[i].KD=3;rx[i].target_position=.21;rx[i].target_speed=.03;rx[i].target_torque=2;tx[i].statusword=0x218;}
  else{unsigned k=i-16;d.is_csp_motor_[i]=true;d.csp_rx_[i]=&crx[k];d.csp_tx_[i]=&ctx[k];crx[k].position=123;crx[k].velocity=4;crx[k].torque=5;crx[k].ModeOp=8;ctx[k].statusword=0x1218;}
 }
 auto before=rx;auto cbefore=crx;auto& obs=d.observation_rt_;
 obs.valid_mask=0xfffff;obs.wkc=obs.expected_wkc=67;obs.status.fill(0x218);obs.sample=1;
 obs.status[0]=0x231; // Healthy axis must never receive a reset pulse.
 assert(d.requestDriveReset());unsigned pulses=0;
 for(unsigned n=0;n<90;++n){
  obs.sample++;obs.sampled_ns=position_session::steady_ns();
  if(n>35)obs.status.fill(0x40);
  d.cycle();
  if(rx[1].controlword==0x80)++pulses;
  for(unsigned i=0;i<20;++i){
   if(i<16){auto got=rx[i];assert(!(got.controlword&9));assert(got.controlword==0||got.controlword==0x80||got.controlword==6);got.controlword=before[i].controlword;assert(std::memcmp(&got,&before[i],sizeof(got))==0);}
   else{auto got=crx[i-16];assert(!(got.controlword&9));got.controlword=cbefore[i-16].controlword;assert(std::memcmp(&got,&cbefore[i-16],sizeof(got))==0);}
   assert(!d.motor_enable_mask_[i]);
  }
  assert(rx[0].controlword!=0x80);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }
 assert(pulses>0&&pulses<=10&&d.drive_reset_.state()==drive_maintenance::cleared);
 obs.csp.command_mask=obs.csp.feedback_mask=0xc0000;obs.csp.mode_command[2]=8;obs.csp.mode_display[2]=9;
 obs.csp.target_position[2]=123456;obs.csp.actual_position[2]=-765;
 d.observation_cache_.publish(obs);const auto report=d.driveDiagnostics();
 assert(report.find("mode_command=8 mode_display=9")!=std::string::npos);
 assert(report.find("target_position_raw=123456")!=std::string::npos);
 assert(report.find("actual_position_raw=-765")!=std::string::npos);
 assert(report.find("manufacturer_error_code=unavailable_not_mapped")!=std::string::npos);
 for(const char* label:{"L1 slot=0","L8 slot=7","R1 slot=8","R8 slot=15","Head1 slot=16","Waist2 slot=19"})assert(report.find(label)!=std::string::npos);
#if __has_include("position_waist_units.h")
 // Exercise the actual legacy writer in memory as well as the owned-session writer.
 EtherCATDriver legacy;legacy.motor_enable_.store(true);
 for(unsigned slot=16;slot<20;++slot){
  const unsigned k=slot-16;
  legacy.is_csp_motor_[slot]=true;legacy.csp_rx_[slot]=&crx[k];legacy.csp_tx_[slot]=&ctx[k];
  ctx[k].statusword=0x27;legacy.motor_enable_mask_[slot].store(true);
  legacy.motor_cmd_pos_[slot].store(.01f);legacy.motor_cmd_spd_[slot].store(slot%2?-.02f:.02f);
 }
 for(unsigned tick=0;tick<35;++tick)legacy.cycle();
 for(unsigned slot=16;slot<20;++slot){
  const double v=slot%2?double(-.02f):double(.02f);
  const bool waist=slot>=18;
  const double multiplier=waist?position_session::waist_units::motor_per_position[slot-18]:1.;
  const auto expected=waist&&position_session::waist_units::feedforward_gain>0?0:
      position_session::csp_counts(v*multiplier);
  assert(crx[slot-16].velocity==expected);
  assert(crx[slot-16].position==position_session::csp_counts(.01f));
 }
#endif
 std::cout<<"actual PDO writer: reset never enables or changes targets/gains/torques; 20-axis diagnostics passed\n";
}

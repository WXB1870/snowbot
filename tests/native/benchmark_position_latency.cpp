// Offline only: actual native import/commit and receipt queue, pipes replace DDS.
#include "position_trajectory.h"
#include "position_receipt.h"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <cmath>
using namespace position_session;
struct Wire {
 unsigned version=1;uint64_t segment_id=0;int64_t effective_at_ns=0;
 std::vector<double> piece_start,duration,source_duration,source_elapsed,controls;
 std::vector<unsigned char> control_count;
 std::array<double,16> max_velocity{},max_acceleration{},max_jerk{};
};
struct Packet {
 using SharedPtr=std::shared_ptr<Packet>;
 struct {uint64_t session=1,sequence=0;int64_t source_stamp_ns=0,expires_at_ns=0;uint8_t operation=1;} meta;
 Wire trajectory;
};
int main(int argc,char** argv){
 if(argc!=4)return 2;
 const int64_t receipt_period=std::stoll(argv[1])*1000000;
 const int stress_threads=std::stoi(argv[2]),import_delay_ms=std::stoi(argv[3]);
 if(receipt_period<=0||stress_threads<0||stress_threads>4||import_delay_ms<0||import_delay_ms>50)return 2;
 std::atomic<bool> stop{false};std::vector<std::thread> load;
 for(int i=0;i<stress_threads;++i)load.emplace_back([&]{volatile double x=.1;while(!stop.load(std::memory_order_relaxed))for(int n=0;n<1000;++n)x=std::sin(x)+.001;});
 TrajectoryQueue queue;ReceiptQueue receipts;
 TrajectoryAdmission<Packet> admission(queue,[&](auto&){if(import_delay_ms)std::this_thread::sleep_for(std::chrono::milliseconds(import_delay_ms));});
 std::array<double,16> measured{},speed{};measured[1]=.2;measured[8]=-.2;measured[14]=.12;measured[15]=-.08;
 std::cout<<"READY "<<steady_ns()<<std::endl;
 std::string line;int result_code=0;
 while(std::getline(std::cin,line)){
  if(line=="QUIT")break;
  if(line=="PING"){std::cout<<"CLOCK "<<steady_ns()<<std::endl;continue;}
  std::istringstream in(line);auto packet=std::make_shared<Packet>();auto& w=packet->trajectory;auto& m=packet->meta;
  unsigned operation=0,pieces=0;
  in>>m.sequence>>operation>>m.source_stamp_ns>>m.expires_at_ns>>w.effective_at_ns>>pieces;m.operation=static_cast<uint8_t>(operation);w.segment_id=m.sequence;
  for(auto* values:{&w.max_velocity,&w.max_acceleration,&w.max_jerk})for(auto& value:*values)in>>value;
  for(unsigned p=0;p<pieces;++p){
   double start=0,duration=0,source_duration=0,source_elapsed=0;
   in>>start>>duration>>source_duration>>source_elapsed;
   w.piece_start.push_back(start);w.duration.push_back(duration);w.source_duration.push_back(source_duration);w.source_elapsed.push_back(source_elapsed);
   for(unsigned axis=0;axis<16;++axis){unsigned count=0;in>>count;w.control_count.push_back(static_cast<unsigned char>(count));for(unsigned k=0;k<count;++k){double value=0;in>>value;w.controls.push_back(value);}}
  }
  if(!in||pieces==0||pieces>64){result_code=3;break;}
  const auto submitted=steady_ns();if(!admission.submit(packet,.25,.5,162)){result_code=4;break;}
  TrajectoryAdmission<Packet>::Result* result=nullptr;
  while(!(result=admission.poll())){
   auto tick=(steady_ns()/1000000+1)*1000000;
   std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(tick)));
  }
  const auto decided=steady_ns();
  const auto error=result->error.empty()?queue.commit(*result->prepared,decided,decided,measured,speed):result->error.c_str();
  const bool accepted=error==nullptr;
  if(!receipts.push(m.session,m.sequence,1,1,decided,accepted,error)){result_code=5;break;}
  admission.release(result);
  const auto dispatch=(steady_ns()/receipt_period+1)*receipt_period;
  std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(dispatch)));
  AdmissionReceipt receipt;if(!receipts.pop(receipt)){result_code=6;break;}
  std::cout<<std::setprecision(12)<<accepted<<' '<<(decided-m.source_stamp_ns)*1e-6<<' '
    <<(steady_ns()-m.source_stamp_ns)*1e-6<<' '<<(w.effective_at_ns-decided)*1e-6<<' '
    <<(decided-submitted)*1e-6<<' '<<(accepted?"accepted":receipt.reason.data())<<std::endl;
  while(admission.busy())std::this_thread::sleep_for(std::chrono::microseconds(100));
 }
 stop=true;for(auto& thread:load)thread.join();return result_code;
}

// Actual SOEM RX/indexing over AF_UNIX only. Never calls NIC setup or a driver.
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sched.h>
#include <cassert>
#include <atomic>
#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
extern "C" {
#include "ethercat.h"
int ecx_inframe(ecx_portt*,int,int);
}
int64_t mono(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
int64_t cpu(){timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
struct SocketPort {
 ecx_portt port{};ecx_redportt red{};int mainfd[2]{},redfd[2]{};
 SocketPort(){
  assert(socketpair(AF_UNIX,SOCK_DGRAM,0,mainfd)==0);
  assert(socketpair(AF_UNIX,SOCK_DGRAM,0,redfd)==0);
  port.sockhandle=mainfd[0];red.sockhandle=redfd[0];port.redport=&red;port.redstate=0;
  auto init=[&](ec_stackT& s,int* fd,ec_bufT* temp,ec_bufT (*rx)[EC_MAXBUF],int (*states)[EC_MAXBUF],int (*sa)[EC_MAXBUF]){
   s.sock=fd;s.tempbuf=temp;s.rxbuf=rx;s.rxbufstat=states;s.rxsa=sa;s.txbuf=&port.txbuf;s.txbuflength=&port.txbuflength;
   timeval timeout{0,1};assert(setsockopt(*fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))==0);
  };
  init(port.stack,&port.sockhandle,&port.tempinbuf,&port.rxbuf,&port.rxbufstat,&port.rxsa);
  init(red.stack,&red.sockhandle,&red.tempinbuf,&red.rxbuf,&red.rxbufstat,&red.rxsa);
  pthread_mutex_init(&port.rx_mutex,nullptr);
  for(auto& n:port.txbuflength)n=ETH_HEADERSIZE+34;
 }
 ~SocketPort(){for(int fd:mainfd)close(fd);for(int fd:redfd)close(fd);pthread_mutex_destroy(&port.rx_mutex);}
 void send_frame(int idx,int wkc,bool secondary=false){
  std::array<unsigned char,ETH_HEADERSIZE+34> bytes{};
  auto* eth=reinterpret_cast<ec_etherheadert*>(bytes.data());eth->etype=htons(ETH_P_ECAT);eth->sa1=htons(0x0101);
  auto* body=reinterpret_cast<ec_comt*>(bytes.data()+ETH_HEADERSIZE);body->elength=htoes(32);body->index=uint8_t(idx);
  bytes[ETH_HEADERSIZE+32]=uint8_t(wkc);bytes[ETH_HEADERSIZE+33]=uint8_t(wkc>>8);
  assert(send(secondary?redfd[1]:mainfd[1],bytes.data(),bytes.size(),0)==int(bytes.size()));
 }
};
int main(int argc,char** argv){
 const bool runnable=argc>2 && std::string(argv[2])=="runnable";
 std::ofstream log;if(argc>1)log.open(argv[1]);
 SocketPort s;std::atomic<bool> go{false},stop{false};std::promise<int> started;auto ready=started.get_future();
 if(runnable){
  cpu_set_t allowed;assert(sched_getaffinity(0,sizeof(allowed),&allowed)==0);int selected=-1;
  for(int i=0;i<CPU_SETSIZE;++i)if(CPU_ISSET(i,&allowed))selected=i;
  cpu_set_t only;CPU_ZERO(&only);CPU_SET(selected,&only);assert(sched_setaffinity(0,sizeof(only),&only)==0);
  sched_param p{};p.sched_priority=50;assert(pthread_setschedparam(pthread_self(),SCHED_FIFO,&p)==0);
 }
 std::thread control([&]{
  if(runnable){sched_param p{};p.sched_priority=60;assert(pthread_setschedparam(pthread_self(),SCHED_FIFO,&p)==0);}
  started.set_value(int(syscall(SYS_gettid)));
  while(!stop.load()){
   if(runnable && go.load()){
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    const auto end=mono()+5000000;while(mono()<end){};break;
   }
   std::this_thread::sleep_for(std::chrono::microseconds(runnable?50:1000));
  }
 });
 const int other=ready.get(),tid=int(syscall(SYS_gettid));
 if(log){
  log<<"[SCHED-THREAD] role=pdo pid="<<getpid()<<" tid="<<tid<<" simulation=1\n";
  log<<"[SCHED-THREAD] role=control pid="<<getpid()<<" tid="<<other<<" simulation=1\n";log.flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
 }
 if(!runnable){
  s.port.rxbufstat[1]=EC_BUF_TX;s.send_frame(1,67);assert(ecx_inframe(&s.port,1,0)==67);
  s.port.rxbufstat[1]=s.port.rxbufstat[2]=EC_BUF_TX;s.send_frame(2,55);s.send_frame(1,67);
  assert(ecx_inframe(&s.port,1,0)==EC_OTHERFRAME);assert(ecx_inframe(&s.port,1,0)==67);assert(ecx_inframe(&s.port,2,0)==55);
  s.red.rxbufstat[3]=EC_BUF_TX;s.send_frame(3,61,true);assert(ecx_inframe(&s.port,3,1)==61);
  s.port.rxbufstat[4]=EC_BUF_TX;assert(ecx_waitinframe(&s.port,4,1000)<=EC_NOFRAME);
  s.send_frame(4,67);assert(ecx_inframe(&s.port,4,0)==67); // A later frame does not corrupt index state.
  s.port.rxbufstat[5]=EC_BUF_TX;
  std::thread delayed([&]{std::this_thread::sleep_for(std::chrono::microseconds(100));s.send_frame(5,67);});
  assert(ecx_waitinframe(&s.port,5,10000)==67);delayed.join();
 }
 std::vector<double> wall,cost;int64_t worst_begin=0,worst_end=0;double worst=0;
 for(unsigned i=0;i<(runnable?1u:40u);++i){
  s.port.rxbufstat[12]=EC_BUF_TX;const auto begin=mono(),used=cpu();
  if(runnable)go.store(true);
  assert((runnable?ecx_waitinframe(&s.port,12,1000):ecx_inframe(&s.port,12,0))<=EC_NOFRAME);
  const auto end=mono();wall.push_back((end-begin)*1e-3);cost.push_back((cpu()-used)*1e-3);
  if(wall.back()>worst){worst=wall.back();worst_begin=begin;worst_end=end;}
 }
 if(runnable){sched_param p{};assert(pthread_setschedparam(pthread_self(),SCHED_OTHER,&p)==0);}
 if(log){
  log<<"[ECAT-FAULT] simulation=1 case="<<(runnable?"runnable_wait":"empty_receive")<<"\n";
  log<<"[ECAT-FAULT-CYCLE] simulation=1 tid="<<tid<<" first_bad=1 recv_begin_ns="<<worst_begin<<" recv_end_ns="<<worst_end<<"\n";log.flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
 }
 stop.store(true);control.join();std::sort(wall.begin(),wall.end());std::sort(cost.begin(),cost.end());
 std::cout<<"{\"samples\":"<<wall.size()<<",\"median_wall_us\":"<<wall[wall.size()/2]<<",\"max_wall_us\":"<<wall.back()<<",\"median_cpu_us\":"<<cost[cost.size()/2]<<",\"runnable_simulation\":"<<(runnable?"true":"false")<<",\"hardware_accessed\":false}"<<std::endl;
}

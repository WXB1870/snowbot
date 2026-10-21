#include "servo_control/position_watchdog.h"
#include "servo_control/position_follower.h"
#include "servo_control/position_clock.h"
#include "position_session_contract.h"
#include <cassert>
#include <thread>
#include <iostream>
#include <vector>
#include <algorithm>
using namespace position_session;
int main(){
 ObservationCache cache;Observation o;o.valid_mask=0xfffff;o.wkc=o.expected_wkc=67;o.sample=1;o.sampled_ns=1000000000;
 o.position.fill(.2);o.status.fill(0x27);cache.publish(o);assert(cache.read().sample==1);
 Gate gate;Meta m{1,1,1000000000,1200000000,1};assert(gate.accepts(m,1000000000,1000000000));gate.commit(m,1000000000,1000000000);
 assert(!gate.accepts(m,1000000000,1000000000));m.sequence=2;m.operation=0;assert(gate.accepts(m,1010000000,1010000000));gate.commit(m,1010000000,1010000000);
 const auto deadline=gate.deadline();m.sequence=1;assert(!gate.accepts(m,1010000000,1010000000));assert(gate.deadline()==deadline);
 m.sequence=3;assert(!gate.accepts(m,1200000000,1200000000));m.stamp=1210000000;m.expires=1300000000;assert(!gate.accepts(m,1210000000,1210000000));
 m.operation=1;assert(gate.accepts(m,1210000000,1210000000));gate.commit(m,1210000000,1210000000);m.operation=2;++m.sequence;assert(gate.accepts(m,1220000000,1220000000));gate.commit(m,1220000000,1220000000);
 Meta restart{2,1,1230000000,1400000000,1};assert(gate.accepts(restart,1230000000,1230000000));gate.commit(restart,1230000000,1230000000);m.operation=1;m.stamp=1410000000;m.expires=1500000000;assert(!gate.accepts(m,1410000000,1410000000));
 gate.interrupt();Meta old_session=restart;old_session.sequence=2;old_session.stamp=1410000000;old_session.expires=1500000000;
 assert(!gate.accepts(old_session,1410000000,1410000000));Meta new_session{3,1,1410000000,1500000000,1};assert(gate.accepts(new_session,1410000000,1410000000));
 Follower f;std::array<double,14> q{};q[0]=.4;q[1]=.2;q[8]=-.2;f.seed(q);q[0]=.8;assert(f.set(q));assert(f.step(.001));assert(f.q[0]==.4);double previous=f.q[0],velocity=0;
 for(int i=0;i<4000;++i){assert(f.step(.001));assert(std::abs(f.v[0])<=f.vmax+1e-9);assert(std::abs(f.v[0]-velocity)<=f.amax*.001+1e-9);assert(std::abs(f.q[0]-previous)<=f.vmax*.001+1e-9);previous=f.q[0];velocity=f.v[0];if(i==100){q[0]=.6;f.set(q);assert(f.q[0]==previous);}}
 assert(!f.step(.1));Interval clock;clock.reset(1000000000);assert(std::abs(clock.tick(1001000000)-.001)<1e-12);assert(clock.tick(1001000000)<0);
 Output p;p.selected=true;p.policy=2;p.operation=1;p.session=1;p.sequence=1;p.deadline=1100000000;p.q.fill(.3);Watchdog w;w.evaluate(p,o,1000000000);assert(!w.disabled&&w.reason==OK);
 o.sampled_ns=1100000000;w.evaluate(p,o,1100000000);assert(w.reason==EXPIRED&&!w.disabled&&w.output.q[0]==.2f);
 o.position[0]=.6f;o.sampled_ns=1101000000;w.evaluate(p,o,1101000000);assert(w.output.q[0]==.2f); // Capture once, not drifting feedback.
 o.wkc=61;w.evaluate(p,o,1102000000);assert(w.disabled&&w.reason==FEEDBACK_INVALID);
 o.wkc=67;o.status[0]|=8;w.evaluate(p,o,1103000000);assert(w.disabled&&w.reason==DRIVE_FAULT);
 o.status[0]=0x27;o.sampled_ns=1104000000;++p.sequence;p.deadline=1300000000;w.evaluate(p,o,1104000000);assert(w.reason==OK&&!w.disabled);
 // Fresh ROS output copies cannot extend the original RT deadline.
 o.sampled_ns=1300000000;w.evaluate(p,o,1300000000);assert(w.reason==EXPIRED);
 p.policy=1;w.evaluate(p,o,1300000000);assert(w.disabled);
 p.selected=false;w.evaluate(p,o,1300000000);assert(!w.disabled&&w.reason==OK);
 std::atomic<bool> done{false};std::thread writer([&]{Observation s;for(uint64_t i=1;i<=200000;++i){s.sample=i;s.sampled_ns=i;s.position.fill(float(i));s.speed.fill(-float(i));cache.publish(s);}done=true;});
 unsigned coherent=0;while(!done.load()||coherent==0){auto s=cache.read();if(s.sample<=1)continue;++coherent;assert(s.sampled_ns==int64_t(s.sample));for(int j=0;j<20;++j){assert(s.position[j]==float(s.sample));assert(s.speed[j]==-float(s.sample));}}writer.join();
 std::vector<long long> times;times.reserve(20000);for(int i=0;i<20000;++i){auto t=steady_ns();cache.publish(o);times.push_back(steady_ns()-t);}std::sort(times.begin(),times.end());
 std::cout<<"PASS gate, source deadline, old sessions, takeover, continuous targets, pause, monotonic clock, RT freeze/WKC/Fault, resource mask and coherent SPSC; publish ns median="<<times[10000]<<" p99="<<times[19800]<<" max="<<times.back()<<"\n";
}

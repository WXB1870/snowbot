#include "servo_control/drive_maintenance.h"
#include <cassert>
#include <iostream>
using namespace drive_maintenance;
position_session::Observation observation(int64_t now) {
 position_session::Observation s;s.sample=1;s.sampled_ns=now;s.valid_mask=all_axes;s.wkc=s.expected_wkc=67;s.status.fill(0x218);return s;
}
int main(){
 uint32_t out=99;int64_t now=1000000000;
 for(unsigned i=0;i<20;++i){assert(reset_controlword(1u<<i,i)==0x80);for(unsigned j=0;j<20;++j)assert(!(reset_controlword(1u<<i,j)&0x7f));}
 // Each interlock rejects without touching the normal PDO writer.
 for(int reason=0;reason<8;++reason){Reset r;auto s=observation(now);assert(r.request());assert(!r.request());
  uint32_t requested=0;bool selected=false,exiting=false;
  if(reason==0)requested=1;
  if(reason==1)selected=true;
  if(reason==2)exiting=true;
  if(reason==3)s.valid_mask=0;
  if(reason==4)s.sampled_ns=now-21000000;
  if(reason==5)s.wkc=0;
  if(reason==6)s.status[19]=0x27;
  if(reason==7)s.sample=0;
  assert(!r.step(s,now,requested,selected,exiting,out));assert(out==0&&r.state()==rejected);
 }
 Reset r;auto s=observation(now);assert(r.request());unsigned pulse_cycles=0;
 for(unsigned i=0;i<800;++i){auto n=now+i*1000000;s.sample++;s.sampled_ns=n;
  if(i>=25)s.status.fill(0x40);
  bool override=r.step(s,n,0,false,false,out);
  if(out){assert(override);assert(out==all_axes);++pulse_cycles;}
  if(i<30)assert(r.state()!=cleared);
 }
 assert(pulse_cycles==10 && r.state()==cleared && !r.busy());
 // Fault remains: one pulse, no retry; explicit subsequent request is allowed.
 Reset stuck;s=observation(now);assert(stuck.request());pulse_cycles=0;
 for(unsigned i=0;i<800;++i){s.sample++;s.sampled_ns=now+i*1000000;stuck.step(s,s.sampled_ns,0,false,false,out);
  if(out)++pulse_cycles;}
 assert(stuck.state()==fault_remains&&pulse_cycles==10&&stuck.request());
 // Invalid or frozen feedback mid-pulse aborts, never reports success.
 for(unsigned mode=0;mode<4;++mode){Reset a;s=observation(now);assert(a.request());assert(a.step(s,now,0,false,false,out));
  s.sample++;s.sampled_ns=now+10000000;assert(a.step(s,s.sampled_ns,0,false,false,out)&&out);
  if(mode!=0)++s.sample;
  if(mode==1)s.wkc=0;
  if(mode==2)s.status[0]=0x27;
  assert(a.step(s,s.sampled_ns+1000000,mode==3?1:0,false,false,out));assert(a.state()==aborted&&out==0);
 }
 Reset empty;s=observation(now);s.status.fill(0x40);assert(empty.request());empty.step(s,now,0,false,false,out);assert(empty.state()==cleared&&out==0);
 std::cout<<"drive reset state machine: per-axis mask, interlocks, single pulse, real clear, timeout and abort passed\n";
}

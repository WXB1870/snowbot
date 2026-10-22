#include "position_session_contract.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <cstdlib>
#include <new>
static bool count_memory=false;
static unsigned allocations=0,deletions=0;
void* operator new(std::size_t n){if(count_memory)++allocations;if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{if(count_memory&&p)++deletions;std::free(p);}
void operator delete(void* p,std::size_t)noexcept{::operator delete(p);}
static unsigned checks=0;
#define CHECK(c) do {++checks;if(!(c))throw std::runtime_error(#c);}while(0)
struct Observation {
    struct {struct {int32_t sec=10;uint32_t nanosec=0;} stamp;} header;
    uint64_t boot_id=1,pdo_sample=1;double pdo_age=.001;
    int wkc=67,expected_wkc=67;uint32_t valid_mask=0xfffff;
    std::array<uint16_t,20> status{};
    std::array<double,20> position{},speed{};
};
static void receive(position_session::Tracker& t,Observation& m,uint64_t seq,int64_t dt) {
    m.pdo_sample=seq;m.header.stamp.nanosec=uint32_t(dt);
    t.observe(m,10000000000LL+dt,1000000000LL+dt);
}
static bool fresh(const position_session::Tracker& t,int64_t dt) {return t.fresh(10000000000LL+dt,1000000000LL+dt);}
static void feedback_sequences() {
    position_session::Tracker t;Observation m;
    CHECK(!fresh(t,0));receive(t,m,1,0);CHECK(!fresh(t,0));receive(t,m,2,1000000);CHECK(fresh(t,1000000));
    // Long valid no-target stream never creates a control session.
    position_session::Gate idle;
    for(unsigned i=3;i<500;++i){receive(t,m,i,int64_t(i)*1000000);CHECK(fresh(t,int64_t(i)*1000000));CHECK(!idle.selected());}
    // Duplicate PDO snapshots cannot refresh progress even when ROS publication time advances.
    const auto stalled=499000000+position_session::ros_feedback_timeout_ns+1000000;
    receive(t,m,499,stalled);CHECK(!fresh(t,stalled));
    receive(t,m,500,stalled+1000000);CHECK(!fresh(t,stalled+1000000));receive(t,m,501,stalled+2000000);CHECK(fresh(t,stalled+2000000));
    // Source restart requires a new baseline.
    m.boot_id=2;receive(t,m,1,stalled+3000000);CHECK(!fresh(t,stalled+3000000));receive(t,m,2,stalled+4000000);CHECK(fresh(t,stalled+4000000));
    for(unsigned failure=0;failure<5;++failure){
        position_session::Tracker x;Observation o;receive(x,o,1,0);receive(x,o,2,1000000);CHECK(fresh(x,1000000));
        if(failure==0)o.wkc=61;
        if(failure==1)o.valid_mask=0;
        if(failure==2)o.status[0]=8;
        if(failure==3)o.position[0]=std::numeric_limits<double>::quiet_NaN();
        if(failure==4)o.speed[8]=std::numeric_limits<double>::infinity();
        receive(x,o,3,2000000);CHECK(!fresh(x,2000000));
        o=Observation{};receive(x,o,4,3000000);CHECK(!fresh(x,3000000));receive(x,o,5,4000000);CHECK(fresh(x,4000000));
    }
    const auto late=position_session::ros_feedback_timeout_ns+1000000;
    position_session::Tracker delayed;Observation o;receive(delayed,o,1,0);receive(delayed,o,2,late);CHECK(!fresh(delayed,late));
    receive(delayed,o,3,late+1000000);CHECK(fresh(delayed,late+1000000));
}
static void readiness_generation() {
    position_session::Tracker t;Observation m;
    receive(t,m,1,0);CHECK(t.received_messages==1);CHECK(t.ready_sample==0);
    receive(t,m,2,1000000);CHECK(t.ready_sample==2);CHECK(t.ready_ros==10001000000LL);
    const uint64_t old_sample=t.sample;const int64_t old_stamp=t.ready_ros;
    m.wkc=61;receive(t,m,3,2000000);CHECK(t.ready_sample==0);CHECK(t.ready_ros==0);
    m.wkc=67;receive(t,m,4,3000000);CHECK(!fresh(t,3000000));CHECK(t.ready_sample==0);
    receive(t,m,5,4000000);CHECK(fresh(t,4000000));CHECK(t.ready_sample==5);CHECK(t.ready_ros==10004000000LL);
    CHECK(old_sample<t.ready_sample);CHECK(old_stamp<t.ready_ros);
    // Invalid but received frames remain distinguishable from no message at all.
    position_session::Tracker absent;CHECK(absent.received_messages==0);
    m.pdo_sample=0;absent.observe(m,10004000000LL,1004000000LL);CHECK(absent.received_messages==1);CHECK(absent.sample==0);
}
static position_session::Meta command(uint64_t session,uint64_t sequence,int64_t stamp,uint8_t operation=1){return {session,sequence,stamp,stamp+100000000,operation};}
static void sessions_deadlines() {
    constexpr int64_t ros=10000000000LL,steady=1000000000LL;
    position_session::Gate g;auto first=command(1,1,ros);
    CHECK(!g.selected());CHECK(g.accepts(first,ros,steady));g.commit(first,ros,steady);CHECK(g.live(steady));
    const auto deadline=g.deadline();CHECK(!g.accepts(first,ros+1000,steady+1000));CHECK(g.deadline()==deadline);
    for(unsigned i=2;i<500;++i){
        auto next=command(1,i,ros+int64_t(i)*10000,0);CHECK(g.accepts(next,next.stamp,steady+int64_t(i)*10000));
        g.commit(next,next.stamp,steady+int64_t(i)*10000);CHECK(g.last().sequence==i);
    }
    g.interrupt();CHECK(!g.live(steady+5000000));
    auto old=command(1,500,ros+6000000);CHECK(!g.accepts(old,old.stamp,steady+6000000));
    auto replacement=command(2,1,ros+6000000);CHECK(g.accepts(replacement,replacement.stamp,steady+6000000));g.commit(replacement,replacement.stamp,steady+6000000);
    g.interrupt();CHECK(!g.accepts(old,old.stamp,steady+6000000));
    position_session::Gate transit;auto queued=command(10,1,ros);
    CHECK(transit.accepts(queued,ros+80000000,steady));transit.commit(queued,ros+80000000,steady);
    CHECK(transit.deadline()==steady+20000000);CHECK(!transit.live(steady+20000000));
    auto invalid=command(0,1,ros);CHECK(!transit.accepts(invalid,ros,steady));
    invalid=command(11,0,ros);CHECK(!transit.accepts(invalid,ros,steady));
    invalid=command(11,1,ros);invalid.expires=ros+200000001;CHECK(!transit.accepts(invalid,ros,steady));
    invalid=command(11,1,ros+1);CHECK(!transit.accepts(invalid,ros,steady));
    // Normal pause/explicit takeover remains distinct from a fault/restart interrupt.
    position_session::Gate pause;auto move=command(20,1,ros);CHECK(pause.accepts(move,ros,steady));pause.commit(move,ros,steady);
    auto hold=command(20,2,ros+1000,2);CHECK(pause.accepts(hold,hold.stamp,steady+1000));pause.commit(hold,hold.stamp,steady+1000);
    auto update=command(20,3,ros+2000,0);CHECK(!pause.accepts(update,update.stamp,steady+2000));
    update.operation=1;CHECK(pause.accepts(update,update.stamp,steady+2000));
}
// Run far beyond the old 33-session failure, including fresh restamped replays.
static void unlimited_sessions_and_replay() {
    constexpr int64_t ros=10000000000LL,steady=1000000000LL;
    position_session::Gate gate,pending;
    count_memory=true;
    for(uint64_t id=1;id<=10000;++id){
        auto m=command(id,1,ros);
        CHECK(gate.accepts(m,ros,steady));gate.commit(m,ros,steady);
        // An active following session still excludes another takeover.
        auto conflict=command(id+1,1,ros);
        CHECK(!gate.accepts(conflict,ros,steady));
        auto pause=command(id,2,ros,2);
        CHECK(gate.accepts(pause,ros,steady));gate.commit(pause,ros,steady);
        auto duplicate=command(id,2,ros);
        CHECK(!gate.accepts(duplicate,ros,steady));
        gate.interrupt();
        // Cancel before selection: even a fresh/new-sequence retry is retired.
        auto queued=command(id,1,ros);
        CHECK(pending.accepts(queued,ros,steady));pending.invalidate_pending(queued);
        queued.sequence=999;CHECK(!pending.accepts(queued,ros,steady));
    }
    const auto generation=gate.generation();
    for(uint64_t id=1;id<=10000;++id){
        auto replay=command(id,99999,ros+1000);
        CHECK(!gate.accepts(replay,replay.stamp,steady+1000));
        CHECK(!pending.accepts(replay,replay.stamp,steady+1000));
        CHECK(gate.generation()==generation);
    }
    auto next=command(10001,1,ros+1000);
    CHECK(gate.accepts(next,next.stamp,steady+1000));
    CHECK(pending.accepts(next,next.stamp,steady+1000));
    gate.commit(next,next.stamp,steady+1000);gate.interrupt();
    // Never wrap a saturated uint64 identity and accidentally accept old IDs.
    next.session=std::numeric_limits<uint64_t>::max();
    CHECK(gate.accepts(next,next.stamp,steady+1000));gate.commit(next,next.stamp,steady+1000);gate.interrupt();
    next.session=1;CHECK(!gate.accepts(next,next.stamp,steady+1000));
    count_memory=false;CHECK(allocations==0&&deletions==0);
    std::cout<<"PASS 10000 takeovers and pending cancellations; original/fresh restamped replay blocked; zero allocation/deletion\n";
}
int main(){try{feedback_sequences();readiness_generation();sessions_deadlines();unlimited_sessions_and_replay();std::cout<<checks<<" shared-contract assertions passed\n";}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}

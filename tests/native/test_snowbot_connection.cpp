#include "connection_state.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace snowbot_connection;
static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) throw std::runtime_error(#x); } while (0)
static void baseline(Connection& c, double t=10., uint64_t n=1) {
    c.endpoint("", "publisher-1");
    CHECK(c.observe(n,t,t,100+t,true));
    CHECK(!c.feedback_fresh(t,100+t));
    CHECK(c.observe(n+1,t+.01,t+.01,100+t+.01,true));
    CHECK(c.feedback_fresh(t+.01,100+t+.01));
}
static void startup_orders() {
    for (bool upper_first : {true,false}) {
        Connection upper,middle;
        auto& first = upper_first ? upper : middle;
        first.initialize();
        for (unsigned i=0; i<100; ++i) {
            CHECK(first.status(1+i*.01,101+i*.01).state == State::WaitingFeedback);
            CHECK(!first.accept(11,1,1+i*.01,1+i*.01,101+i*.01,true).empty());
        }
        (upper_first ? middle : upper).initialize();
        baseline(middle); baseline(upper);
        CHECK(upper.status(10.01,110.01).state == State::ReadyIdle);
        CHECK(middle.status(10.01,110.01).state == State::ReadyIdle);
        CHECK(upper.accept(11,1,10.02,10.02,110.02,true).empty());
        CHECK(middle.accept(11,1,10.02,10.02,110.02,true).empty());
    }
}
static void delayed_no_target() {
    Connection c;c.initialize();c.endpoint("", "publisher-1");
    CHECK(c.status(10,110).reason == "publisher_found_no_message");
    CHECK(!c.feedback_fresh(10,110));
    baseline(c);
    for (unsigned i=3; i<500; ++i) {
        const double t=10+i*.01;
        CHECK(c.observe(i,t,t,100+t,true));
        CHECK(c.status(t,100+t).state == State::ReadyIdle);
        CHECK(!c.executing(t,100+t));
    }
}
static void pdo_stall_fault_recovery() {
    Connection c;c.initialize();baseline(c);
    CHECK(c.accept(5,1,10.02,10.02,110.02,true).empty());
    const auto epoch=c.epoch();
    // Transport still runs but successful sample does not advance.
    for (unsigned i=1; i<30; ++i) {
        const double t=10.01+i*.01;
        CHECK(!c.observe(2,10.01,t,100+t,true));
    }
    CHECK(!c.executing(10.31,110.31));CHECK(c.epoch()>epoch);
    CHECK(c.status(10.31,110.31).state == State::StaleFault);
    CHECK(c.observe(3,10.32,10.32,110.32,true));
    CHECK(!c.feedback_fresh(10.32,110.32));
    CHECK(c.observe(4,10.33,10.33,110.33,true));
    CHECK(c.status(10.33,110.33).state == State::ReadyIdle);
    CHECK(!c.executing(10.33,110.33));
    CHECK(c.accept(5,2,10.34,10.34,110.34,true)=="new_session_required");
    CHECK(c.accept(6,1,10.34,10.34,110.34,true).empty());
    CHECK(!c.observe(5,10.35,10.35,110.35,false,"drive_fault"));
    CHECK(c.status(10.35,110.35).reason == "drive_fault");
    CHECK(!c.executing(10.35,110.35));
}
static void restart() {
    Connection upper;upper.initialize();baseline(upper);
    CHECK(upper.accept(9,1,10.02,10.02,110.02,true).empty());
    upper.endpoint("", "new-middle-instance");
    CHECK(!upper.executing(10.03,110.03));
    CHECK(upper.observe(1,10.03,10.03,110.03,true));
    CHECK(upper.observe(2,10.04,10.04,110.04,true));
    CHECK(!upper.executing(10.04,110.04));
    CHECK(!upper.accept(9,2,10.05,10.05,110.05,true).empty());
    CHECK(upper.accept(10,1,10.05,10.05,110.05,true).empty());
    Connection new_middle;new_middle.initialize();baseline(new_middle,10.02);
    // New process has no retired-session history; echoed instance still rejects old packets.
    CHECK(new_middle.accept(9,99,10.04,10.04,110.04,false)=="peer_instance_or_baseline_changed");
    CHECK(!new_middle.executing(10.04,110.04));
    CHECK(new_middle.accept(10,1,10.05,10.05,110.05,true).empty());
    // PDO source restart, including sequence regression, establishes a new baseline.
    CHECK(!new_middle.observe(1,10.06,10.06,110.06,true));
    CHECK(!new_middle.feedback_fresh(10.06,110.06));
    CHECK(new_middle.observe(2,10.07,10.07,110.07,true));
    CHECK(!new_middle.feedback_fresh(10.07,110.07));
    CHECK(new_middle.observe(3,10.08,10.08,110.08,true));
    CHECK(!new_middle.executing(10.08,110.08));
}
static void diagnostics_and_failures() {
    CHECK(endpoint_reason(0,0,false,false)=="publisher_missing");
    CHECK(endpoint_reason(0,1,true,true)=="unexpected_publisher");
    CHECK(endpoint_reason(1,1,false,true)=="message_type_mismatch");
    CHECK(endpoint_reason(1,1,true,false)=="qos_incompatible_or_unknown");
    CHECK(endpoint_reason(2,2,true,true)=="publisher_ambiguous");
    CHECK(endpoint_reason(1,1,true,true).empty());
    Connection c;
    CHECK(c.status(1,101).state==State::Initializing);
    c.fail("model_missing: /missing/snowbot.urdf");
    CHECK(c.status(1,101).state==State::InitFailed);
    CHECK(c.status(1,101).reason.find("/missing/")!=std::string::npos);
    c.initialize();baseline(c);
    CHECK(!c.accept(1,1,10.02,10.02,110.02,true).empty());
    LogChanges logs;CHECK(logs.due("waiting",0));
    for (unsigned i=1;i<500;++i) CHECK(!logs.due("waiting",i*.01));
    CHECK(logs.due("waiting",5));CHECK(logs.due("ready",5.01));
}
static void deadlines_and_invalid_samples() {
    Connection c;c.initialize();baseline(c);
    CHECK(c.accept(0,1,10.02,10.02,110.02,true)=="invalid_session_or_expired_target");
    CHECK(c.accept(1,0,10.02,10.02,110.02,true)=="invalid_session_or_expired_target");
    CHECK(c.accept(1,1,10.005,10.02,110.02,true)=="target_predates_feedback_baseline");
    CHECK(!c.accept(1,1,10.03,10.02,110.02,true).empty());
    CHECK(!c.accept(1,1,10.02,10.02,110.02,true,.3).empty());
    CHECK(c.accept(1,1,10.02,10.06,110.06,true,.1).empty());
    CHECK(c.accept(1,1,10.02,10.07,110.07,true,.1)=="target_sequence_not_advancing");
    CHECK(c.accept(1,2,10.015,10.07,110.07,true,.1)=="target_time_regressed");
    CHECK(c.observe(3,10.12,10.12,110.12,true));
    // Enqueue/transit delay consumed original budget; receiving did not buy 100 ms more.
    CHECK(!c.executing(10.121,110.121));
    CHECK(c.accept(1,3,10.13,10.13,110.13,true)=="new_session_required");
    CHECK(c.accept(2,1,10.13,10.13,110.13,true).empty());
    CHECK(!c.observe(4,std::numeric_limits<double>::quiet_NaN(),10.14,110.14,true));
    CHECK(!c.executing(10.14,110.14));
    Connection one;one.initialize();one.endpoint("","p");
    CHECK(one.observe(1,10,10,110,true));
    CHECK(one.observe(2,11,11,111,true));
    CHECK(!one.feedback_fresh(11,111)); // a late second sample cannot complete the old baseline
    CHECK(one.observe(3,11.01,11.01,111.01,true));
    CHECK(one.feedback_fresh(11.01,111.01));
}
int main() {
    try {
        startup_orders(); delayed_no_target(); pdo_stall_fault_recovery(); restart();
        diagnostics_and_failures(); deadlines_and_invalid_samples();
        std::cout<<checks<<" connection checks passed\n";
    } catch (const std::exception& e) { std::cerr<<"failed: "<<e.what()<<"\n";return 1; }
}

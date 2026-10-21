#pragma once
#include <cmath>
#include <cstdint>
#include <set>
#include <string>

// Offline-testable software connection contract. No ROS/device/motion access.
namespace snowbot_connection {
constexpr double feedback_timeout = 0.2;
constexpr double target_timeout = 0.2;
enum class State { Initializing, WaitingFeedback, ReadyIdle, Executing, StaleFault, InitFailed };
inline const char* name(State s) {
    switch (s) {
    case State::Initializing: return "initializing";
    case State::WaitingFeedback: return "waiting_feedback";
    case State::ReadyIdle: return "ready_idle";
    case State::Executing: return "executing";
    case State::StaleFault: return "stale_fault";
    case State::InitFailed: return "initialization_failed";
    }
    return "initialization_failed";
}
struct Status { State state; std::string reason; bool feedback_valid; };
class Connection {
    bool initialized_ = false, received_ = false, valid_ = false, baseline_ = false;
    bool active_ = false, recovering_ = false, had_baseline_ = false;
    std::string failure_, endpoint_ = "publisher_missing", peer_, sample_error_;
    uint64_t sample_ = 0, session_ = 0, sequence_ = 0, epoch_ = 1;
    double source_ = 0, received_at_ = 0, target_source_ = 0, target_at_ = 0, ready_at_ = 0, budget_ = target_timeout;
    std::set<uint64_t> retired_;
    void revoke() {
        if (session_) { retired_.insert(session_); recovering_ = true; }
        session_ = sequence_ = 0;
        active_ = false;
    }
    void lose_baseline() {
        if (baseline_ || active_) ++epoch_;
        revoke();
        valid_ = baseline_ = false;
    }
public:
    void initialize() { initialized_ = true; }
    void fail(const std::string& error) { failure_ = error; lose_baseline(); }
    bool failed() const { return !failure_.empty(); }
    bool initialized() const { return initialized_ && failure_.empty(); }
    uint64_t epoch() const { return epoch_; }
    uint64_t sample() const { return sample_; }
    uint64_t session() const { return session_; }
    double sample_stamp() const { return source_; }
    void endpoint(const std::string& error, const std::string& peer) {
        if (error != endpoint_ || peer != peer_) {
            lose_baseline();
            sample_ = 0; source_ = 0; received_ = false;
            peer_ = peer;
        }
        endpoint_ = error;
    }
    bool observe(uint64_t sample, double source, double ros, double steady,
                 bool valid, const std::string& error = "invalid_feedback") {
        poll(ros, steady);
        received_ = true;
        if (!endpoint_.empty()) return false;
        if (!valid || !sample || !std::isfinite(source) || !std::isfinite(ros) ||
            !std::isfinite(steady) || source <= 0 || ros < source ||
            ros-source >= feedback_timeout) {
            lose_baseline(); sample_error_ = error; return false;
        }
        // Repeated publications do not refresh successful PDO receive time.
        if (sample == sample_) return false;
        if (sample < sample_ || source <= source_) {
            lose_baseline(); sample_error_ = "sample_regressed";
            sample_ = sample; source_ = source; received_at_ = steady;
            return false;
        }
        const bool previous_baseline = baseline_;
        baseline_ = valid_; // require two advancing, valid samples after every outage
        if (baseline_ && !previous_baseline) { ready_at_ = ros; had_baseline_ = true; }
        valid_ = true; sample_error_.clear();
        sample_ = sample; source_ = source; received_at_ = steady;
        return true;
    }
    bool sample_fresh(double ros, double steady) const {
        return endpoint_.empty() && valid_ &&
            std::isfinite(ros) && std::isfinite(steady) &&
            ros >= source_ && ros-source_ < feedback_timeout &&
            steady >= received_at_ && steady-received_at_ < feedback_timeout;
    }
    bool feedback_fresh(double ros, double steady) const { return baseline_ && sample_fresh(ros, steady); }
    void poll(double ros, double steady) {
        if ((valid_ || active_) && !sample_fresh(ros, steady)) {
            lose_baseline(); sample_error_ = "pdo_not_advancing_or_feedback_stale";
        }
        if (active_ && (!std::isfinite(ros) || !std::isfinite(steady) ||
            ros < target_source_ || ros-target_source_ >= budget_ ||
            steady < target_at_ || steady-target_at_ >= budget_)) revoke();
    }
    Status status(double ros, double steady) {
        poll(ros, steady);
        if (!failure_.empty()) return {State::InitFailed, failure_, false};
        if (!initialized_) return {State::Initializing, "loading_configuration_model", false};
        if (!endpoint_.empty()) return {had_baseline_ ? State::StaleFault : State::WaitingFeedback, endpoint_, false};
        if (!received_) return {State::WaitingFeedback, "publisher_found_no_message", false};
        if (!feedback_fresh(ros, steady)) {
            const auto reason = !sample_error_.empty() ? sample_error_ :
                (!valid_ ? "no_valid_feedback" : "waiting_for_advancing_sample_baseline");
            return {had_baseline_ ? State::StaleFault : State::WaitingFeedback, reason, false};
        }
        return {active_ ? State::Executing : State::ReadyIdle,
            active_ ? "valid_session_and_target" :
                (recovering_ ? "feedback_valid_new_session_required" : "feedback_valid_waiting_target"), true};
    }
    std::string accept(uint64_t session, uint64_t sequence, double stamp,
                       double ros, double steady, bool current_peer, double budget = target_timeout) {
        poll(ros, steady);
        if (!initialized()) return "initialization_incomplete_or_failed";
        if (!feedback_fresh(ros, steady)) return "feedback_not_ready";
        if (!current_peer) return "peer_instance_or_baseline_changed";
        if (!session || !sequence || !std::isfinite(stamp) || stamp <= 0 ||
            !std::isfinite(budget) || budget <= 0 || budget > target_timeout ||
            ros < stamp || ros-stamp >= budget) return "invalid_session_or_expired_target";
        if (stamp < ready_at_) return "target_predates_feedback_baseline";
        if (session_ == session && stamp < target_source_) return "target_time_regressed";
        if (retired_.count(session)) return "new_session_required";
        if (session_ && session != session_) return "session_busy";
        if (sequence <= sequence_) return "target_sequence_not_advancing";
        session_ = session; sequence_ = sequence;
        target_source_ = stamp; target_at_ = steady-(ros-stamp); budget_ = budget; active_ = true;
        return {};
    }
    bool executing(double ros, double steady) { return status(ros, steady).state == State::Executing; }
};
// Diagnostics share this decision in native tests and the ROS graph adapter.
inline std::string endpoint_reason(unsigned expected, unsigned total, bool type, bool qos) {
    if (!expected) return total ? "unexpected_publisher" : "publisher_missing";
    if (expected != 1 || total != 1) return "publisher_ambiguous";
    if (!type) return "message_type_mismatch";
    if (!qos) return "qos_incompatible_or_unknown";
    return {};
}
class LogChanges {
    std::string previous_;
    double last_ = -1e30;
public:
    bool due(const std::string& value, double steady) {
        if (value != previous_ || steady < last_ || steady-last_ >= 5.) {
            previous_ = value; last_ = steady; return true;
        }
        return false;
    }
};
} // namespace snowbot_connection

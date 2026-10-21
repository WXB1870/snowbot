#pragma once
#include "bindu_connection_state.h"
#include <algorithm>
#include <iterator>
#include <atomic>
#include <chrono>
#include <future>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>

namespace snowbot_connection {
inline double steady_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline uint64_t instance_id() {
    std::random_device random;
    return (uint64_t(random()) << 32 | random()) | 1;
}
// Escape diagnostic strings (paths/config errors) as JSON, without extra dependencies.
inline std::string json_string(const std::string& s) {
    std::ostringstream out; out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
        else out << c;
    }
    return out.str() + '"';
}
class Reporter {
    rclcpp::Node& node_;
    std::string expected_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    LogChanges logs_;
    bool topic_enabled_ = true;
public:
    Connection connection;
    const uint64_t instance = instance_id();
    std::string rejection, sample_peer;
    Reporter(rclcpp::Node& node, const std::string& role, const std::string& expected, bool topic_enabled = true)
        : node_(node), expected_(node.declare_parameter<std::string>("feedback_publisher_node", expected)) {
        topic_enabled_ = topic_enabled;
        if (topic_enabled_) publisher_ = node.create_publisher<std_msgs::msg::String>("bindu/"+role+"_state", 1);
        timer_ = node.create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
        publish();
    }
    struct Endpoint {std::string reason, peer;};
    template<class Subscription> Endpoint inspect(const Subscription& subscription, const std::string& type, const std::string& expected_override = "") {
        const auto infos = node_.get_publishers_info_by_topic(subscription->get_topic_name());
        unsigned count = 0;
        std::string peer;
        bool type_ok = false, qos_ok = false, hash_known = false, hash_equal = false;
        const auto readers = node_.get_subscriptions_info_by_topic(subscription->get_topic_name());
        for (const auto& info : infos) {
            auto ns = info.node_namespace();
            if (ns.empty() || ns.back() != '/') ns += '/';
            if (ns+info.node_name() != (expected_override.empty() ? expected_ : expected_override)) continue;
            ++count;
            std::ostringstream gid;
            for (auto c : info.endpoint_gid()) gid << std::hex << std::setw(2) << std::setfill('0') << int(c);
            peer = gid.str();
            type_ok = info.topic_type() == type;
            for (const auto& reader : readers) {
                if (reader.node_name() != node_.get_name() || reader.node_namespace() != node_.get_namespace() ||
                    reader.topic_type() != type) continue;
                const auto& offered = info.topic_type_hash();
                const auto& requested = reader.topic_type_hash();
                hash_known = offered.version != 0 && requested.version != 0;
                hash_equal = hash_known && offered.version == requested.version &&
                    std::equal(std::begin(offered.value), std::end(offered.value), std::begin(requested.value));
            }
            qos_ok = rclcpp::qos_check_compatible(info.qos_profile(), subscription->get_actual_qos())
                .compatibility == rclcpp::QoSCompatibility::Ok;
        }
        auto reason = endpoint_reason(count, infos.size(), type_ok, qos_ok);
        if (reason.empty() && !hash_known) reason = "message_type_hash_unavailable";
        else if (reason.empty() && !hash_equal) reason = "message_type_hash_mismatch";
        return {reason,peer};
    }
    template<class Subscription> void graph(const Subscription& subscription, const std::string& type) {
        const auto result = inspect(subscription,type);
        connection.endpoint(result.reason,result.peer+":"+sample_peer);
    }
    void publish() {
        const double steady = steady_now();
        const auto state = connection.status(node_.now().seconds(), steady);
        std::ostringstream out;
        out << std::setprecision(17) << "{\"schema\":1,\"stamp\":" << node_.now().seconds()
            << ",\"instance_id\":" << instance << ",\"baseline_epoch\":" << connection.epoch()
            << ",\"state\":" << json_string(name(state.state))
            << ",\"reason\":" << json_string(state.reason)
            << ",\"feedback_valid\":" << (state.feedback_valid ? "true" : "false")
            << ",\"measurement_status\":" << json_string(state.feedback_valid ? "反馈有效" : (connection.sample() ? "反馈无效或陈旧" : "尚无反馈"))
            << ",\"pdo_sample\":" << connection.sample()
            << ",\"measurement_stamp\":" << connection.sample_stamp()
            << ",\"session\":" << connection.session()
            << ",\"last_target_rejection\":" << json_string(rejection) << '}';
        std_msgs::msg::String msg; msg.data = out.str(); if (topic_enabled_) publisher_->publish(msg);
        const std::string event = std::string(name(state.state))+":"+state.reason+":"+rejection;
        if (logs_.due(event, steady)) RCLCPP_INFO(node_.get_logger(), "%s", msg.data.c_str());
    }
};
// Keep the executor alive during configuration/model loading. All control callbacks
// gate on initialized(); only the initializing thread touches model/config members.
// Join before changing Connection or running the controller: no shared-state writes.
// The Python-backed solver stays on the original thread (GIL ownership unchanged).
template<class Initialize> void initialize(rclcpp::Node& node, Reporter& state, Initialize load) {
    if (state.connection.failed()) { state.publish(); return; }
    std::atomic<bool> done{false};
    std::exception_ptr spin_error;
    std::thread communication([&]() {
        try {
            while (!done.load() && rclcpp::ok()) {
                rclcpp::spin_some(node.get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (...) { spin_error = std::current_exception(); }
    });
    std::string error;
    try { load(); }
    catch (const std::exception& e) { error = e.what(); }
    catch (...) { error = "unknown_configuration_or_model_error"; }
    done.store(true); communication.join();
    if (spin_error && error.empty()) {
        try { std::rethrow_exception(spin_error); }
        catch (const std::exception& e) { error = std::string("executor_initialization: ")+e.what(); }
    }
    if (error.empty()) state.connection.initialize();
    else state.connection.fail(error);
    state.publish();
}
} // namespace snowbot_connection

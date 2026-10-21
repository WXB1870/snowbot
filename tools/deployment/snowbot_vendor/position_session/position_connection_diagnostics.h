#pragma once
#include <algorithm>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <rclcpp/qos.hpp>

// Read-only graph diagnostics; no state machine, publishers, timers or devices.
namespace position_connection {
struct EndpointCheck {
    std::string reason;
    std::string publisher_gid;
    bool ready() const { return reason.empty(); }
};
template<class Node, class Subscription>
EndpointCheck check_publisher(Node& node, const Subscription& sub,
                              const std::string& expected_node, const std::string& expected_type) {
    const auto writers = node.get_publishers_info_by_topic(sub->get_topic_name());
    if (writers.empty()) return {"publisher_missing", {}};
    if (writers.size() != 1) return {"publisher_ambiguous", {}};
    const auto& writer = writers.front();
    std::string ns = writer.node_namespace();
    if (ns.empty() || ns.back() != '/') ns += '/';
    if (ns+writer.node_name() != expected_node) return {"publisher_identity_mismatch", {}};
    if (writer.topic_type() != expected_type) return {"message_type_mismatch", {}};
    std::ostringstream gid;
    for (auto byte : writer.endpoint_gid())
        gid << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    const auto readers = node.get_subscriptions_info_by_topic(sub->get_topic_name());
    bool found = false;
    for (const auto& reader : readers) {
        if (reader.node_name() != node.get_name() || reader.node_namespace() != node.get_namespace() ||
                reader.topic_type() != expected_type) continue;
        found = true;
        const auto& offered = writer.topic_type_hash();
        const auto& requested = reader.topic_type_hash();
        if (!offered.version || !requested.version)
            return {"message_type_hash_unavailable", gid.str()};
        if (offered.version != requested.version ||
                !std::equal(std::begin(offered.value), std::end(offered.value), std::begin(requested.value)))
            return {"message_type_hash_mismatch", gid.str()};
    }
    if (!found) return {"subscription_not_discovered", gid.str()};
    const auto qos = rclcpp::qos_check_compatible(writer.qos_profile(), sub->get_actual_qos());
    if (qos.compatibility == rclcpp::QoSCompatibility::Error)
        return {"qos_incompatible: "+qos.reason, gid.str()};
    if (qos.compatibility != rclcpp::QoSCompatibility::Ok)
        return {"qos_compatibility_unknown: "+qos.reason, gid.str()};
    return {{}, gid.str()};
}
} // namespace position_connection

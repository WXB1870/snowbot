#pragma once
#include <algorithm>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <rclcpp/qos.hpp>

// Read-only graph diagnostics; no state machine, publishers, timers or devices.
namespace position_connection {
struct EndpointCheck {
    std::string reason;
    std::string publisher_gid;
    bool type_hash_verified = false;
    bool ready() const { return reason.empty(); }
};
template<class Endpoint, class = void>
struct HasTypeHash : std::false_type {};
template<class Endpoint>
struct HasTypeHash<Endpoint, std::void_t<decltype(std::declval<const Endpoint&>().topic_type_hash())>>
    : std::true_type {};
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
    bool type_hash_verified = false;
    for (const auto& reader : readers) {
        if (reader.node_name() != node.get_name() || reader.node_namespace() != node.get_namespace() ||
                reader.topic_type() != expected_type) continue;
        found = true;
        // Humble exposes no endpoint type hash. Keep that limitation explicit;
        // type name, identity, GID and QoS checks still apply on that API.
        if constexpr (HasTypeHash<std::decay_t<decltype(writer)>>::value) {
            const auto& offered = writer.topic_type_hash();
            const auto& requested = reader.topic_type_hash();
            if (!offered.version || !requested.version)
                return {"message_type_hash_unavailable", gid.str()};
            if (offered.version != requested.version ||
                    !std::equal(std::begin(offered.value), std::end(offered.value), std::begin(requested.value)))
                return {"message_type_hash_mismatch", gid.str()};
            type_hash_verified = true;
        }
    }
    if (!found) return {"subscription_not_discovered", gid.str()};
    const auto qos = rclcpp::qos_check_compatible(writer.qos_profile(), sub->get_actual_qos());
    if (qos.compatibility == rclcpp::QoSCompatibility::Error)
        return {"qos_incompatible: "+qos.reason, gid.str(), type_hash_verified};
    if (qos.compatibility != rclcpp::QoSCompatibility::Ok)
        return {"qos_compatibility_unknown: "+qos.reason, gid.str(), type_hash_verified};
    return {{}, gid.str(), type_hash_verified};
}
} // namespace position_connection

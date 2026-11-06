#include "position_connection_diagnostics.h"
#include "position_session_ros.h"
#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
static unsigned checks=0;
#define CHECK(c) do{++checks;if(!(c))throw std::runtime_error(#c);}while(0)
struct Endpoint {
 std::string node="servo_node",ns="/",type="snowbot_ctl_msgs/msg/PositionObservation";
 rclcpp::QoS qos;
 std::string node_name()const{return node;}std::string node_namespace()const{return ns;}
 std::string topic_type()const{return type;}
 rclcpp::QoS qos_profile()const{return qos;}
 std::array<uint8_t,24> endpoint_gid()const{std::array<uint8_t,24> id{};id[0]=17;return id;}
};
struct Sub {const char* get_topic_name(){return "/bindu/position/observation";}rclcpp::QoS get_actual_qos(){return {};}};
struct Node {
 std::vector<Endpoint> writers,readers;unsigned graph_reads=0;
 unsigned legacy_writers=0;
 const char* get_name(){return "upper";}const char* get_namespace(){return "/";}
 auto get_publishers_info_by_topic(const char*){++graph_reads;return writers;}
 auto get_subscriptions_info_by_topic(const char*){++graph_reads;return readers;}
 unsigned count_publishers(const char*){return legacy_writers;}
};
int main(){try{
 Node node;auto sub=std::make_shared<Sub>();
 const auto check=[&](){return position_connection::check_publisher(node,sub,"/servo_node","snowbot_ctl_msgs/msg/PositionObservation");};
 CHECK(check().reason=="publisher_missing");
 node.writers.push_back({});CHECK(check().reason=="subscription_not_discovered");
 node.readers.push_back({});node.readers[0].node="upper";
 CHECK(check().ready());CHECK(check().publisher_gid.size()==48);CHECK(check().publisher_gid.substr(0,2)=="11");
 node.writers.push_back({});CHECK(check().reason=="publisher_ambiguous");node.writers.pop_back();
 node.writers[0].node="_NODE_NAME_UNKNOWN_";CHECK(check().reason=="publisher_discovering");node.writers[0].node="servo_node";
 node.writers[0].ns="_NODE_NAMESPACE_UNKNOWN_";CHECK(check().reason=="publisher_discovering");node.writers[0].ns="/";
 node.writers[0].node="observer";CHECK(check().reason=="publisher_identity_mismatch");node.writers[0].node="servo_node";
 node.writers[0].ns="/snowbot";CHECK(check().reason=="publisher_identity_mismatch");node.writers[0].ns="/";
 node.writers[0].type="std_msgs/msg/String";CHECK(check().reason=="message_type_mismatch");node.writers[0].type=node.readers[0].type;
 node.readers[0].node="another_subscriber";CHECK(check().reason=="subscription_not_discovered");node.readers[0].node="upper";
 node.writers[0].qos.result=rclcpp::QoSCompatibility::Error;CHECK(check().reason.find("qos_incompatible:")==0);
 node.writers[0].qos.result=rclcpp::QoSCompatibility::Warning;CHECK(check().reason.find("qos_compatibility_unknown:")==0);
 node.writers[0].qos.result=rclcpp::QoSCompatibility::Ok;CHECK(check().ready());CHECK(node.graph_reads>20);
 position_session::CommandGraph graph;int64_t now=0;
 const auto command=[&](){now+=100000001;return graph.check(node,"/bindu/position/servo","dual_arm_control_node","snowbot_ctl_msgs/msg/PositionSessionServo",now);};
 node.writers.clear();CHECK(std::string(command())=="command_source_discovering");
 node.writers.push_back({});node.writers[0].node="_NODE_NAME_UNKNOWN_";CHECK(std::string(command())=="command_source_discovering");
 node.writers[0].node="dual_arm_control_node";CHECK(std::string(command())=="command_source_conflict");
 node.writers[0].type="snowbot_ctl_msgs/msg/PositionSessionServo";CHECK(command()==nullptr);
 node.writers[0].ns="/other";CHECK(std::string(command())=="command_source_conflict");node.writers[0].ns="/";
 node.writers.push_back(node.writers[0]);CHECK(std::string(command())=="command_source_conflict");node.writers.pop_back();
 node.legacy_writers=1;CHECK(std::string(command())=="legacy_source_conflict");
 const auto reads=node.graph_reads;
 CHECK(std::string(graph.check(node,"/bindu/position/servo","dual_arm_control_node","snowbot_ctl_msgs/msg/PositionSessionServo",now+1))=="legacy_source_conflict");
 CHECK(node.graph_reads==reads);
 std::cout<<checks<<" endpoint diagnostic assertions passed\n";
 }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}

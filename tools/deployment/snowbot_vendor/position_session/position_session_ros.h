#pragma once
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>
namespace position_session {
// The receiver authenticates its own command endpoint. Missing DDS metadata is
// discovery, not a device failure; readiness is retried before any takeover.
template<class Node> const char* command_source(Node& node,const char* topic,const char* writer,const char* type){
 const auto infos=node.get_publishers_info_by_topic(topic);
 if(infos.empty())return "command_source_discovering";
 if(infos.size()==1&&(infos[0].node_name()=="_NODE_NAME_UNKNOWN_"||
                     infos[0].node_namespace()=="_NODE_NAMESPACE_UNKNOWN_"))return "command_source_discovering";
 if(infos.size()!=1||infos[0].node_name()!=writer||infos[0].topic_type()!=type||
    (std::string(writer)!="execution"&&infos[0].node_namespace()!="/"))return "command_source_conflict";
 if(std::string(topic)=="/bindu/position/servo"&&
    (node.count_publishers("/joint_position_cmd")||node.count_publishers("/body/motor_cmd")))return "legacy_source_conflict";
 return nullptr;
}
class CommandGraph {
 int64_t next_check_=0;
 const char* reason_="command_source_discovering";
public:
 template<class Node> const char* check(Node& node,const char* topic,const char* writer,const char* type,
   int64_t now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()){
  // Preserve the existing Python GraphSnapshot's 100 ms discovery cadence.
  if(now<next_check_)return reason_;
  next_check_=now+100000000;reason_=command_source(node,topic,writer,type);return reason_;
 }
};
// Communication runs while the original thread loads models/configuration.
// Callbacks reject commands until initialized. Only the communication thread
// accesses feedback/state; join precedes committing initialization results.
// Python solver construction remains on its original thread (GIL affinity).
template<class Load> std::string initialize(rclcpp::Node& node,Load load){
 std::atomic<bool> done{false};std::exception_ptr executor_error;
 std::thread communication([&]{try{while(!done.load() && rclcpp::ok()){
  rclcpp::spin_some(node.get_node_base_interface());std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }}catch(...){executor_error=std::current_exception();}});
 std::string error;
 try{error=load();}catch(const std::exception& e){error=e.what();}catch(...){error="configuration_or_model_exception";}
 done.store(true);communication.join();
 if(executor_error && error.empty())try{std::rethrow_exception(executor_error);}catch(const std::exception& e){error=e.what();}
 return error;
}
} // namespace position_session

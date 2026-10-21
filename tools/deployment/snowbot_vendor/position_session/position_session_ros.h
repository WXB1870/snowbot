#pragma once
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <exception>
#include <string>
#include <thread>
namespace position_session {
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

#include "position_session_ros.h"
#include <iostream>
#include <stdexcept>
static unsigned checks=0;
#define CHECK(c) do{++checks;if(!(c))throw std::runtime_error(#c);}while(0)
static void wait_callbacks(rclcpp::Node& node) {
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
 while(node.callbacks.load()<20 && std::chrono::steady_clock::now()<deadline)
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
 if(node.callbacks.load()<20)throw std::runtime_error("callbacks_blocked_during_model_load");
}
int main(){try{
 const auto caller=std::this_thread::get_id();
 rclcpp::Node missing;
 const auto error=position_session::initialize(missing,[&](){CHECK(std::this_thread::get_id()==caller);wait_callbacks(missing);return std::string("model_missing: /no/snowbot.urdf");});
 CHECK(error=="model_missing: /no/snowbot.urdf");CHECK(missing.callbacks>=20);CHECK(missing.state_updates>=20);
 const unsigned after_join=missing.callbacks.load();std::this_thread::sleep_for(std::chrono::milliseconds(3));CHECK(missing.callbacks==after_join);
 rclcpp::spin_some(missing.get_node_base_interface());CHECK(missing.callbacks==after_join+1);
 rclcpp::Node good;CHECK(position_session::initialize(good,[&](){wait_callbacks(good);return std::string{};}).empty());
 CHECK(good.callbacks>=20);
 rclcpp::Node exception;const auto failure=position_session::initialize(exception,[]()->std::string{throw std::runtime_error("configuration_missing: /no/config.yaml");});
 CHECK(failure=="configuration_missing: /no/config.yaml");
 std::cout<<checks<<" initialization communication assertions passed\n";
 }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}

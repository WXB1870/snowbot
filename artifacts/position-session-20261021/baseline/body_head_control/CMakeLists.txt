cmake_minimum_required(VERSION 3.8)
project(body_head_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(body_head_control_node src/body_head_control_node.cpp)
ament_target_dependencies(body_head_control_node rclcpp std_msgs sensor_msgs)

install(TARGETS body_head_control_node DESTINATION lib/${PROJECT_NAME})

ament_package()

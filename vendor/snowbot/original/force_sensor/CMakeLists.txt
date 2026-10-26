cmake_minimum_required(VERSION 3.8)
project(snowbot_force_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(snowbot_force_sensor_node src/force_sensor_node.cpp)
ament_target_dependencies(snowbot_force_sensor_node
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs ament_index_cpp)
target_link_libraries(snowbot_force_sensor_node yaml-cpp)

install(TARGETS snowbot_force_sensor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

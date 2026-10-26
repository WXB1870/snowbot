cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(snowbot_ctl_msgs REQUIRED)

# tf2 stack (used by msgPublisher.h / global_state.hpp)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ament_index_cpp REQUIRED)

# nlohmann_json (header-only, used by chassis CAN frames)
find_package(nlohmann_json REQUIRED)

# Pinocchio (installed under /opt/openrobots)
set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio")
set(casadi_DIR "/opt/openrobots/lib/cmake/casadi")
set(coal_DIR "/opt/openrobots/lib/cmake/coal")
set(hpp-fcl_DIR "/opt/openrobots/lib/cmake/hpp-fcl")
find_package(pinocchio REQUIRED)

# ---------------------------------------------------------------------------
# Generate this package's own msg / srv / action interfaces.
# The package name is "robot_control", so the C++ namespace will be
# robot_control::msg / robot_control::srv / robot_control::action, matching
# the existing source code.
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BMSAlarmWarning.msg"
  "msg/BMSStatus.msg"
  "msg/BackWheelOdometry.msg"
  "msg/ChassisMotion.msg"
  "msg/FaultBit.msg"
  "msg/FrontWheelOdometry.msg"
  "msg/LifterCmd.msg"
  "msg/LightControlStatus.msg"
  "msg/MotionModeFeedback.msg"
  "msg/MotorHigh.msg"
  "msg/MotorHighArray.msg"
  "msg/MotorLow.msg"
  "msg/MotorLowArray.msg"
  "msg/RemoteControlStatus.msg"
  "msg/SteeringAngles.msg"
  "msg/SystemStatus.msg"
  "msg/SystemStatusArray.msg"
  "msg/WheelSpeeds.msg"
  "srv/SavePreset.srv"
  "action/MoveToPreset.action"
  DEPENDENCIES std_msgs
)

# Modern Jazzy typesupport target for linking against this package's
# OWN generated interfaces (rosidl_get_typesupport_target only works
# for targets created by rosidl_generate_interfaces in the SAME
# package; for cross-package message packages like snowbot_ctl_msgs we
# rely on ament_target_dependencies which handles the exported
# imported targets automatically).
rosidl_get_typesupport_target(own_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 包含头文件路径
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_compile_definitions(
    __GLIBC_USE_IEC_60559_BFP_EXT=1
    __GLIBC_USE_ISOC2X=0
    _GNU_SOURCE=1
)

add_definitions(-DEIGEN_AVOID_COMPLEX_VECTORIZATION)

add_executable(robot_control_node
  #主循环
  src/robot_control_node.cpp

  src/controller/demoTraj.cpp
  src/controller/upper_Transmit.cpp
  src/controller/xrTelepercationCom.cpp

  src/messageType/upper_msgType.cpp

  src/robot_control/msgPublisher.cpp
  src/robot_control/msgType.cpp
  src/robot_control/trajProcessWrap.cpp
  src/robot_control/config.cpp
)

target_include_directories(robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(robot_control_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(robot_control_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  snowbot_ctl_msgs
  Boost
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_kdl
  urdf
  kdl_parser
  robot_state_publisher
  ament_index_cpp
)

target_link_libraries(robot_control_node pthread rt yaml-cpp
  tbb
  pinocchio::pinocchio
  nlohmann_json::nlohmann_json
  ${own_typesupport_target}
)

target_compile_definitions(robot_control_node PRIVATE
    "EIGEN_USE_STD_ABS=1"
)

install(TARGETS robot_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()

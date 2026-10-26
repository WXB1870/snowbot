cmake_minimum_required(VERSION 3.8)
project(snowbot_dual_arm_control)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(snowbot_ctl_msgs REQUIRED)
find_package(dual_arm REQUIRED)

# tf2 stack (used by msgPublish.h)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加python支持
find_package(Python3 REQUIRED COMPONENTS Development NumPy)

# Pinocchio (installed under /opt/openrobots)
set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio")
set(casadi_DIR "/opt/openrobots/lib/cmake/casadi")
set(coal_DIR "/opt/openrobots/lib/cmake/coal")
set(hpp-fcl_DIR "/opt/openrobots/lib/cmake/hpp-fcl")
find_package(pinocchio REQUIRED)

# NOTE: rosidl_get_typesupport_target only works for targets created
# by rosidl_generate_interfaces in the SAME package.  For the
# cross-package snowbot_ctl_msgs messages we rely on
# ament_target_dependencies, which handles the exported imported
# typesupport targets automatically (replaces the old raw
# "${cpp_typesupport_target}" / rosidl_target_interfaces usage).

# 安装环境钩子
install(
  DIRECTORY env_hooks/
  DESTINATION share/${PROJECT_NAME}/env_hooks
)

# 包含头文件路径
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

add_compile_definitions(
    __GLIBC_USE_IEC_60559_BFP_EXT=1
    __GLIBC_USE_ISOC2X=0
    _GNU_SOURCE=1
)

add_definitions(-DEIGEN_AVOID_COMPLEX_VECTORIZATION)

#完整版的主程序
add_executable(dual_arm_control_node

src/dual_arm_control_node.cpp

src/dual_arm_control/msgPublish.cpp

src/robot_config/config.cpp

#添加python支持
src/controller/ik_solver_wrapper.cpp

src/controller/JntController.cpp
src/controller/ArmController.cpp
src/controller/ArmInterface.cpp
src/controller/IKFKsolver.cpp
)

target_include_directories(dual_arm_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dual_arm_control_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(dual_arm_control_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  snowbot_ctl_msgs
  Boost
  tf2
  tf2_ros
  tf2_kdl
  urdf
  kdl_parser
  robot_state_publisher
  ament_index_cpp
)

target_link_libraries(dual_arm_control_node pthread rt yaml-cpp
  tbb
  pinocchio::pinocchio
  ${Python3_LIBRARIES}
)

target_compile_definitions(dual_arm_control_node PRIVATE
    "EIGEN_USE_STD_ABS=1"
)

install(TARGETS dual_arm_control_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 scripts 文件
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(snowbot_servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# robotpkg 安装的 pinocchio 在 /opt/openrobots
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(snowbot_ctl_msgs REQUIRED)
find_package(kinematic_pose_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_definitions(-DEIGEN_USE_CUSTOM_ABS)

# 包含头文件路径
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  thirdparty/soem/soem
  thirdparty/soem/osal
  thirdparty/soem/osal/linux
  thirdparty/soem/oshw/linux
)

add_compile_definitions(
    __GLIBC_USE_IEC_60559_BFP_EXT=1
    __GLIBC_USE_ISOC2X=0
    _GNU_SOURCE=1
)

add_definitions(-DEIGEN_AVOID_COMPLEX_VECTORIZATION)

# 编译SOEM库
add_library(soem STATIC
  thirdparty/soem/soem/ethercatbase.c
  thirdparty/soem/soem/ethercatcoe.c
  thirdparty/soem/soem/ethercatconfig.c
  thirdparty/soem/soem/ethercatdc.c
  thirdparty/soem/soem/ethercateoe.c
  thirdparty/soem/soem/ethercatfoe.c
  thirdparty/soem/soem/ethercatmain.c
  thirdparty/soem/soem/ethercatprint.c
  thirdparty/soem/soem/ethercatsoe.c
  thirdparty/soem/osal/linux/osal.c
  thirdparty/soem/oshw/linux/nicdrv.c
  thirdparty/soem/oshw/linux/oshw.c
)

# 完整版本的主程序
add_executable(servo_node
  src/main.cpp

  src/servo_control/zero_pos_config.cpp
  src/servo_control/transmit.cpp
  src/servo_control/ethercat_driver.cpp

  src/ros_controller/rosLoop.cpp
  src/ros_controller/rosLoopMsgDefine.cpp

  src/JntController/runinController.cpp
  src/JntController/JntTestCtl.cpp
  src/JntController/ForceComp.cpp
  src/JntController/retZeroControl.cpp

  src/impedance/cartesian_impedance_controller.cpp
  src/impedance/dual_arm_coordinator.cpp
)

ament_target_dependencies(servo_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  snowbot_ctl_msgs
  kinematic_pose_msgs
  Boost
  ament_index_cpp
)

# NOTE: rosidl_get_typesupport_target only works for targets created
# by rosidl_generate_interfaces in the SAME package.  For the
# cross-package snowbot_ctl_msgs messages we rely on
# ament_target_dependencies above, which handles the exported
# imported typesupport targets automatically (replaces the old
# undefined "${cpp_typesupport_target}" usage).

target_link_libraries(servo_node soem pthread rt yaml-cpp
  tbb
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_compile_definitions(servo_node PRIVATE
    "EIGEN_USE_STD_ABS=1"
)

# 安装规则
install(TARGETS servo_node
  DESTINATION lib/${PROJECT_NAME})

# 将 include 目录安装到系统
install(DIRECTORY include/
  DESTINATION include
)

# 安装 Python 测试脚本 (阻抗控制). 每个 install(PROGRAMS) 只能 RENAME 一次,
# 故分多次写入, 安装后通过 ros2 run snowbot_servo_control <name> 调用.
install(PROGRAMS scripts/impedance_enable.py
  DESTINATION lib/${PROJECT_NAME} RENAME impedance_enable)
install(PROGRAMS scripts/impedance_status_monitor.py
  DESTINATION lib/${PROJECT_NAME} RENAME impedance_status_monitor)
install(PROGRAMS scripts/impedance_hold_test.py
  DESTINATION lib/${PROJECT_NAME} RENAME impedance_hold_test)
install(PROGRAMS scripts/impedance_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME} RENAME impedance_keyboard_teleop)

# 告诉其他依赖这个包的工程去哪里找头文件
ament_export_include_directories(include)

ament_package()

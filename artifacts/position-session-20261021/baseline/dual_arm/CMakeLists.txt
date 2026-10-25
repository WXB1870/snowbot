cmake_minimum_required(VERSION 3.8)
project(dual_arm)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息接口生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  DEPENDENCIES std_msgs
)

# 现代方式获取本包消息的 C++ 类型支持目标（替代已废弃的
# rosidl_target_interfaces）。
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 添加可执行文件
add_executable(moveit_to_joint_state
  src/moveit_to_joint_state.cpp
)

ament_target_dependencies(moveit_to_joint_state rclcpp sensor_msgs std_msgs)

# 链接本包生成的消息类型支持库
target_link_libraries(moveit_to_joint_state ${cpp_typesupport_target})

# 安装可执行文件
install(TARGETS
  moveit_to_joint_state
  DESTINATION lib/${PROJECT_NAME})

# 安装其他资源
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_ik_demo");
    
    // 创建异步spinner
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });

    moveit::planning_interface::MoveGroupInterface arm(node, "l_arm_manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("l_arm_home");
    arm.move();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 设置目标位置
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.4;

    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        arm.execute(my_plan);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("l_arm_home");
    arm.move();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::shutdown();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }

    return 0;
}

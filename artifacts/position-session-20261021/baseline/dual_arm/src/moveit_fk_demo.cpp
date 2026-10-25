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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_fk_demo");
    
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

    double targetPose[7] = {1.57, -0.676384, -0.376217, -1.1, 1.052834, 0.454125,1.1};
    std::vector<double> joint_group_positions(7);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];
    joint_group_positions[6] = targetPose[6];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
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

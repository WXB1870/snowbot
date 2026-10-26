#ifndef __chassis_control__H__
#define __chassis_control__H__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int8.hpp>
#include <memory>
#include <string>
#include <atomic>
#include <chrono>
#include <cmath>

#include "controller/chassis_control/can/chassis_service.h"
#include "controller/chassis_control/can/socket_can.h"
#include "controller/chassis_control/chassis.h"
#include "controller/chassis_control/serial_port.h"
#include "controller/chassis_control/elevator.h"
#include "robot_control/msg/lifter_cmd.hpp"
#include "robot_control/srv/save_preset.hpp"
#include "robot_control/action/move_to_preset.hpp"

/*
        by 霜云喵
================================
底盘通讯与控制汇总代码 -- 整体通讯架构
============修改日志============

==============协议==============

================================
*/

//----------完整的底盘控制器----------
//** 包含与小车和升降台的通讯 **

using MoveToPreset = robot_control::action::MoveToPreset;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveToPreset>;

class chassis_controller{
    private:
        // 常量（可按需调整）
        static constexpr int ERROR_MARGIN = 5;
        static constexpr int STOP_OFFSET = 3;

        // ROS 对象（由外部 node 提供）
        rclcpp::Node::SharedPtr node_;

        // 硬件/业务对象
        std::unique_ptr<SerialPort> serial_;
        std::unique_ptr<ElevatorController> elevator_;
        std::shared_ptr<agx::nav::ChassisService> m_chassis_service;
        std::shared_ptr<agx::nav::SocketCan> m_socket_can;

        // ROS 接口
        rclcpp::Subscription<robot_control::msg::LifterCmd>::SharedPtr lifter_cmd_sub_;
        rclcpp::Service<robot_control::srv::SavePreset>::SharedPtr save_preset_srv_;
        rclcpp_action::Server<MoveToPreset>::SharedPtr action_server_;

        // 状态
        std::atomic<bool> busy_{false};

        // 私有方法
        void cmd_callback(const robot_control::msg::LifterCmd::SharedPtr msg);
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                std::shared_ptr<const MoveToPreset::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);
        void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

    public:
        // 构造与析构
        explicit chassis_controller(rclcpp::Node::SharedPtr node);
        ~chassis_controller() = default;

        // 初始化（在构造后或外部调用）
        void init();

        // 直接控制接口（可被外部调用）
        void move_up();
        void move_down();
        void stop();
};

#endif
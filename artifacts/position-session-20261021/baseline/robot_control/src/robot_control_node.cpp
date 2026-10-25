#include <cstdio>
#include <iostream>

#include"robot_control/msgPublisher.h"

/*
        by 霜云喵
================================
通讯主函数代码（上层节点）
============修改日志============

==============协议==============
本代码为上位机节点主函数代码
控制流程为：
    1. 先获取机械臂关节返回值，据此计算本轮控制量
    2. 将计算出的控制量发送给下位机

本节点为整体的上位机节点，负责总体轨迹规划等任务。

================================
*/

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

	std::cout << "上位机控制程序启动" << std::endl;
	std::cout << "===================================" << std::endl;

	rclcpp::init(argc, argv);
	auto node = std::make_shared<RobotCtlNode>();

	node->run();
	rclcpp::shutdown();
    return 0;
}

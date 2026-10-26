#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "dual_arm_control/msgPublish.h"



/*
        by 霜云喵
================================
通讯主函数代码（中层节点）
============修改日志============

==============协议==============
本代码为上位机节点主函数代码
控制流程为：
    1. 先获取机械臂关节返回值，据此计算本轮控制量
    2. 将计算出的控制量发送给下位机

通讯成功后，当第一轮时，下位机发送的返回值为0,而且会返回一个首轮控制标志
当处于首轮控制时，或未能与下位机成功通讯，应当持续发送零控制量

==============通讯协议1==============

中层节点与下位机的通讯。下位机：关节通讯节点

传输信息：

1. 控制信息：
    temp_time -- 本中层节点的循环周期，用于与下层节点协调周期，并用于插补
    runin_mod -- 下位机（关节通讯节点）插补模式
        1 -- 线性插补
        2 -- 二次插补
    if_runin_renew -- 是否刷新下位机插补器
        0 -- 否
        1 -- 是
    if_useret_to_start_runin -- 是否使用返回值作开始插补时第一轮输入
        0 -- 否
        1 -- 是
    if_use_debugMod -- 是否使用调试模式
        0 -- 否
        1 -- 是
    l_position_at_runin_st 、r_position_at_runin_st -- 下发的插补开始时初始值

    place_hold -- 位置保持模式控制参量：
        1 -- 开启位置保持模式
        0 -- 关闭位置保持模式

2. 左右臂目标位置，速度、三类目标力矩、目标kp, kd值
    此处略，不再赘述。

3. reserve量（保留量）

**待修改 -- 是否启用调试模式，若启用调试模式，则不论是否连接机器人关节，下位机均只向着rviz2发送数据，若启用调试模式，则本节点不向着rviz发送数据**

==============通讯协议2==============

上层节点与中层节点通讯代码。（上层节点为上位机，用于规划等）

================================
*/

int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;

	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArmCtlNode>();

    node->run();
    rclcpp::shutdown();
	return 0;
}

#ifndef __chassis__H__
#define __chassis__H__

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
#include <cmath>
#include <iostream>

#include "messageType/chassis_msgType.h"

/*
        by 霜云喵
================================
底盘小车通讯代码
============修改日志============

==============协议==============

================================
*/

//----------底盘通讯代码----------

//底盘通讯类
class chassis_car{
    private:

    public:
        //辅助函数 -- 应用死区过滤

        //运动参数
        float max_linear_x_;         //前后最大线速度(m/s)
        float max_linear_y_;         //左右最大线速度(m/s)
        float max_angular_z_;       //最大角速度(rad/s)
    
    public:
        //构造函数
        chassis_car();

        //获取并发布底盘运动信息
        void get_and_publish_(double linear_x_, double linear_y_, double angualr_z_);
};

#endif
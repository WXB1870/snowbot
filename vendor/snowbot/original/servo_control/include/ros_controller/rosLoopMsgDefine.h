#ifndef __rosLoopMsgDefine__H__
#define __rosLoopMsgDefine__H__

#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <string>

#define TEMPTIME (0.001)        //迭代时间
#define SERVODOF (14)            //伺服控制自由度

//双臂自由度定义
#define LARMDOF (7)
#define RARMDOF (7)

#define ZERO_POS_THRESHOLD    (0.001) //回零触发阈值 (rad)
#define ZERO_POS_EXIT_THRESH  (0.001) //回零退出阈值 (rad)

//网卡名称定义
extern std::string netCard_name;

//方向协调代码

/*  定义结构体 -- CAN版配置结构
-------------------------------
协议：
    CAN板上连接的关节ID需要按照从CAN1到CAN2的顺序排列，必须保证ID与实际从站的关节对的上
*/

/*  定义结构体 -- 关节目标信息
------------------------------
    协议：
    transMod -- 传输协议
        1 -- 力位混合控制模式
        4 -- 位置模式
        5 -- 电流模式
    其余为关节传输变量，具体含义见关节手册
*/
struct joint_desMsg{
    int transMod;
    double joint_pos;
    double joint_spd;
    double joint_tor;
    double joint_Kp;
    double joint_Kd;
};

//定义结构体 -- 关节反馈信息
struct joint_retMsg{

    //基础的反馈值 -- 反馈位置、反馈速度、反馈电流
    double joint_posRet;
    double joint_spdRet;
    double joint_currentRet;

    //其他反馈值 -- 温度
    double joint_temperature;
};

//定义结构体 -- 夹爪控制指令 (transMod=1 力位混控)
struct gripperCmd{
    double joint_pos = 0.0;  // 目标角度 (rad)
    double joint_spd = 0.0;  // 目标速度 (rad/s)
    double joint_tor = 0.0;  // 目标力矩 (Nm)
    double joint_Kp  = 15.0; // 刚度
    double joint_Kd  = 10.0; // 阻尼
};

extern gripperCmd left_gripper_cmd;
extern gripperCmd right_gripper_cmd;

// 夹爪反馈
struct gripperFdb{
    double pos_deg = 0.0;    // 实际角度 (deg)
    double current_A = 0.0;  // 实际电流 (A)
};
extern gripperFdb left_gripper_fdb;
extern gripperFdb right_gripper_fdb;

//定义结构体 -- ECAT-8CAN 通道配置
struct can_slave_struct{
    int channel_idx;                // CAN通道索引 (0-7)
    std::vector<int> motor_slot;    // 每个关节在通道内的motor slot (0-7)

    int can_num_activate;           //使用的can通道数目(1或者2)
    std::vector<int> can_id;        //对应使用的CAN ID

    std::vector<int> can_slave_num; //每一路CAN上面挂载的从站数量

    int JntNum_onCan;               //CAN板上连接的关节的数目
    std::vector<int> can_jnt_ID;    //CAN板上连接的关节的ID
    std::vector<int> can_jnt_direComp;          //CAN板上方向补偿
    std::vector<double> can_jnt_Kt;             //CAN板上所有关节扭矩常数
    std::vector<joint_desMsg> can_jnt_des_msg;  //CAN板上所有关节目标信息
    std::vector<joint_retMsg> can_jnt_ret_msg;  //CAN板上所有关节反馈信息
};

//定义结构体 -- 下位机结构
struct servo_struct{
    int CAN_board_num;
    std::vector<can_slave_struct> CAN_boards;
};


/* 定义函数 -- 初始化下位机结构
----------------------------------------------
从.yaml文件读取从站配置，并装入到下位机结构中并返回
并以此为指导初始化下位机结构
*/
std::string get_yaml_path();
servo_struct load_servo_config(const std::string& filepath);

#endif
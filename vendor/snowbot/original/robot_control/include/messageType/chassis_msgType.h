#ifndef __chassis_msgType__H__
#define __chassis_msgType__H__

//----------底盘通讯用的消息类型----------

// 升降台控制指令消息
struct lifter_control_msg{

    // --- 升降台控制部分 ---
    int control_mod;    //控制模式： 0 -- 遥操作； 1 -- 预设点控制
    int move_cmd;       //遥操作指令： 0 -- 停止； 1 -- 向上； -1 -- 向下
    int target_position; // 预设点控制的目标位置 -- 设定为0,则为移动到记忆点
    bool save_preset;    // 是否保存当前高度为预设点

};

// 底盘控制指令消息
struct chassis_control_msg{
    double linear_x;   // 前进速度 (m/s)
    double linear_y;   // 横向移动速度 (m/s)（用于全向移动）
    double angular_z;  // 旋转速度 (rad/s)
};

// 整体控制消息：包含升降台控制和底盘控制
struct base_control_msg{

    // --- 升降台控制部分 ---
    lifter_control_msg lifter_cmd;  // 升降台控制指令

    // --- 底盘控制部分 ---
    chassis_control_msg chassis_cmd; // 底盘控制指令

};

#endif
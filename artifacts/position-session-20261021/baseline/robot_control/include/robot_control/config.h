#ifndef __config__H__
#define __config__H__

#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iomanip>

#include "robot_control/msgType.h"

/*
        by 霜云喵
================================
配置代码，配置零位和PD参数
============修改日志============

==============协议==============

================================
*/




extern std::string yaml_path_zeroPos;
extern std::string yaml_path_desc_zeroPos;
extern std::string yaml_path_initialPD;

//默认位置定义
extern std::vector<double> lArm_zeroPos;
extern std::vector<double> rArm_zeroPos;

extern std::vector<double> l_descZeroPos;
extern std::vector<double> r_descZeroPos;

//默认PD参数定义
extern std::vector<double> lArm_defaultKp_low;
extern std::vector<double> lArm_defaultKd_low;
extern std::vector<double> rArm_defaultKp_low;
extern std::vector<double> rArm_defaultKd_low;

extern std::vector<double> lArm_defaultKp_mid;
extern std::vector<double> lArm_defaultKd_mid;
extern std::vector<double> rArm_defaultKp_mid;
extern std::vector<double> rArm_defaultKd_mid;

extern std::vector<double> lArm_defaultKp_high;
extern std::vector<double> lArm_defaultKd_high;
extern std::vector<double> rArm_defaultKp_high;
extern std::vector<double> rArm_defaultKd_high;

extern std::vector<double> lArm_defaultKp_superhigh;
extern std::vector<double> lArm_defaultKd_superhigh;
extern std::vector<double> rArm_defaultKp_superhigh;
extern std::vector<double> rArm_defaultKd_superhigh;

extern std::vector<double> lArm_defaultKp_tag;
extern std::vector<double> lArm_defaultKd_tag;
extern std::vector<double> rArm_defaultKp_tag;
extern std::vector<double> rArm_defaultKd_tag;


/*  机械臂零位配置读取类
---------------------------
功能：
    1. 从 zeropos.yaml 读取双臂零位关节角度
    2. 提供获取左/右臂零位向量的接口
    3. 打印默认位置到控制台

协议：
    零位定义为双臂竖直下垂时的关节位置，
    角度单位为弧度 (rad)，关节顺序与 Unitree_name_list 一致
*/
class ZeroPosConfig{
    private:
        // 零位位置数据
        std::vector<double> lArm_zeroPos;               // 左臂零位 (7 关节)
        std::vector<double> rArm_zeroPos;               // 右臂零位 (7 关节)

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        ZeroPosConfig();
        ZeroPosConfig(const std::string& yaml_path);

        // 从 YAML 文件加载零位配置
        // 返回 true 表示加载成功
        bool loadFromYaml(const std::string& yaml_path);

        // 获取零位位置
        std::vector<double> getLeftArmZeroPos()  const;
        std::vector<double> getRightArmZeroPos() const;

        // 打印默认位置到控制台
        void printDefaultPositions() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

/*  笛卡尔空间零位配置读取类
--------------------------------
功能：
    1. 从 desc_zeropos.yaml 读取双臂笛卡尔空间零位
    2. 提供获取左/右臂末端位姿向量的接口
    3. 打印默认位置到控制台

协议：
    位姿格式：[x, y, z, roll, pitch, yaw]
    - x, y, z  — 末端位置 (m)
    - roll, pitch, yaw — RPY 欧拉角 (rad)
*/
class DescZeroPosConfig{
    private:
        // 笛卡尔零位数据 [x,y,z,roll,pitch,yaw]
        std::vector<double> lArm_descPos;           // 左臂末端零位 (6 维)
        std::vector<double> rArm_descPos;           // 右臂末端零位 (6 维)

        // YAML 文件路径
        std::string yaml_path_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        DescZeroPosConfig();
        DescZeroPosConfig(const std::string& yaml_path);

        // 从 YAML 文件加载笛卡尔零位配置
        bool loadFromYaml(const std::string& yaml_path);

        // 获取笛卡尔零位
        std::vector<double> getLeftArmDescZeroPos()  const;
        std::vector<double> getRightArmDescZeroPos() const;

        // 打印默认位置到控制台
        void printDefaultPositions() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};


/*  机械臂 PD 参数读取类
---------------------------
功能：
    1. 从 initialPD.yaml 读取双臂 PD 控制器增益参数
    2. 通过刚度等级控制变量（low/mid/high/superhigh/tag）选择对应参数组
    3. 提供获取左/右臂 Kp、Kd 向量的接口
    4. 打印当前 PD 配置到控制台

协议：
    关节顺序与 Unitree_name_list 一致（7 关节），
    Kp 为比例增益，Kd 为微分增益
*/
class InitialPDConfig{
    private:
        // PD 参数数据
        std::vector<double> lArm_Kp;                // 左臂比例增益 (7 关节)
        std::vector<double> lArm_Kd;                // 左臂微分增益 (7 关节)
        std::vector<double> rArm_Kp;                // 右臂比例增益 (7 关节)
        std::vector<double> rArm_Kd;                // 右臂微分增益 (7 关节)

        // YAML 文件路径
        std::string yaml_path_;

        // 当前刚度等级
        std::string rigidity_level_;

        // 加载状态标志
        bool loaded_;

    public:
        // 构造函数
        InitialPDConfig();
        InitialPDConfig(const std::string& yaml_path,
                        const std::string& rigidity_level);

        // 从 YAML 文件加载指定刚度等级的 PD 配置
        // rigidity_level: "low" / "mid" / "high" / "superhigh" / "tag"
        // 返回 true 表示加载成功
        bool loadFromYaml(const std::string& yaml_path,
                          const std::string& rigidity_level);

        // 获取 PD 参数
        std::vector<double> getLeftArmKp()  const;
        std::vector<double> getLeftArmKd()  const;
        std::vector<double> getRightArmKp() const;
        std::vector<double> getRightArmKd() const;

        // 获取当前刚度等级
        std::string getRigidityLevel() const;

        // 打印当前 PD 配置到控制台
        void printPDConfig() const;

        // 检查是否已成功加载
        bool isLoaded() const;
};

// --------------- 封装读取函数 --------------- 

//注意关节空间与笛卡尔空间零位的匹配

// ----- 关节层零位配置 -----
//将零位角度配置读取到全局变量中
bool loadZeroPosFromYaml(const std::string& yaml_path);

// ----- 笛卡尔零位配置 -----
//将零位笛卡尔空间配置读取到全局变量中
bool loadDescZeroPosFromYaml(const std::string& yaml_path);

// ----- 默认PD参数配置 -----
//加载 PD 配置文件 -- 从 initialPD.yaml 读取 PD 参数并填入全局变量
bool loadInitialPDFromYaml(const std::string& yaml_path);

#endif
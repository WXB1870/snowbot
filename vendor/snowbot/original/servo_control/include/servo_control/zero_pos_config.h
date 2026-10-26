#ifndef __zero_pos_config__H__
#define __zero_pos_config__H__

#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include "ros_controller/rosLoopMsgDefine.h"

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
extern std::string yaml_path_zeroPos;
extern std::string yaml_path_shutdownPos;
extern std::string yaml_path_initialPD;

//默认位置定义
extern std::vector<double> lArm_zeroPos;
extern std::vector<double> rArm_zeroPos;
extern double lGripper_zeroPos;
extern double rGripper_zeroPos;

//关机位置定义 (shutdownpos.yaml, Ctrl+C 回零目标)
extern std::vector<double> lArm_shutdownPos;
extern std::vector<double> rArm_shutdownPos;
extern double lGripper_shutdownPos;
extern double rGripper_shutdownPos;

//默认 PD 参数定义
extern std::vector<double> lArm_defaultKp;
extern std::vector<double> lArm_defaultKd;
extern std::vector<double> rArm_defaultKp;
extern std::vector<double> rArm_defaultKd;

class ZeroPosConfig{
    private:
        // 零位位置数据
        std::vector<double> lArm_zeroPos;               // 左臂零位 (7 关节)
        std::vector<double> rArm_zeroPos;               // 右臂零位 (7 关节)
        double lGripper_zeroPos;                        // 左夹爪零位
        double rGripper_zeroPos;                        // 右夹爪零位

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
        double getLeftGripperZeroPos()  const { return lGripper_zeroPos; }
        double getRightGripperZeroPos() const { return rGripper_zeroPos; }

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

//加载零位配置文件 -- 从 zeropos.yaml 读取零位位置并填入全局变量
bool loadZeroPosFromYaml(const std::string& yaml_path);

//加载关机配置文件 -- 从 shutdownpos.yaml 读取关机位置并填入全局变量
bool loadShutdownPosFromYaml(const std::string& yaml_path);

//加载 PD 配置文件 -- 从 initialPD.yaml 读取 PD 参数并填入全局变量
bool loadInitialPDFromYaml(const std::string& yaml_path,
                           const std::string& rigidity_level);




#endif
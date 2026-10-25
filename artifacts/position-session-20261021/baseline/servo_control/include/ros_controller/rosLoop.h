#ifndef __rosLoop__H__
#define __rosLoop__H__

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "servo_control/zero_pos_config.h"
#include "servo_control/transmit.h"

#include <memory>
#include <chrono>
#include <functional>

#include "ros_controller/rosLoopMsgDefine.h"

#include "JntController/runinController.h"
#include "JntController/JntTestCtl.h"
#include "JntController/ForceComp.h"
#include "JntController/retZeroControl.h"

#include "snowbot_ctl_msgs/msg/ctl_information.hpp"
#include "snowbot_ctl_msgs/msg/ret_information.hpp"
#include "snowbot_ctl_msgs/msg/jnt_comp_res_visu.hpp"
#include "snowbot_ctl_msgs/msg/dbg_information.hpp"

#include "impedance/cartesian_impedance_controller.h"
#include "impedance/dual_arm_coordinator.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "kinematic_pose_msgs/msg/kinematic_pose.hpp"

extern "C" {
#include "ethercat.h"
}

/*
        by 霜云喵
================================
通讯主函数代码（下位机）
============修改日志============

==============协议==============
本代码为下位机节点主函数代码
控制流程为：
    1. 先获取机械臂关节返回值，并发送
    2. 然后获取上位机发送的指令
    3. 将上位机发送的指令传输给关节

通讯成功后，当第一轮时，发送的返回值均为0,并发送初始轮标志
若通讯失败，未能成功获取上位机指令，则进入零刚度高阻尼模式，回零位

注意：Kp, Kd不参与插补；除了位置量以外其他量在开始插补时默认为0

==============通讯协议1==============

中层节点与下位机的通讯。下位机：关节通讯节点

传输信息：

1. 控制信息：
    temp_time -- 本中层节点的循环周期，用于与下层节点协调周期，并用于插补
    runin_mod -- 下位机（关节通讯节点）插补模式
        0 -- 不插补
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

三种传输模式的传输协议：
1. 力位混合模式
  关键的传输量有三个：
  期望位置、期望速度、期望力前馈。力位混合控制有不同的刚度模式，其在下层对应不同的Kp、Kd配置。

2. 位置控制模式
  关键的传输量有两个：
  期望位置、期望速度。其他的控制量置0占位或合理填充即可。

3. 电流控制模式
  关键的传输量只有1个：
  期望电流。期望电流通过力前馈传输通道赋值。其他量置0或者合理填充即可


================================

排列顺序协议：传输过程中，
    前7个关节 -- 对应左臂
    第8到第14个关节 -- 对应右臂

================================
*/

class ServoNode : public rclcpp::Node {
    public:
        //构造函数（初始化用）
        ServoNode();

        //析构函数
        ~ServoNode(){
            if (performance_test) {
                delete performance_test;
            }
        }

        //ros循环通讯变量初始化函数
        void rosLoopVar_Init();

        //ros消息参量发送模块 -- 反馈值发送函数
        void msg_publish();

        //ros消息回调函数 -- 上位机控制消息接收模块
        void ctlmsg_get();

        //消息传输辅助函数 -- 仅仅进行一次消息的接收(与底层关节的通讯)
        void EtherCATmsg_Get();

        //消息传输辅助函数 -- 进行一次消息的收发(与底层关节的通讯)
        void EtherCATrun();

        //主要循环函数
        void run();

    private:

        //====================消息发送变量====================

        //----------反馈信息发送----------
        rclcpp::Time global_timestamp_;

        //上位机通讯消息发送
        rclcpp::Publisher<snowbot_ctl_msgs::msg::RetInformation>::SharedPtr RetData_pub;    //返回值发送
        rclcpp::Subscription<snowbot_ctl_msgs::msg::CtlInformation>::SharedPtr CtlData;     //控制量获取

        //六维力传感器发布 (从 EtherCAT 读取)
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_pub;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_pub;
        void publish_sensor_data();

        //debug消息发送接口
        rclcpp::Publisher<snowbot_ctl_msgs::msg::DbgInformation>::SharedPtr debugDataPub_comparison;    //比对用debug消息发送

        //夹爪控制订阅 (Float64MultiArray: [0]=目标角度deg, [1]=电流阈值A)
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_gripper_sub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_gripper_sub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_gripper_direct_sub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_gripper_direct_sub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torso_cmd_sub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cmd_sub;
        void gripper_cmd_apply(const std_msgs::msg::Float64MultiArray::SharedPtr msg, bool is_left);
        void gripper_runin();
        void torso_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void head_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void torso_runin();
        void head_runin();
        rclcpp::Time last_left_sensor_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        rclcpp::Time last_right_sensor_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        //添加标志位，标记是否接收到指令
        bool has_received_command_;     //当该标志位为false -- 非使能状态； 当标志位为true -- 使能状态
        bool ifFirstTemp;

        //----------控制调试信息发送----------
        performanceTestSeal* performance_test;

        //==================================================
        //消息传递辅助变量 -- CAN板数量，以及各个CAN板上关节数量
        JntTopology servo_jnt_topology;

        //消息传递变量
        std::vector<std::vector<joint_desMsg>> servo_desMsg;
        std::vector<std::vector<joint_retMsg>> servo_retMsg;

        int vector_len = 0;
        //解包前目标变量
        std::vector<joint_desMsg> servo_desMsg_orig;
        //解包后反馈变量
        std::vector<joint_retMsg> servo_retMsg_unpacked;
        //电机力控参数
        std::vector<double> jnt_kt_unpacked;

        //----------插补器变量----------
        //左右臂插补位置初始化刷新量
        std::vector<double> l_posReceive_atRuninSt;
        std::vector<double> r_posReceive_atRuninSt;

        //----------模式控制变量----------
        double tempTime_upperMonitor;       //上位机迭代时间
        int runin_mod;                      //插补模式
        bool if_topic_point_position = false; //Topic 点位模式
        int if_runin_renew;                 //是否刷新插补
        int msg_get_ifUseDebugMod;          //是否使用调试模式
        int if_useret_to_start_runin;       //判断量 -- 是否使用返回值作为第一轮插补的输入值

        //----------过程控制变量----------
        std::vector<double> servo_desVel_last;       //上一轮的目标速度

        //控制消息获取辅助变量
        std::vector<int> lArm_transMod;

        std::vector<double> lArm_pos_receive;
        std::vector<double> lArm_vel_receive;

        std::vector<double> lArm_torComp_grav_receive;
        std::vector<double> lArm_torComp_interia_receive;
        std::vector<double> lArm_torComp_feedforward_receive;

        std::vector<double> lArm_Kp_receive;
        std::vector<double> lArm_Kd_receive;

        std::vector<int> rArm_transMod;

        std::vector<double> rArm_pos_receive;
        std::vector<double> rArm_vel_receive;

        std::vector<double> rArm_torComp_grav_receive;
        std::vector<double> rArm_torComp_interia_receive;
        std::vector<double> rArm_torComp_feedforward_receive;

        std::vector<double> rArm_Kp_receive;
        std::vector<double> rArm_Kd_receive;

        //插补过程辅助变量
        bool ifFirstRunin = true;

        //回零控制变量
        JointPos_Init retZero_Init;

        //------------------------------

        /*  目标值包装协议
        --------------------
        包装协议 -- 把目标数据直接包装，直接按照原始顺序依次对应
        */
        void desMsg_pack_func();

        /*  反馈值配套解包函数
        -----------------------
        解包协议 -- 直接把反馈数据堆叠即可
        */
        void retMsg_unpack_func();     

        //==================================================
        //控制器及其配套过程控制变量

        //-----插补器-----
        runin_control runin_control_core;

        int temp_len = 0;

        //控制量 -- 插补器是否初化过
        bool ifRuninerInited = false;

        //-----计时器-----
        double time_res = 0.0;
        double time_last = 0.0;
        int winlen = 0;     //计时器窗口长度

        // 躯干/头部 runin 变量
        static constexpr int TORSO_MOTORS = 2;
        static constexpr int HEAD_MOTORS  = 2;
        bool t_ifRunin = false; int t_step = 0, t_winlen = 0; double t_time_last = 0.0;
        bool t_inited = false;
        double t_pos_curr[TORSO_MOTORS] = {0};
        double t_pos_start[TORSO_MOTORS] = {0}, t_pos_target[TORSO_MOTORS] = {0};
        double t_spd[TORSO_MOTORS] = {0}, t_tor[TORSO_MOTORS] = {0};
        double t_kp[TORSO_MOTORS]  = {0}, t_kd[TORSO_MOTORS]  = {0};

        bool h_ifRunin = false; int h_step = 0, h_winlen = 0; double h_time_last = 0.0;
        bool h_inited = false;
        double h_pos_curr[HEAD_MOTORS] = {0};
        double h_pos_start[HEAD_MOTORS] = {0}, h_pos_target[HEAD_MOTORS] = {0};
        double h_spd[HEAD_MOTORS] = {0}, h_tor[HEAD_MOTORS] = {0};
        double h_kp[HEAD_MOTORS]  = {0}, h_kd[HEAD_MOTORS]  = {0};

        bool ifShallInit = true;

        //关机回零模式标志 (Ctrl+C 后在主循环内按 shutdownpos.yaml 回零)
        bool shutdown_mode_ = false;

        //-----动力学补偿-----
        
        //刚度补偿
        K_comp pos_k_comp_l;
        K_comp pos_k_comp_r;

        //位置、速度输出滤波器
        runinOutpFtr posOutpFtr;
        runinOutpFtr velOutpFtr;
        runinOutpFtr TorOutpFtr;

        //初始段输出控制代码
        int ftrInitOutpTimes = 4000;
        int ftrInitOutpCount = 0;
        bool ifFirstFtrInitOutp = true;

        //摩擦补偿
        Stribeck_FricCompWrap stribeckComp_wrap;

        //==================================================
        //上位机变量传输函数组

        //函数1 -- 传输信息回调函数
        void message_callback(const snowbot_ctl_msgs::msg::CtlInformation::SharedPtr msg);

        //函数2 -- 执行插补函数
        bool ifRunin = false;   //插补启动标志
        void ctlMsg_runin();

        // 夹爪插补变量
        bool g_left_inited   = false;
        bool g_right_inited  = false;
        bool g_left_ifRunin  = false;
        bool g_right_ifRunin = false;
        double g_left_time_last  = 0.0;
        double g_right_time_last = 0.0;
        int    g_left_step  = 0, g_left_winlen  = 0;
        int    g_right_step = 0, g_right_winlen = 0;
        double g_left_pos_start  = 0.0, g_left_pos_target  = 0.0, g_left_pos_curr = 0.0;
        double g_right_pos_start = 0.0, g_right_pos_target = 0.0, g_right_pos_curr = 0.0;
        double g_left_spd  = 0.0, g_left_tor  = 0.0, g_left_kp  = 0.0, g_left_kd  = 0.0;
        double g_right_spd = 0.0, g_right_tor = 0.0, g_right_kp = 0.0, g_right_kd = 0.0;

        //函数3 -- 数据处理与转换函数
        void outpTraj_process();

        //==================================================
        //debug辅助函数

        //debug函数1 -- 比对容器中直接获取的传输值与实际的传输值
        void comp_desMsg_containerMsg();

        //==================== 阻抗控制接口 ====================
        // 双臂阻抗控制器实例 (左/右各一, 单臂独立做阻抗)
        // 参考位姿订阅由控制器内部直接管理:
        //   ~/impedance/arm_l_link_7/reference  (左臂)
        //   ~/impedance/arm_r_link_7/reference  (右臂)
        std::shared_ptr<snowbot_impedance::CartesianImpedanceController>
            impedance_left_;
        std::shared_ptr<snowbot_impedance::CartesianImpedanceController>
            impedance_right_;

        // 双臂协调层 (阶段 1: 主从控制).
        //   默认 mode=OFF, 两臂独立阻抗控制 (与当前行为完全一致).
        //   显式配置 dual_arm.mode 或运行时切换为 follower 时激活:
        //     leader: 主臂保持独立阻抗
        //     follower: 从臂参考位姿 = 主臂 ee × T_tool_constraint
        snowbot_impedance::DualArmCoordinator dual_arm_coordinator_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dual_arm_mode_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trp_reference_sub_;
        bool impedance_enable_{false};        // 总使能开关
        bool impedance_inited_{false};        // 初始化成功标志
        // 使能上升沿标志: enable 回调置 true, applyImpedanceControl 检测并调用
        // 控制器 enable() 捕获当前末端位姿/sensor bias 后清零.
        std::atomic<bool> impedance_just_enabled_{false};

        // 六维力传感器数据滤波 (一阶低通)
        //   在 applyImpedanceControl 入口处提前读取并滤波,
        //   即使阻抗未使能也持续更新, 使能时无阶跃
        Eigen::Matrix<double, 6, 1> wrench_l_filtered_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> wrench_r_filtered_{Eigen::Matrix<double, 6, 1>::Zero()};
        double sensor_lpf_alpha_{0.15};  // 低通滤波系数 (0~1, 越小越平滑)
        bool sensor_filter_inited_{false};
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr impedance_enable_sub_;

        // 前馈力矩测试: 通过 ROS 话题直接给定关节力矩, 绕过阻抗计算
        //   ~/tau_ff_left  Float64MultiArray[7]: 左臂 J1~J7 前馈力矩 [Nm]
        //   ~/tau_ff_right Float64MultiArray[7]: 右臂 J1~J7 前馈力矩 [Nm]
        //   非零值覆盖阻抗输出, =0 时恢复阻抗控制
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_ff_left_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_ff_right_sub_;
        std::vector<double> tau_ff_left_{};
        std::vector<double> tau_ff_right_{};
        std::mutex tau_ff_mutex_;
        bool tau_ff_left_active_{false};   // 左臂收到非零前馈时置 true
        bool tau_ff_right_active_{false};  // 右臂收到非零前馈时置 true
        void tau_ff_left_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void tau_ff_right_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        // 阻抗控制初始化: 加载 URDF, 配置参数, 创建订阅/发布
        void initImpedanceControllers();
        // 主循环中调用: 计算阻抗力矩并注入到 servo_desMsg_orig
        void applyImpedanceControl();
};

#endif
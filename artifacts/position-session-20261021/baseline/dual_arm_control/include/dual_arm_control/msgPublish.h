#ifndef __msgPublish__H__
#define __msgPublish__H__

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"

#include"controller/ArmInterface.h"

#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_kdl/tf2_kdl.hpp>
#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "snowbot_ctl_msgs/msg/ctl_information.hpp"
#include "snowbot_ctl_msgs/msg/ret_information.hpp"
#include "snowbot_ctl_msgs/msg/dbg_information.hpp"
#include "snowbot_ctl_msgs/msg/mid_node_ctl_msg_send.hpp"
#include "snowbot_ctl_msgs/msg/mid_node_ctl_msg_ret.hpp"
#include "snowbot_ctl_msgs/msg/test_msg.hpp"
#include "snowbot_ctl_msgs/msg/end_desc_pub.hpp"

#include <unistd.h> // for read()
#include <sys/select.h> // for select()

/*
        by 霜云喵
================================
通讯代码，用于实现ros通讯
============修改日志============

***注意修改未接受到反馈值时的处理方法。
修改为：
    若未能接到反馈值，并已经运动到目标轨迹终点，则进入位置保持模式
    若接受到反馈值，且反馈值中包含阻尼回零标志位，则进入阻尼回零模式

==============协议==============
中层节点与上层节点通讯：

1. 机器人关节控制存在以下几种模式：
    归零模式 -- 仅仅Kd不为0，用于机器人安全地在外力拖动下回到零位
    关节模式 -- 左右臂关节空间点位控制
    点位模式 -- 左右臂笛卡尔空间点位控制
    关节轨迹控制模式 -- 执行关节空间目标轨迹（点迹形式传输）
    点位轨迹控制模式 -- 执行笛卡尔空间目标轨迹（点迹形式传输）
    位置保持模式 -- 重力补偿、kp、kd、目标位置输出不动，力前馈输出为上位机发送的目标力，其他的为0
                  作用为保持当前位置
    
    控制逻辑为：
        (1) 中层节点打开但未接受到上层节点输出指令时，处于位置保持模式。
        (2) 中层节点打开，并接受到上层节点指令时候，执行上层节点指令指定的控制模式
        (3) 上层节点在初始化阶段，应当检测中层节点反馈值，若上层节点初始化过程中检测到中层节点关节角反馈值与
            零位差值过大，则应当自动切换到回零位模式。只有当中层节点关节角度反馈值与零位差别足够小时候，上层
            节点才会进入其他的关节控制模式。
        (4) 上层节点在每次第一轮输出时会强制令if_reInit为true


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

关节消息传输变量含义

temp_time -- 本中层节点的循环周期，用于与下层节点协调周期，并用于插补
runin_mod -- 下位机（关节通讯节点）插补模式
    0 -- 不插补
    1 -- 线性插补
    2 -- 二次插补
if_runin_renew -- 是否刷新下位机插补器
    0 -- 否
    1 -- 是
ifUseRet_toStRunin; -- 是否使用返回值作开始插补时第一轮输入
    0 -- 否
    1 -- 是
ifUseDebugMod -- 是否使用调试模式 （未使用）
    0 -- 否
    1 -- 是
base_rigidityCtl_mod_l 、base_rigidityCtl_mod_l -- 底层控制与传输模式
    4 -- 位置模式
    5 -- 电流模式
    其他 -- 力位混合模式

l_position_at_runin_st 、r_position_at_runin_st -- 下发的插补开始时初始值

================================
*/

//============================================================
//全局变量：

//---------------中层节点与下位机通讯全局变量---------------

//控制量发送：
extern int runin_mod;               //插补模式
extern int if_runin_renew;          //判断是否刷新底层插补器
extern int ifUseRet_toStRunin;      //判断是否采用返回值
extern int ifUseDebugMod;           //判断是否使用调试模式

extern int base_rigidityCtl_mod_l[SINGLEARMDOF];    //控制量：底层控制模式
extern int base_rigidityCtl_mod_r[SINGLEARMDOF];    //控制量：底层控制模式

//左右臂初始值刷新
extern std::vector<double> lArm_pos_atRuninSt;
extern std::vector<double> rArm_pos_atRuninSt;

//目标参量发送：
extern std::vector<double> lArm_joint_pos;
extern std::vector<double> lArm_joint_kp;
extern std::vector<double> lArm_joint_kd;
extern std::vector<double> lArm_joint_spd;

extern std::vector<double> lArm_joint_tor_grav;
extern std::vector<double> lArm_joint_tor_interia;
extern std::vector<double> lArm_joint_tor_feedforward;

extern std::vector<double> rArm_joint_pos;
extern std::vector<double> rArm_joint_kp;
extern std::vector<double> rArm_joint_kd;
extern std::vector<double> rArm_joint_spd;

extern std::vector<double> rArm_joint_tor_grav;
extern std::vector<double> rArm_joint_tor_interia;
extern std::vector<double> rArm_joint_tor_feedforward;

//反馈接收：
extern std::vector<double> lArm_pos_fdback;
extern std::vector<double> lArm_spd_fdback;
extern std::vector<double> lArm_iq_fdback;

extern std::vector<double> rArm_pos_fdback;
extern std::vector<double> rArm_spd_fdback;
extern std::vector<double> rArm_iq_fdback;

extern int if_lower_firstTime;      //底层插补是否为第一轮插补
extern int if_middle_firstTime;     //中层控制是否为第一轮控制

//控制状态向上位机反馈：
extern std::vector<bool> lArm_ifReachEnd;   //左臂是否到达插补终点
extern std::vector<bool> rArm_ifReachEnd;   //右臂是否到达插补终点

//---------------中层节点与上位机通讯全局变量---------------

//-----控制信息-----

//综合模式控制信息
extern modCtl modCtlMsg_get;

//程序打印控制参数
extern optiPrintMsgCtl printMsgPkg_get;

//优化器参数打包
extern optimizerCtl optimizerPara_get;

//关节限制信息打包
extern JntLimMsg lArm_LimMsg_get;
extern JntLimMsg rArm_LimMsg_get;
extern JntLimMsg allArm_LimMsg_get;

//控制过程信息打包
extern bool ifReInit_get;              //控制是否重新初始化
extern bool ifUpdateLim_get;           //是否刷新关节限制
extern bool ifRedefineJntTrajStab_get; //是否重新定义静止关节的角度

//左右臂关节初始位置
extern std::vector<double> lArm_stPos_get;  //左臂初始位置
extern std::vector<double> rArm_stPos_get;  //右臂初始位置

//-----目标位置等信息-----

extern std::vector<double> lArm_desPos_descartes;
extern std::vector<double> lArm_desPos_JntSpace;

extern std::vector<double> lArm_desTor_descartes;
extern std::vector<double> lArm_desTor_JntSpace;

extern std::vector<double> rArm_desPos_descartes;
extern std::vector<double> rArm_desPos_JntSpace;

extern std::vector<double> rArm_desTor_descartes;
extern std::vector<double> rArm_desTor_JntSpace;

//搭配自定义PD参数模式使用 -- PD参数接收模块
extern std::vector<double> lArm_Kp_;
extern std::vector<double> lArm_Kd_;
extern std::vector<double> rArm_Kp_;
extern std::vector<double> rArm_Kd_;

/*  辅助目标前馈力矩
---------------------
辅助目标前馈力矩不由上层求解器发送，
辅助目标前馈力矩包含上层求解器目标力矩在关节空间的映射，以及阻抗力矩
辅助目标前馈力矩的作用为用用作位置保持模式中目标力矩的力前馈。
*/
extern std::vector<double> lArm_torFeedForward_assist;
extern std::vector<double> rArm_torFeedForward_assist;

//---------------通讯辅助变量---------------

//debug用的辅助变量
extern std::vector<double> vec_dbg_1;
extern std::vector<double> vec_dbg_2;

// 全局变量用于信号处理
static volatile sig_atomic_t shutdown_flag = 0;

//============================================================

// 辅助函数

// 信号处理函数
void signalHandler(int sig);

bool is_key_pressed(int timeout_ms = 0);

//============================================================
//调试用辅助函数
std::vector<double> JntPDoutpDebug(double posDes, double velDes, double posRet, double Kp, double Kd);
// 末端位置画图的曲线函数
class EndPos_plot{
    private:
        //消息发送变量：
        rclcpp::Node* node_ptr_;
        rclcpp::Publisher<snowbot_ctl_msgs::msg::EndDescPub>::SharedPtr pvTrajPub;

        //成员变量：
        std::vector<double> lArm_DescPos_last;
        std::vector<double> rArm_DescPos_last;

        float temp_time = TEMPTIME;

    public:
        EndPos_plot(rclcpp::Node* node);

        void end_pos_plot(
            std::vector<double> lArm_desDescPos, std::vector<double> rArm_desDescPos,
            std::vector<double> lArm_retDescPos, std::vector<double> rArm_retDescPos
        );
};


//============================================================
//通讯核心类：

class ArmCtlNode : public rclcpp::Node{
    public:
        /*  构造函数
        ---------------
            在构造函数中执行控制的初始化
        */
        ArmCtlNode();
        ~ArmCtlNode(){
            if(endPosTrajPlot){
                delete endPosTrajPlot;
            }
        }

        //------------------------------
        //ros传输变量初始化模块
        void rosloop_var_init();

        //------------------------------
        //控制参量发送模块
        void msg_publish();

        //上位机指令发送模块
        void midMonitor_msg_publish();

        //反馈值接受模块 -- 调用该函数时同时触发两个回调函数
        void fdback_get();

        //------------------------------
        //仿真信息与debug信息发送模块

        // 发送关节角度到RViz
        // 发布身体/头部锁关节TF
        void publish_body_head_tf();
        // 使用KDL计算正向运动学并发布TF
        void publish_tf_from_joint_states(KDL::Chain mkdl_chain_, std::vector<double> mjoint_pos);

        // 发布关节状态
        void publish_joint_states(
            KDL::Chain mkdl_chain_, std::vector<double> mjoint_pos, 
            std::vector<double> mjoint_spd, std::vector<double> mjoint_tor
        );

        //debug信息发送模块
        void dbg_publish(std::vector<double> mInp_1, int len);
        void dbg_publish_2(std::vector<double> mInp_1, std::vector<double> mInp_2);

        //------------------------------
        //ArmInterface接口交互模块

        //初始化模块
        void InterfaceInit();

        //进行一轮控制计算的模块
        void controlCal();

        void retZeroControlInit();  // 回零控制初始化计算代码
        void control_seal();        // 控制轮封装代码

        //进行一轮输出
        void controlOutp();

        //------------------------------
        //ros2 循环函数
        void run();

    private:
        //----------通讯变量----------

        std::string ifname_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_joint_service_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

        //rviz2通讯变量
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher_;
        std::shared_ptr<urdf::Model> robot_model_;
        std::string robot_description_;

        // 缓存 body/head/gripper joint_states (由各控制节点发布)
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_cache_sub_;
        void js_cache_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        std::mutex js_cache_mutex_;
        double cached_body_j1_ = 0.0, cached_body_j2_ = 0.0;
        double cached_head_j1_ = 0.0, cached_head_j2_ = 0.0;
        double cached_grip_l_ = 0.0, cached_grip_r_ = 0.0;

        // KDL相关变量
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_lArm_chain_;
        KDL::Chain kdl_rArm_chain_;

        std::thread tf_publish_thread_;
        sensor_msgs::msg::JointState::SharedPtr joint_states_;
        std::mutex joint_states_mutex_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

        //全局时间戳
        rclcpp::Time global_timestamp_;

        //消息发布与接收变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::CtlInformation>::SharedPtr mCtlData_pub;
        rclcpp::Subscription<snowbot_ctl_msgs::msg::RetInformation>::SharedPtr mRetData;

        rclcpp::Subscription<snowbot_ctl_msgs::msg::MidNodeCtlMsgSend>::SharedPtr upperMonitorMsg_get;
        rclcpp::Publisher<snowbot_ctl_msgs::msg::MidNodeCtlMsgRet>::SharedPtr upperMonitorMsg_send;

        rclcpp::Publisher<snowbot_ctl_msgs::msg::DbgInformation>::SharedPtr mDbgData_pub;

        //添加标志位
        bool has_received_command_;
        bool has_received_upperMonitor_command_;

        //第一轮的检查
        bool checkTemp = true;

        //迭代过程标志量
        bool ifFirstTemp = true;        //判断是否为第一轮控制
        bool ifIKsolverInited = false;
        bool ifInitTemp = true;

        //电机使能空闲抱闸 (默认关闭, 收到上位机命令才使能)
        int motor_enable_ = 0;              // 当前使能状态
        int motor_idle_count_ = 0;          // 空闲计数
        static constexpr int MOTOR_IDLE_TIMEOUT = 50;  // 50周期@50Hz = 1秒

        //------------------------------
        //求解器操作界面
        ArmCtlOptInterface solverInterface;

        //末端位值画图的曲线函数
        EndPos_plot* endPosTrajPlot;

        //------------------------------
        //回调函数

        //关节回调函数
        void set_joint_angles(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response); 

        //消息回调函数
        void message_callback(const snowbot_ctl_msgs::msg::RetInformation::SharedPtr msg);

        //上位机发送消息回调函数
        void upperMonitor_msg_callback(const snowbot_ctl_msgs::msg::MidNodeCtlMsgSend::SharedPtr msg);
};

//============================================================

#endif
#ifndef __msgPublisher__H__
#define __msgPublisher__H__

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_kdl/tf2_kdl.hpp>
#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <termios.h>      // 终端控制
#include <unistd.h>       // read(), STDIN_FILENO
#include <sys/select.h>   // select()
#include <iomanip>        // std::setprecision

#include "std_srvs/srv/set_bool.hpp"
#include "snowbot_ctl_msgs/msg/ctl_information.hpp"
#include "snowbot_ctl_msgs/msg/ret_information.hpp"
#include "snowbot_ctl_msgs/msg/dbg_information.hpp"
#include "snowbot_ctl_msgs/msg/mid_node_ctl_msg_send.hpp"
#include "snowbot_ctl_msgs/msg/mid_node_ctl_msg_ret.hpp"
#include "snowbot_ctl_msgs/msg/joint_position_command.hpp"

#include "controller/upper_Transmit.h"
#include "messageType/chassis_msgType.h"
#include "messageType/upper_msgType.h"

#include <unistd.h> // for read()
#include <sys/select.h> // for select()

//#include "robot_control/msgType.h"
#include "controller/demoTraj.h"
#include "controller/xrTelepercationCom.h"

#include <chrono>
#include <thread>

/*
        by 霜云喵
================================
通讯代码，用于实现ros通讯
============修改日志============

==============协议==============
上层节点发送频率不得高于中层节点发送频率
上层节点发送频率必须是中层节点的整数倍

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

================================
*/

//---------------中层节点反馈值---------------

//关节信息反馈值
extern std::vector<double> lArm_pos_fdback;
extern std::vector<double> lArm_spd_fdback;
extern std::vector<double> lArm_iq_fdback;

extern std::vector<double> rArm_pos_fdback;
extern std::vector<double> rArm_spd_fdback;
extern std::vector<double> rArm_iq_fdback;

//判断量 -- 中层循环是否到达迭代终点
extern std::vector<bool> lArm_ifReach_end; 
extern std::vector<bool> rArm_ifReach_end;

//中层节点本轮迭代的目标位置、速度以及三类力
extern std::vector<double> lArm_temp_desPos;
extern std::vector<double> lArm_temp_desVel;

extern std::vector<double> lArm_temp_desGravComp;       //重力补偿力
extern std::vector<double> lArm_temp_desInteriaComp;    //惯性力与科氏力的综合前馈
extern std::vector<double> lArm_temp_desFeedForward;    //前馈力

extern std::vector<double> rArm_temp_desPos;
extern std::vector<double> rArm_temp_desVel;

extern std::vector<double> rArm_temp_desGravComp;
extern std::vector<double> rArm_temp_desInteriaComp;
extern std::vector<double> rArm_temp_desFeedForward;

//---------------上层节点下发值---------------

// 全局变量用于信号处理
static volatile sig_atomic_t shutdown_flag = 0;
//============================================================
// 辅助函数
// 信号处理函数
void signalHandler(int sig);


//============================================================
//通讯核心类：

class RobotCtlNode : public rclcpp::Node{
    public:

        /*  构造函数
        ---------------
            在构造函数中执行初始化
        */
        RobotCtlNode();
        void OutpInit();        //初始化输出量的函数，默认处于强制归零模式

        //---------------
        //控制参量发送函数
        void msg_publish();

        //反馈值接受模块
        void fdback_get();

        //--------------
        //debug信息发送模块
        void dbg_publish(std::vector<double> mInp_1, int len);
        void dbg_publish_2(std::vector<double> mInp_1, std::vector<double> mInp_2);

        //--------------
        /*  上层求解器代码再此处插入    
        ----------------------------
        上层求解器轨迹调用与处理代码位置：trajProcessWrap.cpp
        */
        

        //轨迹转换代码
        void TrajOutp(int DescTrajMod = 1);
        

        //--------------
        //ros2循环函数
        bool ifFirstRun = true;
        bool ifFirstOutput = true;
        void run();

    private:
        enum class ControlSource {
            Vr,
            Demo,
            Topic
        };

        ControlSource control_source_;

        //----------通讯变量----------

        //全局时间戳
        rclcpp::Time global_timestamp_;

        //上层节点消息收发变量
        rclcpp::Publisher<snowbot_ctl_msgs::msg::MidNodeCtlMsgSend>::SharedPtr upperMointor_Msg_send;
        rclcpp::Subscription<snowbot_ctl_msgs::msg::MidNodeCtlMsgRet>::SharedPtr upperMonitor_Msg_get;
        rclcpp::Publisher<snowbot_ctl_msgs::msg::DbgInformation>::SharedPtr mDbgInformation;
        rclcpp::Subscription<snowbot_ctl_msgs::msg::JointPositionCommand>::SharedPtr joint_position_cmd_sub_;

        bool has_received_ret_;

        //----------通讯信息处理变量----------
        //上层节点信息传输
        armTrajMsg_Trans armTrajMsg;        //机械臂轨迹控制信息传输包裹
        armTrajMsg_Trans jointPositionMsg;   //独立 Topic 关节位置控制信息
        base_control_msg baseCtlMsg;         //底盘控制信息传输包裹

        arm_traj_ctl armTrajTransCtl;       //机械臂轨迹控制类

        dataSendWrap upperMonitor_dataSend; //信息传输包装
        //---------------------------------


        //过程控制变量：
        bool ifInitTemp = true;
        bool ifForceToRetZero = false;
        bool joint_position_cmd_pending_ = false;
        int old_transMod;

        //通讯信息回调函数
        void message_callback(const snowbot_ctl_msgs::msg::MidNodeCtlMsgRet::SharedPtr msg);
        void joint_position_cmd_callback(
            const snowbot_ctl_msgs::msg::JointPositionCommand::SharedPtr msg);
        void jointPositionInit();
        void jointPositionOutp();

        //----------交互演示轨迹发送----------
        allDemoTraj demoTrajWrap;
        
        //控制参量
        int trajChoose;             //选择的轨迹

        void demoTrajInit();        //演示轨迹初始化模块
        void demoTrajOutp();        //演示轨迹输出模块
        //---------------------------------

        //遥操作通讯包装变量
        xr_telepercation_ctl xrTeleComuCtl;

        //遥操作节点信息接受变量
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lArm_endPos;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rArm_endPos;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lArm_end_delta;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rArm_end_delta;

        //遥操作通讯初始化
        void xr_teleComu_init();
        void xr_teleComu_outp();

        //遥操作消息回调函数
        void lArm_xrTele_endPos_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void rArm_xrTele_endPos_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void lArm_xrTele_endDelta_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void rArm_xrTele_endDelta_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
};



//============================================================


#endif
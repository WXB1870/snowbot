#include "robot_control/msgPublisher.h"

/*
        by 霜云喵
================================
上位机输出轨迹处理代码
============修改日志============

==============协议==============

================================
*/


//轨迹输出
/*  
注意这个轨迹输出函数并不会检测是否下位机所有关节到达目标位置，只要调用一次这个函数，就进行依次计算并输出一次轨迹
是否应当输出一次轨迹，既下位机所有关节是否到达目标位置应当由外部函数检测
*/
void RobotCtlNode::TrajOutp(int DescTrajMod){
    upperMonitor_dataSend = armTrajTransCtl.trans_and_outp(armTrajMsg.gravCompMod, armTrajMsg.JntStabMod_l, armTrajMsg.JntStabMod_r, armTrajMsg.ctlTransMod, armTrajMsg.ctlDesMsg, armTrajMsg.ifReachEnd, armTrajMsg.ifUseTorFeedForward, DescTrajMod);
}

//---------------演示轨迹函数---------------

void RobotCtlNode::demoTrajInit(){
    std::cout << "请选择执行的轨迹" << std::endl;
    std::cout << "  1 -- 点位控制测试轨迹；" << std::endl
              << "  2 -- 精度测试轨迹1；" << std::endl
              << "  3 -- 速度测试轨迹1；" << std::endl
              << "  4 -- 直线演示轨迹；" << std::endl
              << "  5 -- 零空间控制演示轨迹" << std::endl
              << "  6 -- 三点控制演示轨迹" << std::endl
              << "  7 -- 零力拖动演示轨迹" << std::endl
              ;
              /**/
    int m_Choose;
    std::cin >> m_Choose;
    this->trajChoose = m_Choose;

    std::cout << "请选择底层刚度与控制模式（**注意，请保证选择模式的匹配**)" << std::endl;
    std::cout << "  1 -- 低刚度模式；" << std::endl
            << "  2 -- 中等刚度模式；" << std::endl
            << "  3 -- 高刚度模式；" << std::endl
            << "  7 -- 超高刚度模式" << std::endl;
            /**/
    int control_mod;
    std::cin >> control_mod; 
    if(control_mod > 0 && control_mod < 10){
        demoTrajWrap.m_rigidityCtlMod = control_mod;
    }else{
        std::cout << "输入有误，默认进入保护待机模式" << std::endl;
        demoTrajWrap.m_rigidityCtlMod = 0;
    }

    std::cout << "执行轨迹：" << trajChoose << std::endl;

    if(trajChoose == 1){
        demoTrajWrap.init_v1();
    }else if(trajChoose == 2){
        demoTrajWrap.init_v2();
    }else if(trajChoose == 3){
        demoTrajWrap.init_v3();
    }else if(trajChoose == 4){
        demoTrajWrap.init_v4();
    }else if(trajChoose == 5){
        demoTrajWrap.init_v5();
        demoTrajWrap.initCtl_v5 = true;
    }else if(trajChoose == 6){
        demoTrajWrap.init_v6();
    }else if(trajChoose == 7){
        demoTrajWrap.init_v7();
    }
    else{
        std::cout << "输入有误，默认执行轨迹1" << std::endl;
        demoTrajWrap.init_v1();
    }

}

void RobotCtlNode::demoTrajOutp(){
    DotOutpWrap temp_dotOutp;

    if(trajChoose == 1){
        temp_dotOutp = demoTrajWrap.TrajOutp_v1();
    }else if(trajChoose == 2){
        temp_dotOutp = demoTrajWrap.TrajOutp_v2();
    }else if(trajChoose == 3){
        temp_dotOutp = demoTrajWrap.TrajOutp_v3();
    }else if(trajChoose == 4){
        temp_dotOutp = demoTrajWrap.TrajOutp_v4();
    }else if(trajChoose == 5){
        temp_dotOutp = demoTrajWrap.TrajOutp_v5();
    }else if(trajChoose == 6){
        temp_dotOutp = demoTrajWrap.TrajOutp_v6();
    }else if(trajChoose == 7){
        temp_dotOutp = demoTrajWrap.TrajOutp_v7();
    }
    else{
        temp_dotOutp = demoTrajWrap.TrajOutp_v1();
    }
    //数据传递函数赋值
    this->armTrajMsg = temp_dotOutp.armTrajMsg;
    this->baseCtlMsg = temp_dotOutp.baseCtlMsg;

    //调用限位刷新函数
    armTrajTransCtl.JntStabMod(
        temp_dotOutp.l_ifJntStab, temp_dotOutp.r_ifJntStab
    );

}

//----------------------------------------


//-----------------------------------------------------------
//遥操作处理代码

//遥操作信息收发初始化
void RobotCtlNode::xr_teleComu_init(){
    lArm_endPos = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/left_hand_euler_delta",
        10,
        std::bind(&RobotCtlNode::lArm_xrTele_endPos_callback, this, std::placeholders::_1)
    );

    rArm_endPos = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/right_hand_euler_delta",
        10,
        std::bind(&RobotCtlNode::rArm_xrTele_endPos_callback, this, std::placeholders::_1)
    );

    lArm_end_delta = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/left_hand_delta",
        10,
        std::bind(&RobotCtlNode::lArm_xrTele_endDelta_callback, this, std::placeholders::_1)
    );
    rArm_end_delta = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/vr/right_hand_delta",
        10,
        std::bind(&RobotCtlNode::rArm_xrTele_endDelta_callback, this, std::placeholders::_1)
    );

    xrTeleComuCtl.init();
}

//遥操作数据转化并输出
void RobotCtlNode::xr_teleComu_outp(){
    upperMonitor_dataSend = xrTeleComuCtl.trans_and_outp();
}

//四个遥操作消息回调函数
void RobotCtlNode::lArm_xrTele_endPos_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){

    xrTeleComuCtl.l_arm_end_pos[0] += msg->x;
    xrTeleComuCtl.l_arm_end_pos[1] += msg->y;
    xrTeleComuCtl.l_arm_end_pos[2] += msg->z;
 
    // std::cout << " xr message get success 1" << std::endl;
}
void RobotCtlNode::rArm_xrTele_endPos_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){

    xrTeleComuCtl.r_arm_end_pos[0] += msg->x;
    xrTeleComuCtl.r_arm_end_pos[1] += msg->y;
    xrTeleComuCtl.r_arm_end_pos[2] += msg->z;

    // std::cout << " xr message get success 2" << std::endl;
}
void RobotCtlNode::lArm_xrTele_endDelta_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){

    xrTeleComuCtl.l_arm_end_delta[0] += 1.5*msg->x;
    xrTeleComuCtl.l_arm_end_delta[1] += 1.5*msg->y;
    xrTeleComuCtl.l_arm_end_delta[2] += 1.5*msg->z;
    
    // std::cout << " xr message get success 3" << std::endl;
}
void RobotCtlNode::rArm_xrTele_endDelta_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){


    xrTeleComuCtl.r_arm_end_delta[0] += 1.5*msg->x;
    xrTeleComuCtl.r_arm_end_delta[1] += 1.5*msg->y;
    xrTeleComuCtl.r_arm_end_delta[2] += 1.5*msg->z;
    
    // std::cout << " xr message get success 4" << std::endl;
}

//-----------------------------------------------------------

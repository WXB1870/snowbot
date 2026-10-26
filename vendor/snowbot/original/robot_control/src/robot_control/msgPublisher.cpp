#include "robot_control/msgPublisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <limits>

//===================================================================================
//全局变量初始化

//关节信息反馈值
std::vector<double> lArm_pos_fdback(SingleArmDof, 0.0);
std::vector<double> lArm_spd_fdback(SingleArmDof, 0.0);
std::vector<double> lArm_iq_fdback(SingleArmDof, 0.0);

std::vector<double> rArm_pos_fdback(SingleArmDof, 0.0);
std::vector<double> rArm_spd_fdback(SingleArmDof, 0.0);
std::vector<double> rArm_iq_fdback(SingleArmDof, 0.0);

//判断量 -- 中层循环是否到达迭代终点
std::vector<bool> lArm_ifReach_end(SingleArmDof, true);
std::vector<bool> rArm_ifReach_end(SingleArmDof, true);

//中层节点本轮迭代的目标位置、速度以及三类力
std::vector<double> lArm_temp_desPos(SingleArmDof, 0.0);
std::vector<double> lArm_temp_desVel(SingleArmDof, 0.0);

std::vector<double> lArm_temp_desGravComp(SingleArmDof, 0.0);       //重力补偿力
std::vector<double> lArm_temp_desInteriaComp(SingleArmDof, 0.0);    //惯性力与科氏力的综合前馈
std::vector<double> lArm_temp_desFeedForward(SingleArmDof, 0.0);    //前馈力

std::vector<double> rArm_temp_desPos(SingleArmDof, 0.0);
std::vector<double> rArm_temp_desVel(SingleArmDof, 0.0);

std::vector<double> rArm_temp_desGravComp(SingleArmDof, 0.0);
std::vector<double> rArm_temp_desInteriaComp(SingleArmDof, 0.0);
std::vector<double> rArm_temp_desFeedForward(SingleArmDof, 0.0);

//---------------上层节点下发值---------------

// 信号处理函数
void signalHandler(int sig) {
    shutdown_flag = 1;
}

//===================================================================================

//构造函数
RobotCtlNode::RobotCtlNode() : Node("robot_upperMonitor") ,
    control_source_(ControlSource::Demo),
    has_received_ret_(false)
{
    const auto control_source = this->declare_parameter<std::string>("control_source", "demo");
    if (control_source == "vr") {
        control_source_ = ControlSource::Vr;
    } else if (control_source == "demo") {
        control_source_ = ControlSource::Demo;
    } else if (control_source == "topic") {
        control_source_ = ControlSource::Topic;
    } else {
        throw std::invalid_argument(
            "control_source must be \"vr\", \"demo\", or \"topic\" (got: " + control_source + ")");
    }

    RCLCPP_INFO(this->get_logger(), "control_source=%s", control_source.c_str());

    //初始化发布与接受的消息类型
    upperMointor_Msg_send = this->create_publisher<snowbot_ctl_msgs::msg::MidNodeCtlMsgSend>(
        "upperMonitor_msg_send",
        10
    );
    upperMonitor_Msg_get = this->create_subscription<snowbot_ctl_msgs::msg::MidNodeCtlMsgRet>(
        "midMonitor_msg_send",
        10,
        std::bind(&RobotCtlNode::message_callback, this, std::placeholders::_1)
    );

    mDbgInformation = this->create_publisher<snowbot_ctl_msgs::msg::DbgInformation>(
        "upperMonitor_dbg_node",
        10
    );

    joint_position_cmd_sub_ = this->create_subscription<snowbot_ctl_msgs::msg::JointPositionCommand>(
        "/joint_position_cmd",
        10,
        std::bind(&RobotCtlNode::joint_position_cmd_callback, this, std::placeholders::_1)
    );

    //从.yaml读取零位及PD配置
    yaml_path_zeroPos = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/zeropos.yaml";
    yaml_path_desc_zeroPos = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/desc_zeropos.yaml";
    yaml_path_initialPD = ament_index_cpp::get_package_share_directory("snowbot_ctl_msgs") + "/config/initialPD.yaml";
    std::cout << " ===== 初始化关节层零位配置 ===== " << std::endl;
    loadZeroPosFromYaml(yaml_path_zeroPos);
    std::cout << " ===== 初始化笛卡尔空间零位配置 ===== " << std::endl;
    loadDescZeroPosFromYaml(yaml_path_desc_zeroPos);
    std::cout << " ===== 初始化PD参数配置 ===== " << std::endl;
    loadInitialPDFromYaml(yaml_path_initialPD);
}

//初始化输出量的函数，默认处于强制归零模式
void RobotCtlNode::OutpInit(){
    //连续轨迹计算函数

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;
    ctlDesMsgWrap ctlStMsg;

    ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    ctlStMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    //关节目标位置
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos, rArm_desJntPos;
    lArm_desJntPos = lArm_zeroPos;
    rArm_desJntPos = rArm_zeroPos;
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos, rArm_desDescPos;
    lArm_desDescPos = l_descZeroPos;
    rArm_desDescPos = r_descZeroPos;

    ctlDesMsgWrap ctlDesMsg;
    ctlDesMsg.desPos_l_Descartes = lArm_desDescPos;
    ctlDesMsg.desPos_r_Descartes = rArm_desDescPos;
    ctlDesMsg.desPos_l_JointSpace = lArm_desJntPos;
    ctlDesMsg.desPos_r_JointSpace = rArm_desJntPos;

    ctlDesMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    ctlDesMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //控制值初始化
    //-----数据传输与插补模式-----
    upperMonitor_dataSend.ctl_transMod = 0;
    upperMonitor_dataSend.runinMod = 3;

    //-----求解模式信息-----
    upperMonitor_dataSend.ifUseRet = 0;
    upperMonitor_dataSend.ifUseTemp = 0;

    //-----自定义PD传输参数-----
    upperMonitor_dataSend.KpTrans_l.resize(SingleArmDof, 0.0);
    upperMonitor_dataSend.KdTrans_l.resize(SingleArmDof, 0.0);
    upperMonitor_dataSend.KpTrans_r.resize(SingleArmDof, 0.0);
    upperMonitor_dataSend.KdTrans_r.resize(SingleArmDof, 0.0);

    //-----重力补偿控制-----
    upperMonitor_dataSend.GravCompCtlMsg.ifUseGravComp = 1;
    upperMonitor_dataSend.GravCompCtlMsg.mIfUseFricComp = 0;
    upperMonitor_dataSend.GravCompCtlMsg.mIfUseInpFtr = 1;
    upperMonitor_dataSend.GravCompCtlMsg.mIfUseOutpFtr = 0;
    upperMonitor_dataSend.GravCompCtlMsg.GravCompInpMod = 1;

    upperMonitor_dataSend.stJntID_lArm = 0;
    upperMonitor_dataSend.stJntID_rArm = 7;
    upperMonitor_dataSend.urdf_choose = 2;

    //-----插补信息包装-----
    double trajTime = 3.0;
    double accRate = 1.2;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    upperMonitor_dataSend.TrajTime.resize(2);
    upperMonitor_dataSend.TrajTime[0].assign(SingleArmDof, trajTime);
    upperMonitor_dataSend.TrajTime[1].assign(SingleArmDof, trajTime);

    upperMonitor_dataSend.accRateBase.resize(2);
    upperMonitor_dataSend.accRateBase[0].assign(SingleArmDof, accRate);
    upperMonitor_dataSend.accRateBase[1].assign(SingleArmDof, accRate);

    upperMonitor_dataSend.m_stabMsg.resize(2);
    upperMonitor_dataSend.m_stabMsg[0].assign(SingleArmDof, stabMsg);
    upperMonitor_dataSend.m_stabMsg[1].assign(SingleArmDof, stabMsg);

    //-----开关量包装-----
    upperMonitor_dataSend.ifJntStab.resize(2);
    upperMonitor_dataSend.ifJntStab[0] = lArm_ifJntStab;
    upperMonitor_dataSend.ifJntStab[1] = rArm_ifJntStab;

    upperMonitor_dataSend.ifRedefineJntTrajStab = false;

    //-----关节限位信息-----
    // 锁定身体/头部/夹爪关节 (仅解算手臂J1~J7)
    upperMonitor_dataSend.lArm_JntLim.ifUseJntLim = true;
    upperMonitor_dataSend.lArm_JntLim.limJntNum = 5;
    upperMonitor_dataSend.lArm_JntLim.limJntLabel.resize(upperMonitor_dataSend.lArm_JntLim.limJntNum);
    upperMonitor_dataSend.lArm_JntLim.limJntAng.resize(upperMonitor_dataSend.lArm_JntLim.limJntNum);
    upperMonitor_dataSend.lArm_JntLim.limJntLabel[0] = "body_joint_1";
    upperMonitor_dataSend.lArm_JntLim.limJntAng[0] = 0.0;
    upperMonitor_dataSend.lArm_JntLim.limJntLabel[1] = "body_joint_2";
    upperMonitor_dataSend.lArm_JntLim.limJntAng[1] = 0.0;
    upperMonitor_dataSend.lArm_JntLim.limJntLabel[2] = "head_joint_1";
    upperMonitor_dataSend.lArm_JntLim.limJntAng[2] = 0.0;
    upperMonitor_dataSend.lArm_JntLim.limJntLabel[3] = "head_joint_2";
    upperMonitor_dataSend.lArm_JntLim.limJntAng[3] = 0.0;
    upperMonitor_dataSend.lArm_JntLim.limJntLabel[4] = "arm_l_joint_8";
    upperMonitor_dataSend.lArm_JntLim.limJntAng[4] = 0.0;

    upperMonitor_dataSend.rArm_JntLim.ifUseJntLim = true;
    upperMonitor_dataSend.rArm_JntLim.limJntNum = 1;
    upperMonitor_dataSend.rArm_JntLim.limJntAng.resize(upperMonitor_dataSend.rArm_JntLim.limJntNum);
    upperMonitor_dataSend.rArm_JntLim.limJntLabel.resize(upperMonitor_dataSend.rArm_JntLim.limJntNum);
    upperMonitor_dataSend.rArm_JntLim.limJntLabel[0] = "arm_r_joint_8";
    upperMonitor_dataSend.rArm_JntLim.limJntAng[0] = 0.0;

    upperMonitor_dataSend.ifUpdateLim = false;

    //-----关节初始化信息-----
    upperMonitor_dataSend.ifReInit = false;
    upperMonitor_dataSend.lArm_stPos = lArm_stJntPos;
    upperMonitor_dataSend.rArm_stPos = rArm_stJntPos;

    upperMonitor_dataSend.ifDispJntInfo = false;
    upperMonitor_dataSend.ifUse_visualization = false;
    upperMonitor_dataSend.ifDisp_msgs = false;


    //求解器参数初始化
    upperMonitor_dataSend.optiWeight[0] = 50.0;
    upperMonitor_dataSend.optiWeight[1] = 1.0;
    upperMonitor_dataSend.optiWeight[2] = 0.002;
    upperMonitor_dataSend.optiWeight[3] = 0.01;
    upperMonitor_dataSend.optiWeight[4] = 0.000005;

    upperMonitor_dataSend.template_times = 400;
    upperMonitor_dataSend.errTol = 1e-6;

    upperMonitor_dataSend.ifPrintOptiLevel = 0;
    upperMonitor_dataSend.ifPrintOptiTime = false;

    upperMonitor_dataSend.ctlDesMsg = ctlStMsg;

    //----------上层节点信息传输函数初始化----------
    //不开启重力补偿
    armTrajMsg.gravCompMod = 0;

    //不开启限位
    armTrajMsg.JntStabMod_l.ifUseJntLim = false;
    armTrajMsg.JntStabMod_l.limJntNum = 1;
    armTrajMsg.JntStabMod_l.limJntID = { 2 };
    armTrajMsg.JntStabMod_l.limJntAng = { 0.8 };
    armTrajMsg.JntStabMod_r.ifUseJntLim = false;
    armTrajMsg.JntStabMod_r.limJntNum = 1;
    armTrajMsg.JntStabMod_r.limJntID = { 2 };
    armTrajMsg.JntStabMod_r.limJntAng = { -0.8 };

    //默认归零模式
    armTrajMsg.ctlTransMod = 0;

    //默认零位为初始位置
    armTrajMsg.ctlDesMsg.lArm_desMsg.assign(SingleArmDof, 0.0);
    armTrajMsg.ctlDesMsg.rArm_desMsg.assign(SingleArmDof, 0.0);

    armTrajMsg.ifReachEnd = true;
    armTrajMsg.ifUseTorFeedForward = false;

    //升降机默认处于遥操作模式，初始静止
    baseCtlMsg.lifter_cmd.control_mod = 0;
    baseCtlMsg.lifter_cmd.move_cmd = 0;
    baseCtlMsg.lifter_cmd.target_position = 1.0;
    baseCtlMsg.lifter_cmd.save_preset = false;

    //底盘小车默认不动
    baseCtlMsg.chassis_cmd.linear_x = 0.0;
    baseCtlMsg.chassis_cmd.linear_y = 0.0;
    baseCtlMsg.chassis_cmd.angular_z = 0.0;

    //初始化机械臂控制类
    armTrajTransCtl.init();
}

//控制参量发送模块
void RobotCtlNode::msg_publish(){

    auto temp_upperMonitorData_pub = snowbot_ctl_msgs::msg::MidNodeCtlMsgSend();
    temp_upperMonitorData_pub.header.stamp = this->now();

    //依次发送各个控制量

    //----------数据传输与插补模式选择----------
    temp_upperMonitorData_pub.ctl_trans_mod = upperMonitor_dataSend.ctl_transMod;
    temp_upperMonitorData_pub.runin_mod = upperMonitor_dataSend.runinMod;

    //----------求解模式信息----------
    temp_upperMonitorData_pub.if_use_ret = upperMonitor_dataSend.ifUseRet;
    temp_upperMonitorData_pub.if_use_temp = upperMonitor_dataSend.ifUseTemp;
    temp_upperMonitorData_pub.if_use_ret_to_start_runin =
        upperMonitor_dataSend.ifUseRetToStartRunin;
    temp_upperMonitorData_pub.if_streaming_joint_position =
        upperMonitor_dataSend.ifStreamingJointPosition;

    for(int i = 0; i < SingleArmDof; i++){
        temp_upperMonitorData_pub.rigidity_ctl_mod_l[i] = upperMonitor_dataSend.rigidityCtl_mod_l[i];
        temp_upperMonitorData_pub.rigidity_ctl_mod_r[i] = upperMonitor_dataSend.rigidityCtl_mod_r[i];
    }

    for(int i = 0; i < SingleArmDof; i++){
        temp_upperMonitorData_pub.kp_trans_l[i] = upperMonitor_dataSend.KpTrans_l[i];
        temp_upperMonitorData_pub.kd_trans_l[i] = upperMonitor_dataSend.KdTrans_l[i];

        temp_upperMonitorData_pub.kp_trans_r[i] = upperMonitor_dataSend.KpTrans_r[i];
        temp_upperMonitorData_pub.kd_trans_r[i] = upperMonitor_dataSend.KdTrans_r[i];
    }

    //----------重力补偿控制信息----------
    temp_upperMonitorData_pub.if_use_grav_comp = upperMonitor_dataSend.GravCompCtlMsg.ifUseGravComp;
    temp_upperMonitorData_pub.if_use_fric_comp = upperMonitor_dataSend.GravCompCtlMsg.mIfUseFricComp;
    temp_upperMonitorData_pub.if_use_inp_ftr = upperMonitor_dataSend.GravCompCtlMsg.mIfUseInpFtr;
    temp_upperMonitorData_pub.if_use_outp_ftr = upperMonitor_dataSend.GravCompCtlMsg.mIfUseOutpFtr;
    temp_upperMonitorData_pub.grav_comp_inp_mod = upperMonitor_dataSend.GravCompCtlMsg.GravCompInpMod;

    temp_upperMonitorData_pub.st_jnt_id_l_arm = upperMonitor_dataSend.stJntID_lArm;
    temp_upperMonitorData_pub.st_jnt_id_r_arm = upperMonitor_dataSend.stJntID_rArm;
    temp_upperMonitorData_pub.urdf_choose = upperMonitor_dataSend.urdf_choose;

    temp_upperMonitorData_pub.upper_monitor_time = TempTime;
    for(int i = 0; i < SingleArmDof; i++){

        //----------插补信息包装----------
        temp_upperMonitorData_pub.traj_time_l_arm[i] = upperMonitor_dataSend.TrajTime[0][i];
        temp_upperMonitorData_pub.traj_time_r_arm[i] = upperMonitor_dataSend.TrajTime[1][i];

        temp_upperMonitorData_pub.acc_rate_base_l_arm[i] = upperMonitor_dataSend.accRateBase[0][i];
        temp_upperMonitorData_pub.acc_rate_base_r_arm[i] = upperMonitor_dataSend.accRateBase[1][i];

        temp_upperMonitorData_pub.l_arm_if_use_stab[i] = upperMonitor_dataSend.m_stabMsg[0][i].ifUseStab;
        temp_upperMonitorData_pub.r_arm_if_use_stab[i] = upperMonitor_dataSend.m_stabMsg[1][i].ifUseStab;

        temp_upperMonitorData_pub.l_arm_start_stab_times[i] = upperMonitor_dataSend.m_stabMsg[0][i].start_stabTimes;
        temp_upperMonitorData_pub.r_arm_start_stab_times[i] = upperMonitor_dataSend.m_stabMsg[1][i].start_stabTimes;

        temp_upperMonitorData_pub.l_arm_des_stab_times[i] = upperMonitor_dataSend.m_stabMsg[0][i].des_stabTimes;
        temp_upperMonitorData_pub.r_arm_des_stab_times[i] = upperMonitor_dataSend.m_stabMsg[1][i].des_stabTimes;

        //----------关节恒定输出位置开关----------
        temp_upperMonitorData_pub.l_arm_if_jnt_stab[i] = upperMonitor_dataSend.ifJntStab[0][i];
        temp_upperMonitorData_pub.r_arm_if_jnt_stab[i] = upperMonitor_dataSend.ifJntStab[1][i];

        temp_upperMonitorData_pub.if_redefine_stab_jnt_ang = upperMonitor_dataSend.ifRedefineJntTrajStab;
    }

    //----------关节限制信息----------
    temp_upperMonitorData_pub.l_arm_if_use_jntlim = upperMonitor_dataSend.lArm_JntLim.ifUseJntLim;
    temp_upperMonitorData_pub.r_arm_if_use_jntlim = upperMonitor_dataSend.rArm_JntLim.ifUseJntLim;

    temp_upperMonitorData_pub.l_arm_jntlim_num = upperMonitor_dataSend.lArm_JntLim.limJntNum;
    temp_upperMonitorData_pub.r_arm_jntlim_num = upperMonitor_dataSend.rArm_JntLim.limJntNum;

    if(upperMonitor_dataSend.lArm_JntLim.ifUseJntLim){
        for(int i = 0; i < upperMonitor_dataSend.lArm_JntLim.limJntNum; i++){
            temp_upperMonitorData_pub.l_arm_limjnt_label[i] = upperMonitor_dataSend.lArm_JntLim.limJntLabel[i];
            temp_upperMonitorData_pub.l_arm_limjnt_ang[i] = upperMonitor_dataSend.lArm_JntLim.limJntAng[i];
        }
    }

    if(upperMonitor_dataSend.rArm_JntLim.ifUseJntLim){
        for(int i = 0; i < upperMonitor_dataSend.rArm_JntLim.limJntNum; i++){
            temp_upperMonitorData_pub.r_arm_limjnt_label[i] = upperMonitor_dataSend.rArm_JntLim.limJntLabel[i];
            temp_upperMonitorData_pub.r_arm_limjnt_ang[i] = upperMonitor_dataSend.rArm_JntLim.limJntAng[i];
        }
    }

    temp_upperMonitorData_pub.if_update_lim = upperMonitor_dataSend.ifUpdateLim;

    //----------关节初始化信息----------
    //是否重新初始化判断量
    temp_upperMonitorData_pub.if_reinit = upperMonitor_dataSend.ifReInit;
    if(ifFirstOutput){
        temp_upperMonitorData_pub.if_reinit = true;
    }

    //左右臂关节初始位置
    for(int i = 0; i < SingleArmDof; i++){
        temp_upperMonitorData_pub.l_arm_st_pos[i] = upperMonitor_dataSend.lArm_stPos[i];
        temp_upperMonitorData_pub.r_arm_st_pos[i] = upperMonitor_dataSend.rArm_stPos[i];
    }

    //信息打印开关量
    temp_upperMonitorData_pub.if_disp_jnt_info = upperMonitor_dataSend.ifDispJntInfo;
    temp_upperMonitorData_pub.if_use_visualization = upperMonitor_dataSend.ifUse_visualization;
    temp_upperMonitorData_pub.if_disp_msgs = upperMonitor_dataSend.ifDisp_msgs;

    //优化器参数
    for(int i = 0; i < 5; i++){
        temp_upperMonitorData_pub.opti_weight[i] = upperMonitor_dataSend.optiWeight[i];
    }
    temp_upperMonitorData_pub.solver_template_times = upperMonitor_dataSend.template_times;
    temp_upperMonitorData_pub.err_tol = upperMonitor_dataSend.errTol;

    temp_upperMonitorData_pub.if_print_level = upperMonitor_dataSend.ifPrintOptiLevel;
    temp_upperMonitorData_pub.if_print_optidata = upperMonitor_dataSend.ifPrintOptiTime;

    //----------关节目标数据发送----------
    //笛卡尔空间数据
    for(int i = 0; i < 6; i++){
        temp_upperMonitorData_pub.l_arm_des_pos_descartes[i] = upperMonitor_dataSend.ctlDesMsg.desPos_l_Descartes[i];
        temp_upperMonitorData_pub.r_arm_des_pos_descartes[i] = upperMonitor_dataSend.ctlDesMsg.desPos_r_Descartes[i];

        temp_upperMonitorData_pub.l_arm_des_tor_descartes[i] = upperMonitor_dataSend.ctlDesMsg.desTor_l_Descartes[i];
        temp_upperMonitorData_pub.r_arm_des_tor_descartes[i] = upperMonitor_dataSend.ctlDesMsg.desTor_r_Descartes[i];
    }

    //关节空间数据
    for(int i = 0; i < SingleArmDof; i++){
        temp_upperMonitorData_pub.l_arm_des_pos_jntspace[i] = upperMonitor_dataSend.ctlDesMsg.desPos_l_JointSpace[i];
        temp_upperMonitorData_pub.r_arm_des_pos_jntspace[i] = upperMonitor_dataSend.ctlDesMsg.desPos_r_JointSpace[i];

        temp_upperMonitorData_pub.l_arm_des_tor_jntspace[i] = upperMonitor_dataSend.ctlDesMsg.desTor_l_JointSpace[i];
        temp_upperMonitorData_pub.r_arm_des_tor_jntspace[i] = upperMonitor_dataSend.ctlDesMsg.desTor_r_JointSpace[i];
    }

    upperMointor_Msg_send->publish(temp_upperMonitorData_pub);
    ifFirstOutput = false;
}

void RobotCtlNode::fdback_get(){
    rclcpp::spin_some(this->get_node_base_interface());  // 非阻塞检查回调
}

//debug画图信息发送模块
void RobotCtlNode::dbg_publish(std::vector<double> mInp_1, int len){
    auto tempDbgData_pub = snowbot_ctl_msgs::msg::DbgInformation();

    int temp_len = len;
    if(len > 7){
        temp_len = 7;
    }

    for(int i = 0; i < temp_len; i++){
        tempDbgData_pub.float_reserve[i] = mInp_1[i];
    }

    mDbgInformation->publish(tempDbgData_pub);
}
//第二种debug画图信息发送模块——一次发送两个轨迹的信息
void RobotCtlNode::dbg_publish_2(std::vector<double> mInp_1, std::vector<double> mInp_2){
    auto tempDbgData_pub = snowbot_ctl_msgs::msg::DbgInformation();

    int temp_len_1 = mInp_1.size();
    if(temp_len_1 > 7){
        temp_len_1 = 7;
    }
    for(int i = 0; i < temp_len_1; i++){
        tempDbgData_pub.float_reserve[i] = mInp_1[i];
    }

    int temp_len_2 = mInp_2.size();
    if(temp_len_2 > 7){
        temp_len_2 = 7;
    }
    for(int i = 0; i < temp_len_2; i++){
        tempDbgData_pub.float_reserve_b[i] = mInp_2[i];
    }

    mDbgInformation->publish(tempDbgData_pub);
}

//--------------

//-----辅助变量-----

//-----------------

//ros2循环函数
void RobotCtlNode::run()
{
    //用户交互
    std::cout << "开始执行上位机程序" << std::endl;
    std::cout << "====================arm_control_app_v2.0.1====================" << std::endl;

    //若调用python程序，需要在此处初始化python的解释器

    //初始化执行频率
    float loopRate_ubRound = 1/TempTime;
    int loopRate = std::round(loopRate_ubRound);
    std::cout << "轨迹执行频率为：" << loopRate << " Hz。" << std::endl;
    rclcpp::Rate loop_rate(loopRate);

    OutpInit();

    /*  信息发送代码初始化
    -----------------------
    */
    if (control_source_ == ControlSource::Demo) {
        demoTrajInit();
    } else if (control_source_ == ControlSource::Vr) {
        xr_teleComu_init();
    } else {
        jointPositionInit();
    }

    int temp_i = 0;

    // 键盘监听相关变量
    bool is_paused = false;
    bool key_pressed = false;
    char last_key = '\0';
    const int CHECK_INTERVAL = 10;  // 每10次循环检查一次键盘
    int check_counter = 0;


    //开始ros2循环
    while ((rclcpp::ok())){

        //------------------------------
        //----------键盘监听代码----------
        //本代码段用于暂停程序执行

        // 定期检查键盘输入（简化版本）
        check_counter++;
        if (check_counter >= CHECK_INTERVAL) {
            check_counter = 0;

            // 使用非阻塞方式检查标准输入
            struct timeval tv = {0, 0};
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);

            int ready = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);


            if (ready > 0) {
                std::string input;
                std::getline(std::cin, input);

                if (!input.empty()) {
                    char ch = input[0];
                    if (ch == 'r' || ch == 'R') {
                        if (!is_paused) {
                            is_paused = true;
                            std::cout << "\n=== 程序已暂停 ===" << std::endl;
                            std::cout << "按 E 键恢复输出" << std::endl;
                        }
                    } else if (ch == 'e' || ch == 'E') {
                        if (is_paused) {
                            is_paused = false;
                            std::cout << "\n=== 程序已恢复 ===" << std::endl;
                        }
                    }
                }
            }
        }


        //如果处于暂停状态，跳过控制输出
        if (is_paused) {
            // 暂停状态下只处理必要的ROS回调
            rclcpp::spin_some(this->get_node_base_interface());
            msg_publish();
            loop_rate.sleep();
            continue;
        }

        //------------------------------
        //从中层节点获取反馈信息，并基于此进行控制

        //更新时间戳
        global_timestamp_ = this->now();

        //获取一次反馈
        fdback_get();


        //-----初始段控制-----

        //若为初始轮，则依据反馈值特性判断是否要进入强制初始化，以保证执行其他命令前机械臂一定在零位
        if(ifInitTemp){

            //防呆设计 -- 只有先打开中层节点，才能打开这个上层节点
            if(!this->has_received_ret_){
                std::cout << "警告 -- 必须先打开机器人中层节点，才能打开这个上层节点" << std::endl;
                loop_rate.sleep();
                continue;
            }

            double threshold_ang = 0.2;
            // 使用std::any_of检查是否有任何元素满足条件
            bool leftExceeded = std::any_of(lArm_pos_fdback.begin(), lArm_pos_fdback.end(),
                [threshold_ang](double value) { return std::abs(value) >= threshold_ang; });

            bool rightExceeded = std::any_of(rArm_pos_fdback.begin(), rArm_pos_fdback.end(),
                [threshold_ang](double value) { return std::abs(value) >= threshold_ang; });
            //特征值：是否强制进入归零模式
            ifForceToRetZero = leftExceeded || rightExceeded;

            if(ifForceToRetZero){
                std::cout << "初始化阶段 -- 进入归零模式，回归零位" << std::endl;
            }
            ifInitTemp = false;
        }

        //若满足强制归零条件，则需要执行强制归零函数
        if(ifForceToRetZero){
            //控制器模式刷新（其余参量均使用初始值即可）
            upperMonitor_dataSend.ctl_transMod = 0;


            //仅仅能接收到中层节点数据时候，才会开始归零（保障鲁棒）
            if(this->has_received_ret_){
                bool allReachEnd_l = std::all_of(lArm_ifReach_end.begin(), lArm_ifReach_end.end(), [](bool val) { return val; });
                bool allReachEnd_r = std::all_of(rArm_ifReach_end.begin(), rArm_ifReach_end.end(), [](bool val) { return val; });
                bool ifAllEnd = allReachEnd_l && allReachEnd_r;

                //如果到达零位，则退出强制归零
                if(ifAllEnd){
                    ifForceToRetZero = false;
                }
            }
        }

        //-------------------

        //输出一次控制值，只有成功与下位机通讯成功，且非强制归零状态时时才调用发送函数
        if(this->has_received_ret_ && (!ifForceToRetZero)){

            //成功获取反馈值时候，或者中层节点处于debug模式的时候，据此发送
            bool allReachEnd_l = std::all_of(lArm_ifReach_end.begin(), lArm_ifReach_end.end(), [](bool val) { return val; });
            bool allReachEnd_r = std::all_of(rArm_ifReach_end.begin(), rArm_ifReach_end.end(), [](bool val) { return val; });
            bool ifAllEnd = allReachEnd_l && allReachEnd_r;
            bool ifAllReachEnd = allReachEnd_l && allReachEnd_r;
            // Topic 是连续目标流：每次收到新目标都下发；
            // Demo/VR 仍保持原来的到位后刷新逻辑。
            bool ifTrigger = control_source_ == ControlSource::Topic
                ? joint_position_cmd_pending_
                : (ifAllEnd || ifFirstRun);

            if(ifTrigger){

                /*  依据反馈值，在此处作数据处理
                -------------------------------
                */
                if (control_source_ == ControlSource::Demo) {
                    demoTrajOutp();
                    TrajOutp(1);
                } else if (control_source_ == ControlSource::Vr) {
                    xr_teleComu_outp();
                } else {
                    jointPositionOutp();
                }
            }

            if(ifTrigger){
                msg_publish();
                ifFirstRun = false;
                joint_position_cmd_pending_ = false;
            }

            //更新标志参量
            this->has_received_ret_ = false;

        }else{
            msg_publish();
        }

        //---------------------------------------------------
        //debug输出
        std::vector<double> dbg_1, dbg_2;
        dbg_1.assign(7, 0.0);
        dbg_2.assign(7, 0.0);

        dbg_publish_2(dbg_1, dbg_2);
        //---------------------------------------------------

        //尾处理代码
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "程序已安全退出");
}



//--------------

//独立 Topic 关节位置控制：固定使用关节位置模式和高刚度参数。
void RobotCtlNode::jointPositionInit()
{
    jointPositionMsg.gravCompMod = 1;
    jointPositionMsg.JntStabMod_l.ifUseJntLim = false;
    jointPositionMsg.JntStabMod_l.limJntNum = 1;
    jointPositionMsg.JntStabMod_l.limJntID = {3};
    jointPositionMsg.JntStabMod_l.limJntAng = {0.0};
    jointPositionMsg.JntStabMod_r.ifUseJntLim = false;
    jointPositionMsg.JntStabMod_r.limJntNum = 1;
    jointPositionMsg.JntStabMod_r.limJntID = {3};
    jointPositionMsg.JntStabMod_r.limJntAng = {0.0};
    // Topic 是连续关节目标流，使用连续轨迹模式，避免每帧重置点位轨迹计数器。
    jointPositionMsg.ctlTransMod = 3;
    jointPositionMsg.ifReachEnd = true;
    // 该开关对应尚未实现的外部目标力矩前馈；重力和摩擦补偿由独立开关控制。
    jointPositionMsg.ifUseTorFeedForward = false;

    auto& cmd = jointPositionMsg.ctlDesMsg;
    // 与 Demo 点位轨迹一致：使用控制器上一段轨迹的终点作为下一段起点，
    // 不要每次新 Topic 命令都从伺服反馈重新起轨。
    // 仿真反馈当前由 servo_node 发布为零值，Topic 轨迹使用控制器内部上一段目标作为起点。
    cmd.ifInitUseRet = 0;
    cmd.lArm_desMsg = lArm_zeroPos;
    cmd.rArm_desMsg = rArm_zeroPos;
    cmd.lArm_rigidity_mod.assign(SingleArmDof, 3);
    cmd.rArm_rigidity_mod.assign(SingleArmDof, 3);
    cmd.lArm_JntKp = lArm_defaultKp_high;
    cmd.lArm_JntKd = lArm_defaultKd_high;
    cmd.rArm_JntKp = rArm_defaultKp_high;
    cmd.rArm_JntKd = rArm_defaultKd_high;

    cmd.m_SingleDotCtlMsg.ifUseTemp = 0;
    cmd.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    cmd.m_SingleDotCtlMsg.JntTrajTime_L.assign(SingleArmDof, 0.02);
    cmd.m_SingleDotCtlMsg.JntTrajTime_R.assign(SingleArmDof, 0.02);
    cmd.m_SingleDotCtlMsg.JntAccRate_L.assign(SingleArmDof, 1.2);
    cmd.m_SingleDotCtlMsg.JntAccRate_R.assign(SingleArmDof, 1.2);

    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    // Topic 模式收到新目标后立即从当前反馈开始插补，不在起点停留 1 秒。
    stabMsg.start_stabTimes = 0;
    stabMsg.des_stabTimes = 50;
    cmd.m_SingleDotCtlMsg.lArm_stabMsg.assign(SingleArmDof, stabMsg);
    cmd.m_SingleDotCtlMsg.rArm_stabMsg.assign(SingleArmDof, stabMsg);

    armTrajTransCtl.JntStabMod(
        std::vector<int>(SingleArmDof, 0),
        std::vector<int>(SingleArmDof, 0));

    RCLCPP_INFO(this->get_logger(),
                "Topic关节位置接口已就绪：/joint_position_cmd，高刚度模式");
}

void RobotCtlNode::jointPositionOutp()
{
    upperMonitor_dataSend = armTrajTransCtl.trans_and_outp(
        jointPositionMsg.gravCompMod,
        jointPositionMsg.JntStabMod_l,
        jointPositionMsg.JntStabMod_r,
        jointPositionMsg.ctlTransMod,
        jointPositionMsg.ctlDesMsg,
        jointPositionMsg.ifReachEnd,
        jointPositionMsg.ifUseTorFeedForward,
        1);

    // Topic smoothing belongs to the middle-layer stateful follower. Do not
    // feed publisher restarts through the legacy upper-layer spline history.
    upperMonitor_dataSend.ctlDesMsg.desPos_l_JointSpace = jointPositionMsg.ctlDesMsg.lArm_desMsg;
    upperMonitor_dataSend.ctlDesMsg.desPos_r_JointSpace = jointPositionMsg.ctlDesMsg.rArm_desMsg;
    upperMonitor_dataSend.ctlDesMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    upperMonitor_dataSend.ctlDesMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    // Topic 是 50Hz 连续关节目标流，使用连续轨迹模式；
    // 每个目标点覆盖一个上层周期，避免点位轨迹被反复重置。
    upperMonitor_dataSend.ctl_transMod = 3;
    // 仿真反馈不是实际关节位置，不能用全零反馈重置轨迹起点。
    upperMonitor_dataSend.ifUseRet = 0;
    upperMonitor_dataSend.ifUseTemp = 0;
    // 这个标志只用于标识“Topic 点位模式”；并不代表每条消息都重建轨迹。
    // 插补起点仍沿用上一段控制器输出，不能改成当前反馈，否则会抖动。
    // 仿真反馈为零值，下层沿用内部上一段轨迹状态作为插补起点。
    upperMonitor_dataSend.ifUseRetToStartRunin = false;
    upperMonitor_dataSend.ifStreamingJointPosition = true;
    constexpr double stream_traj_time = 0.02;
    upperMonitor_dataSend.TrajTime[0].assign(SingleArmDof, stream_traj_time);
    upperMonitor_dataSend.TrajTime[1].assign(SingleArmDof, stream_traj_time);
    upperMonitor_dataSend.accRateBase[0].assign(SingleArmDof, 1.2);
    upperMonitor_dataSend.accRateBase[1].assign(SingleArmDof, 1.2);

    // 中层将重力输出到 grav 通道，将 Stribeck 摩擦补偿输出到 feedforward 通道。
    upperMonitor_dataSend.GravCompCtlMsg.mIfUseFricComp = 1;

    RCLCPP_INFO(this->get_logger(),
                "下发 Topic 关节命令: L1=%.3f L4=%.3f R1=%.3f R4=%.3f",
                jointPositionMsg.ctlDesMsg.lArm_desMsg[0],
                jointPositionMsg.ctlDesMsg.lArm_desMsg[3],
                jointPositionMsg.ctlDesMsg.rArm_desMsg[0],
                jointPositionMsg.ctlDesMsg.rArm_desMsg[3]);
}

//通讯信息回调函数
void RobotCtlNode::joint_position_cmd_callback(
    const snowbot_ctl_msgs::msg::JointPositionCommand::SharedPtr msg)
{
    if (control_source_ != ControlSource::Topic) {
        RCLCPP_WARN(this->get_logger(),
                    "忽略 /joint_position_cmd：当前 control_source 不是 topic");
        return;
    }

    const auto finite_positions = [](const auto& values) {
        return std::all_of(values.begin(), values.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!finite_positions(msg->l_arm_position) || !finite_positions(msg->r_arm_position)) {
        RCLCPP_ERROR(this->get_logger(),
                     "拒绝 /joint_position_cmd：关节位置包含非有限值");
        return;
    }

    const auto same_position = [](const auto& incoming, const std::vector<double>& current) {
        if (current.size() != incoming.size()) {
            return false;
        }
        return std::equal(
            incoming.begin(), incoming.end(), current.begin(),
            [](double next, double old) { return std::abs(next - old) < 1e-9; });
    };

    // 同一目标持续发布时不重复重建轨迹，避免到位后被反复重新触发。
    if (same_position(msg->l_arm_position, jointPositionMsg.ctlDesMsg.lArm_desMsg) &&
        same_position(msg->r_arm_position, jointPositionMsg.ctlDesMsg.rArm_desMsg)) {
        return;
    }

    jointPositionMsg.ctlDesMsg.lArm_desMsg.assign(
        msg->l_arm_position.begin(), msg->l_arm_position.end());
    jointPositionMsg.ctlDesMsg.rArm_desMsg.assign(
        msg->r_arm_position.begin(), msg->r_arm_position.end());
    joint_position_cmd_pending_ = true;

    RCLCPP_INFO(this->get_logger(), "已接收新的 Topic 关节位置命令");
}

void RobotCtlNode::message_callback(const snowbot_ctl_msgs::msg::MidNodeCtlMsgRet::SharedPtr msg)
{

    //关节信息反馈值装入
    for(int i = 0; i < SingleArmDof; i++){
        lArm_pos_fdback[i] = msg->l_arm_pos_fdback[i];
        lArm_spd_fdback[i] = msg->l_arm_spd_fdback[i];
        lArm_iq_fdback[i] = msg->l_arm_iq_fdback[i];

        rArm_pos_fdback[i] = msg->r_arm_pos_fdback[i];
        rArm_spd_fdback[i] = msg->r_arm_spd_fdback[i];
        rArm_iq_fdback[i] = msg->r_arm_iq_fdback[i];
    }

    //状态反馈值装入
    for(int i = 0; i < SingleArmDof; i++){

        //是否到达目标位置
        lArm_ifReach_end[i] = msg->l_arm_if_jnt_reach_despos[i];
        rArm_ifReach_end[i] = msg->r_arm_if_jnt_reach_despos[i];

        //本轮反馈目标位置、速度、三类前馈力
        lArm_temp_desPos[i] = msg->l_arm_jntpos_des[i];
        lArm_temp_desVel[i] = msg->l_arm_jntspd_des[i];

        lArm_temp_desGravComp[i] = msg->l_arm_jnttor_gravcomp[i];
        lArm_temp_desInteriaComp[i] = msg->l_arm_jnttor_interiacomp[i];
        lArm_temp_desFeedForward[i] = msg->l_arm_jnttor_feedforward[i];

        rArm_temp_desPos[i] = msg->r_arm_jntpos_des[i];
        rArm_temp_desVel[i] = msg->r_arm_jntspd_des[i];

        rArm_temp_desGravComp[i] = msg->r_arm_jnttor_gravcomp[i];
        rArm_temp_desInteriaComp[i] = msg->r_arm_jnttor_interiacomp[i];
        rArm_temp_desFeedForward[i] = msg->r_arm_jnttor_feedforward[i];
    }

    this->has_received_ret_ = true;
}

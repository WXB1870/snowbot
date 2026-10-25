#include "JntController/JntTestCtl.h"




//-------------------------------------------------------------------------------------
currentTrackTest::currentTrackTest(rclcpp::Node* node) : node_ptr_(node){

    //初始化消息发送
    currentTestMsg = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::TestMsg>(
        "/test/current_track_msg_send",
        10
    );

    //初始化vector容器
    lArm_desCurrent.assign(7, 0.0);
    rArm_desCurrent.assign(7, 0.0);

    lArm_currentFdback.assign(7, 0.0);
    rArm_currentFdback.assign(7, 0.0);
}

void currentTrackTest::publish(){
    auto temp_CurMsgPub = snowbot_ctl_msgs::msg::TestMsg();

    //刷新时间戳
    auto m_now = node_ptr_->now();
    temp_CurMsgPub.timestamp_sec = m_now.seconds() + m_now.nanoseconds() * 1e-9;

    for(int i = 0; i < 7; i++){
        temp_CurMsgPub.l_reserve_1[i] = lArm_desCurrent[i];
        temp_CurMsgPub.r_reserve_1[i] = rArm_desCurrent[i];

        temp_CurMsgPub.l_reserve_2[i] = lArm_currentFdback[i];
        temp_CurMsgPub.r_reserve_2[i] = rArm_currentFdback[i];

        temp_CurMsgPub.l_reserve_3[i] = lArm_desCurrent[i] - lArm_currentFdback[i];
        temp_CurMsgPub.r_reserve_3[i] = rArm_desCurrent[i] - rArm_currentFdback[i];
    }

    currentTestMsg->publish(temp_CurMsgPub);
}

void currentTrackTest::msgGetAndPlot_PVTmod(
    std::vector<double> lArm_posDes, std::vector<double> lArm_posRet, std::vector<double> lArm_Kp,
    std::vector<double> rArm_posDes, std::vector<double> rArm_posRet, std::vector<double> rArm_Kp,
    
    std::vector<double> lArm_velDes, std::vector<double> lArm_velRet, std::vector<double> lArm_Kd,
    std::vector<double> rArm_velDes, std::vector<double> rArm_velRet, std::vector<double> rArm_Kd,

    std::vector<double> lArm_TorFeedForward, std::vector<double> rArm_TorFeedForward,

    std::vector<double> l_current_fdback, std::vector<double> r_current_fdback
){
    //计算目标电流：
    for(int i = 0; i < 7; i++){
        lArm_desCurrent[i] = 
            ((lArm_posDes[i] - l_retComp[i] * lArm_posRet[i])*lArm_Kp[i] + (lArm_velDes[i] - l_retComp[i] * lArm_velRet[i])*lArm_Kd[i] + lArm_TorFeedForward[i]) / Joint_Kt[i];
    
        rArm_desCurrent[i] = 
            ((rArm_posDes[i] - r_retComp[i] * rArm_posRet[i])*rArm_Kp[i] + (rArm_velDes[i] - r_retComp[i] * rArm_velRet[i])*rArm_Kd[i] + rArm_TorFeedForward[i]) / Joint_Kt[i + 7];
    }

    //装入实际反馈电流：
    lArm_currentFdback = l_current_fdback;
    rArm_currentFdback = r_current_fdback;

    //消息发布
    publish();
}

//-------------------------------------------------------------------------------------

pvTrackOutp::pvTrackOutp(rclcpp::Node* node) : node_ptr_(node) {

    //初始化消息发送
    pvTrackPub = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::TestMsg>(
        "/test/pos_vel_track_msg_pub",
        10
    );

    //初始化成员变量：
    lArm_posDesLst.assign(7, 0.0);
    rArm_posDesLst.assign(7, 0.0);

    lArm_velDesLst.assign(7, 0.0);
    rArm_velDesLst.assign(7, 0.0);
}

void pvTrackOutp::msgGetAndPlot_PVTmod(
    std::vector<double> lArm_posDes, std::vector<double> lArm_posRet,
    std::vector<double> lArm_velDes, std::vector<double> lArm_velRet,

    std::vector<double> rArm_posDes, std::vector<double> rArm_posRet,
    std::vector<double> rArm_velDes, std::vector<double> rArm_velRet
){
    auto tempPV_pub = snowbot_ctl_msgs::msg::TestMsg();

    //刷新时间戳
    auto m_now = node_ptr_->now();
    tempPV_pub.timestamp_sec = m_now.seconds() + m_now.nanoseconds() * 1e-9;

    for(int i = 0; i < 7; i++){
        
        //目标位置
        tempPV_pub.l_reserve_1[i] = lArm_posDes[i];
        tempPV_pub.r_reserve_1[i] = rArm_posDes[i];

        //目标速度
        tempPV_pub.l_reserve_2[i] = lArm_velDes[i];
        tempPV_pub.r_reserve_2[i] = rArm_velDes[i];

        //目标加速度
        tempPV_pub.l_reserve_3[i] = (lArm_velDes[i] - lArm_velDesLst[i])/tempTime;
        tempPV_pub.r_reserve_3[i] = (rArm_velDes[i] - rArm_velDesLst[i])/tempTime;

        //实际位置
        tempPV_pub.l_reserve_4[i] = l_retComp[i] * lArm_posRet[i];
        tempPV_pub.r_reserve_4[i] = r_retComp[i] * rArm_posRet[i];

        //实际速度
        tempPV_pub.l_reserve_5[i] = l_retComp[i] * lArm_velRet[i];
        tempPV_pub.r_reserve_5[i] = r_retComp[i] * rArm_velRet[i];

        //位置偏差
        tempPV_pub.l_reserve_6[i] = (lArm_posDes[i] - l_retComp[i] * lArm_posRet[i]);
        tempPV_pub.r_reserve_6[i] = (rArm_posDes[i] - r_retComp[i] * rArm_posRet[i]);

        //速度偏差
        tempPV_pub.l_reserve_7[i] = (lArm_velDes[i] - l_retComp[i] * rArm_posRet[i]);
        tempPV_pub.r_reserve_7[i] = (rArm_posDes[i] - r_retComp[i] * rArm_posRet[i]);

    }

    lArm_posDesLst = lArm_posDes;
    rArm_posDesLst = rArm_posDes;

    lArm_velDesLst = lArm_velDes;
    rArm_velDesLst = rArm_velDes;

    pvTrackPub->publish(tempPV_pub);
}

//-------------------------------------------------------------------------------------

TorCompVisu::TorCompVisu(rclcpp::Node* node) : node_ptr_(node) {
    //初始化消息发送变量
    TorCompVisu_pub = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::TestMsg>(
        "/test/TorComp_track_msg_pub",
        10
    );
}

void TorCompVisu::msgGetAndPlot_PVTmod(
    std::vector<double> lArm_gravComp, std::vector<double> rArm_gravComp,
    std::vector<double> lArm_coriolisComp, std::vector<double> rArm_coriolisComp,
    std::vector<double> lArm_accTorComp, std::vector<double> rArm_accTorComp
){
    auto tempTor_pub = snowbot_ctl_msgs::msg::TestMsg();

    //刷新时间戳
    auto m_now = node_ptr_->now();
    tempTor_pub.timestamp_sec = m_now.seconds() + m_now.nanoseconds() * 1e-9;

    for(int i = 0; i < 7; i++){
        tempTor_pub.l_reserve_1[i] = lArm_gravComp[i];
        tempTor_pub.r_reserve_1[i] = rArm_gravComp[i];

        tempTor_pub.l_reserve_2[i] = lArm_coriolisComp[i];
        tempTor_pub.r_reserve_2[i] = rArm_coriolisComp[i];

        tempTor_pub.l_reserve_3[i] = lArm_accTorComp[i];
        tempTor_pub.r_reserve_3[i] = rArm_accTorComp[i];

        tempTor_pub.l_reserve_4[i] = lArm_coriolisComp[i] + lArm_accTorComp[i];
        tempTor_pub.r_reserve_4[i] = rArm_coriolisComp[i] + rArm_accTorComp[i];

        tempTor_pub.l_reserve_5[i] = lArm_gravComp[i] + lArm_coriolisComp[i] + lArm_accTorComp[i];
        tempTor_pub.r_reserve_5[i] = rArm_gravComp[i] + rArm_coriolisComp[i] + rArm_accTorComp[i];
    }

    TorCompVisu_pub->publish(tempTor_pub);
}

//-------------------------------------------------------------------------------------

K_compVisu::K_compVisu(rclcpp::Node* node) : node_ptr_(node){
    //初始化消息发送变量
    KcompPublisher = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::TestMsg>(
        "/test/K_comp_track_msg_pub",
        10
    );
}

void K_compVisu::msgGetAndPlot_PVTmod(
    std::vector<double> lArm_posComp, std::vector<double> rArm_posComp,
    std::vector<double> lArm_origin_pos, std::vector<double> rArm_origin_pos,
    std::vector<double> lArm_comped_pos, std::vector<double> rArm_comped_pos
){
    auto tempKcomp_pub = snowbot_ctl_msgs::msg::TestMsg();

    //刷新时间戳
    auto m_now = node_ptr_->now();
    tempKcomp_pub.timestamp_sec = m_now.seconds() + m_now.nanoseconds() * 1e-9;

    for(int i = 0; i < 7; i++){
        tempKcomp_pub.l_reserve_1[i] = lArm_posComp[i];
        tempKcomp_pub.r_reserve_1[i] = rArm_posComp[i];

        tempKcomp_pub.l_reserve_2[i] = lArm_origin_pos[i];
        tempKcomp_pub.r_reserve_2[i] = rArm_origin_pos[i];

        tempKcomp_pub.l_reserve_3[i] = lArm_comped_pos[i];
        tempKcomp_pub.r_reserve_3[i] = rArm_comped_pos[i];
    }

    KcompPublisher->publish(tempKcomp_pub);
}

//所有补偿的补偿结果可视化
void K_compVisu::allCompRes_view(
    std::vector<double> l_inp_1, std::vector<double> r_inp_1,
    std::vector<double> l_inp_2, std::vector<double> r_inp_2,
    std::vector<double> l_inp_3, std::vector<double> r_inp_3,
    std::vector<double> l_inp_4, std::vector<double> r_inp_4,
    std::vector<double> l_inp_5, std::vector<double> r_inp_5,
    std::vector<double> l_inp_6, std::vector<double> r_inp_6,
    std::vector<double> l_inp_7, std::vector<double> r_inp_7
){
    auto tempVisu = snowbot_ctl_msgs::msg::TestMsg();

    auto m_now = node_ptr_->now();
    tempVisu.timestamp_sec = m_now.seconds() + m_now.nanoseconds() * 1e-9;

    //输入量装入
    for(int i = 0; i < 7; i++){
        tempVisu.l_reserve_1[i] = l_inp_1[i];
        tempVisu.r_reserve_1[i] = r_inp_1[i];

        tempVisu.l_reserve_2[i] = l_inp_2[i];
        tempVisu.r_reserve_2[i] = r_inp_2[i];

        tempVisu.l_reserve_3[i] = l_inp_3[i];
        tempVisu.r_reserve_3[i] = r_inp_3[i];

        tempVisu.l_reserve_4[i] = l_inp_4[i];
        tempVisu.r_reserve_4[i] = r_inp_4[i];

        tempVisu.l_reserve_5[i] = l_inp_5[i];
        tempVisu.r_reserve_5[i] = r_inp_5[i];

        tempVisu.l_reserve_6[i] = l_inp_6[i];
        tempVisu.r_reserve_6[i] = r_inp_6[i];

        tempVisu.l_reserve_7[i] = l_inp_7[i];
        tempVisu.r_reserve_7[i] = r_inp_7[i];
    }

    KcompPublisher->publish(tempVisu);
}

//-------------------------------------------------------------------------------------
fricCompRes_visu::fricCompRes_visu(rclcpp::Node* node): node_ptr_(node){
    //消息发送变量初始化
    fricCompVisuPublisher = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::TestMsg>(
        "/test/frictionComp_result_pub",
        10
    );
}

void fricCompRes_visu::msgGetAndPlot_PVTmod(
    std::vector<double> lArm_vel, std::vector<double> rArm_vel,
    std::vector<double> lArm_tor, std::vector<double> rArm_tor,
    std::vector<double> lArm_fricCompRes, std::vector<double> rArm_fricCompRes,
    std::vector<double> lArm_torOri_feedForward, std::vector<double> rArm_torOri_feedForward,
    std::vector<double> lArm_tor_feedForward, std::vector<double> rArm_tor_feedForward
){
    auto tempFric_pub = snowbot_ctl_msgs::msg::TestMsg();

    auto m_now = node_ptr_->now();
    tempFric_pub.timestamp_sec = m_now.seconds() + m_now.nanoseconds() * 1e-9;

    for(int i = 0; i < 7; i++){
        tempFric_pub.l_reserve_1[i] = lArm_vel[i];
        tempFric_pub.r_reserve_1[i] = rArm_vel[i];

        tempFric_pub.l_reserve_2[i] = lArm_tor[i];
        tempFric_pub.r_reserve_2[i] = rArm_tor[i];

        tempFric_pub.l_reserve_3[i] = lArm_fricCompRes[i];
        tempFric_pub.r_reserve_3[i] = rArm_fricCompRes[i];

        tempFric_pub.l_reserve_4[i] = lArm_torOri_feedForward[i];
        tempFric_pub.r_reserve_4[i] = rArm_torOri_feedForward[i];

        tempFric_pub.l_reserve_5[i] = lArm_tor_feedForward[i];
        tempFric_pub.r_reserve_5[i] = rArm_tor_feedForward[i];
    }

    fricCompVisuPublisher->publish(tempFric_pub);
}
//-------------------------------------------------------------------------------------
//控制结果综合可视化
Comprehensive_visu::Comprehensive_visu(rclcpp::Node* node) : node_ptr_(node){

    //消息发送变量初始化
    Jnt_ctl_res_pub = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::JntCtlResVisu>(
        "/ctl_msg_pub/jnt_ctl_result_publish",
        10
    );

    Jnt_comp_res_pub = node_ptr_->create_publisher<snowbot_ctl_msgs::msg::JntCompResVisu>(
        "/ctl_msg_pub/jnt_comp_result_publish",
        10
    );

    //辅助变量初始化
    lArm_velDes_last.assign(7, 0.0);
    rArm_velDes_last.assign(7, 0.0);

    auto m_now = node_ptr_->now();
    time_last = m_now.seconds() + m_now.nanoseconds() * 1e-9;
}

//控制结果发送
void Comprehensive_visu::ctl_res_pub_PVTmod(
    std::vector<double> l_posDes_origin, std::vector<double> r_posDes_origin,
    std::vector<double> l_posDes, std::vector<double> r_posDes,
    std::vector<double> l_velDes_origin, std::vector<double> r_velDes_origin,
    std::vector<double> l_velDes, std::vector<double> r_velDes,
    std::vector<double> l_torFeedforward, std::vector<double> r_torFeedforward,
    std::vector<double> l_Kp, std::vector<double> r_Kp,
    std::vector<double> l_Kd, std::vector<double> r_Kd,

    std::vector<double> l_posRet, std::vector<double> r_posRet,
    std::vector<double> l_velRet, std::vector<double> r_velRet,
    std::vector<double> l_currentRet, std::vector<double> r_currentRet,

    double time_spot
){
    auto tempResPub = snowbot_ctl_msgs::msg::JntCtlResVisu();

    for(int i = 0; i < 7; i++){
        tempResPub.l_des_pos_ori[i] = l_posDes_origin[i];
        tempResPub.r_des_pos_ori[i] = r_posDes_origin[i];

        tempResPub.l_des_pos[i] = l_posDes[i];
        tempResPub.r_des_pos[i] = r_posDes[i];

        tempResPub.l_des_vel_ori[i] = l_velDes_origin[i];
        tempResPub.r_des_vel_ori[i] = r_velDes_origin[i];

        tempResPub.l_des_vel[i] = l_velDes[i];
        tempResPub.r_des_vel[i] = r_velDes[i];

        tempResPub.l_des_acc[i] = (l_velDes[i] - lArm_velDes_last[i])/tempTime;
        tempResPub.r_des_acc[i] = (r_velDes[i] - rArm_velDes_last[i])/tempTime;

        tempResPub.l_tor_feedforward[i] = l_torFeedforward[i];
        tempResPub.r_tor_feedforward[i] = r_torFeedforward[i];

        tempResPub.l_des_kp[i] = l_Kp[i];
        tempResPub.r_des_kp[i] = r_Kp[i];

        tempResPub.l_des_kd[i] = l_Kd[i];
        tempResPub.r_des_kd[i] = r_Kd[i];

        tempResPub.l_pos_ret[i] = l_posRet[i];
        tempResPub.r_pos_ret[i] = r_posRet[i];

        tempResPub.l_vel_ret[i] = l_velRet[i];
        tempResPub.r_vel_ret[i] = r_velRet[i];

        tempResPub.l_current_ret[i] = l_currentRet[i];
        tempResPub.r_current_ret[i] = r_currentRet[i];

        tempResPub.l_pos_err[i] = l_posDes[i] - l_posRet[i];
        tempResPub.r_pos_err[i] = r_posDes[i] - r_posRet[i];
    }

    tempResPub.time_spot = time_spot;
    tempResPub.derta_time = time_spot - time_last;

    time_last = time_spot;

    Jnt_ctl_res_pub->publish(tempResPub);
}

void Comprehensive_visu::comp_res_pub_PVTmod(
    std::vector<double> l_gravComp, std::vector<double> r_gravComp,
    std::vector<double> l_interiaComp, std::vector<double> r_interiaComp,
    std::vector<double> l_feedForwardComp, std::vector<double> r_feedForwardComp,

    std::vector<double> l_K_comp, std::vector<double> r_K_comp,
    std::vector<double> l_fricComp, std::vector<double> r_fricComp
){
    auto tempResPub = snowbot_ctl_msgs::msg::JntCompResVisu();

    for(int i = 0; i < 7; i++){
        tempResPub.l_grav_comp[i] = l_gravComp[i];
        tempResPub.r_grav_comp[i] = r_gravComp[i];

        tempResPub.l_interia_comp[i] = l_interiaComp[i];
        tempResPub.r_interia_comp[i] = r_interiaComp[i];

        tempResPub.l_feedforward_comp[i] = l_feedForwardComp[i];
        tempResPub.r_feedforward_comp[i] = r_feedForwardComp[i];

        tempResPub.l_k_comp[i] = l_K_comp[i];
        tempResPub.r_k_comp[i] = r_K_comp[i];

        tempResPub.l_fric_comp[i] = l_fricComp[i];
        tempResPub.r_fric_comp[i] = r_fricComp[i];
    }

    Jnt_comp_res_pub->publish(tempResPub);
}

//-------------------------------------------------------------------------------------
performanceTestSeal::performanceTestSeal(rclcpp::Node* node) : node_ptr_(node) {
    // 创建各个测试类的实例
    current_track = new currentTrackTest(node);
    pos_vel_track = new pvTrackOutp(node);
    torComp_track = new TorCompVisu(node);
    k_comp_visu = new K_compVisu(node);
    fric_comp_visu = new fricCompRes_visu(node);

    comprehensive_msg_pub = new Comprehensive_visu(node);
}

performanceTestSeal::~performanceTestSeal() {
    // 清理资源
    delete current_track;
    delete pos_vel_track;
    delete torComp_track;
    delete k_comp_visu;
    delete fric_comp_visu;

    delete comprehensive_msg_pub;
}

//-------------------------------------------------------------------------------------

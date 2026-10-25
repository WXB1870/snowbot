#include"controller/xrTelepercationCom.h"
#include"robot_control/config.h"

std::vector<double> dbg_reserve(7, 0.0);

//------------------------------------------------------------
//辅助类：插补器
//点位插补
TrajResult stTraj_runiner::dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase)
{
    //轨迹一半的时间用来加速
    //由曲线推到可得，当加速过程时间占到时间的一半时，按照钟型曲线加速过程和匀速过程走过距离比例为1：2

    float t_half = mTrajTime / 2;
    float posDes_half = mposDes / 2;

    //计算半切轨迹及其增量
    std::vector<float> acc_traj;
    std::vector<float> acc_vel;

    //轨迹参数计算a, b, 和v_max
    if (accTimeRateBase <= 1.0) {
        std::cerr << "accTimeRateBase must be greater than 0.5 to avoid division by zero" << std::endl;
        accTimeRateBase = 1.05; // 设置默认安全值
    }
    float t_acc = t_half / accTimeRateBase;
    float t_stab_half = t_half - t_acc;
    float denominator = std::pow(t_half, 4) * (2*accTimeRateBase - 1);
    float a = -12 * std::pow(accTimeRateBase, 4) * posDes_half / denominator;
    float b = a * t_half / accTimeRateBase;
    //float v_max = (1.0/3.0)*a*std::pow(t_acc, 3) - (1.0/2.0)*b*std::pow(t_acc,2);

    float acc_moveLen = (1.0/12)*a*std::pow(t_acc, 4) - (1.0/6)*b*std::pow(t_acc, 3);
    //计算加减速段轨迹
    float temp_pos_lst = 0;
    float temp_T = 0;
    while (true){
        float temp_pos;
        if(temp_T < t_acc || temp_T == t_acc){
            temp_pos = (1.0/12)*a*std::pow(temp_T, 4) - (1.0/6)*b*std::pow(temp_T, 3);

            acc_traj.push_back(temp_pos);
            acc_vel.push_back((temp_pos - temp_pos_lst)/TempTime);

            temp_pos_lst = temp_pos;
            temp_T = temp_T + TempTime;
        }else{
            break;
        }
    }

    //计算匀速阶段轨迹 -- 修改以补偿
    float stabVel_trajLen = mposDes - 2*acc_traj.back();
    float stabVel_time = mTrajTime - 2*(temp_T - TempTime);
    int stabVelTrmpTimes = std::round(stabVel_time/TempTime);
    float stabVel_derPos = stabVel_trajLen / (stabVelTrmpTimes);
    float stabVel = stabVel_trajLen / (stabVelTrmpTimes * TempTime);

    //据此计算整个轨迹
    std::vector<float> whole_traj;
    std::vector<float> whole_traj_vel;

    //输入加速段轨迹
    for(int i = 0; i < acc_traj.size(); i++){
        whole_traj.push_back(acc_traj[i]);
        whole_traj_vel.push_back(acc_vel[i]);
    }

    //输入匀速段轨迹
    float temp_pos_2 = acc_traj.back();
    for(int i = 0; i < stabVelTrmpTimes; i++){
        temp_pos_2 += stabVel_derPos;
        whole_traj.push_back(temp_pos_2);
        whole_traj_vel.push_back(stabVel);
    }

    //输入减速段轨迹
    for(int i = 0; i < acc_traj.size(); i++){
        float element = *std::next(acc_traj.rbegin(), i);
        float element_vel = *std::next(acc_vel.rbegin(), i);
        float m_temp_pos = mposDes - element;

        whole_traj.push_back(m_temp_pos);
        whole_traj_vel.push_back(element_vel);
    }

    TrajResult mRes;
    mRes.positionIncrements = whole_traj;
    mRes.VelocityTraj = whole_traj_vel;

    //获取最后一轮的位置和速度
    lastTempPosOutp = mposDes;
    lastTempVelOutp = 0.0f;

    return mRes;
}

std::pair<float, float> stTraj_runiner::lastPosVelGet(float posSt)
{
    float posOutpCor = this->lastTempPosOutp + posSt;
    std::pair<float, float> mRes = {posOutpCor, this->lastTempVelOutp};
    return mRes;
}

//------------------------------------------------------------

void kalman_filter_v::mMatrixMul(double a[],double b[],int m,int n,int k,double c[])
{ int i,j,l,u;
    for (i=0; i<=m-1; i++){
        for (j=0; j<=k-1; j++)
        { u=i*k+j; c[u]=0.0;
            for (l=0; l<=n-1; l++)
            c[u]=c[u]+a[i*n+l]*b[l*k+j];
        }
    }
}

void kalman_filter_v::FilterInit(double inputVar, double mQgain, double mRgain)
{
    //****** debug
    for(int i=0;i<2;i++){
        for(int j=0;j<2;j++)
        {
            Pmtrx[i][j]=P0mtrx[i][j];
        }
    }

    *outvar = inputVar;
    *(outvar+1) = 0;
    this->Qgain = mQgain;
    this->Rgain = mRgain;
}

double kalman_filter_v::FilteredResCal(double inputvar)
{
    // 预测阶段
    mMatrixMul(FAImtrx[0], outvar, 2, 2, 1, midx); // 这里假设MatrixMul能正确处理这些尺寸
    mMatrixMul(FAImtrx[0], Pmtrx[0], 2, 2, 2, FAIPFAIT[0]); // 预测协方差矩阵
    // 加上过程噪声
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            FAIPFAIT[i][j] += Qgain * Qgain * Qmtrx[i][j];
        }
    }
    // 计算卡尔曼增益
    float K = FAIPFAIT[0][0] / (FAIPFAIT[0][0] + Rgain * Rgain);

    // 更新状态
    outvar[0] = midx[0] + K * (inputvar - midx[0]); // 对于一维状态，只更新第一个元素

    // 更新协方差矩阵（对于一维状态，只更新相关的元素）
    double I_minus_KH[2][2] = {{1.0f - K, 0.0f}, {-K, 1.0f}}; // 实际上，对于一维情况，这简化为[1-K, 0; -K, 1]（但保留2x2形式以备将来扩展）
    mMatrixMul(I_minus_KH[0], FAIPFAIT[0], 2, 2, 2, Pmtrx[0]); // 更新后的协方差矩阵

    // 返回结果
    return outvar[0];
}

//--------------------------------------------------
//通讯核心类：

std::vector<double> quaternionToEuler(const std::vector<double>& quaternion);
void xr_telepercation_ctl::init(){

    l_arm_end_pos.assign(3, 0.0);
    r_arm_end_pos.assign(3, 0.0);
    l_arm_end_delta.assign(3, 0.0);
    r_arm_end_delta.assign(3, 0.0);
    lArm_end_pos_rpy.assign(6, 0.0);
    rArm_end_pos_rpy.assign(6, 0.0);

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //初始信息包装
    //关节空间：
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间：
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg.desTor_l_JointSpace.assign(SingleArmDof, 0.0);
    ctlStMsg.desTor_r_JointSpace.assign(SingleArmDof, 0.0);

    //-------------------------
    //计算初始轨迹
    l_trajRuniner.resize(6);
    r_trajRuniner.resize(6);
    l_stDerPosTraj.resize(6);
    r_stDerPosTraj.resize(6);
    for(int i = 0; i < 6; i++){
        float der_pos_l = lArm_end_pos_rpy_st[i] - lArm_stDescPos[i];
        float der_pos_r = rArm_end_pos_rpy_st[i] - rArm_stDescPos[i];

        l_stDerPosTraj[i] = l_trajRuniner[i].dPos_queue_stabTime_dot(der_pos_l, this->trajTime, this->accRateBase);
        r_stDerPosTraj[i] = r_trajRuniner[i].dPos_queue_stabTime_dot(der_pos_r, this->trajTime, this->accRateBase);
    }

    outpTimes = 0;
    maxLen = l_stDerPosTraj[0].positionIncrements.size();
    for(int i = 0; i < 6; i++){
        if(maxLen < l_stDerPosTraj[i].positionIncrements.size()){
            maxLen = l_stDerPosTraj[i].positionIncrements.size();
        }

        if(maxLen < r_stDerPosTraj[i].positionIncrements.size()){
            maxLen = r_stDerPosTraj[i].positionIncrements.size();
        }
    }

    //-------------------------

    //控制值初始化
    //-----数据传输与插补模式-----
    mdataSend.ctl_transMod = 4;
    mdataSend.runinMod = 1;

    //-----求解模式信息-----
    mdataSend.ifUseRet = 0;
    mdataSend.ifUseTemp = 0;

    for(int i = 0; i < SingleArmDof; i++){
        mdataSend.rigidityCtl_mod_l[i] = this->rigidity_mod;
        mdataSend.rigidityCtl_mod_r[i] = this->rigidity_mod;
    }

    mdataSend.KpTrans_l.resize(SingleArmDof);
    mdataSend.KdTrans_l.resize(SingleArmDof);
    mdataSend.KpTrans_r.resize(SingleArmDof);
    mdataSend.KdTrans_r.resize(SingleArmDof);

    //-----重力补偿控制-----
    mdataSend.GravCompCtlMsg.ifUseGravComp = 1;
    mdataSend.GravCompCtlMsg.mIfUseFricComp = 0;
    mdataSend.GravCompCtlMsg.mIfUseInpFtr = 1;
    mdataSend.GravCompCtlMsg.mIfUseOutpFtr = 0;
    mdataSend.GravCompCtlMsg.GravCompInpMod = 1;

    mdataSend.stJntID_lArm = 0;
    mdataSend.stJntID_rArm = 7;
    mdataSend.urdf_choose = 2;

    //-----插补信息包装-----
    //发送变量初始化
    double trajTime = 1.0;
    double accRate = 1.2;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    mdataSend.TrajTime.resize(2);
    mdataSend.TrajTime[0].assign(SingleArmDof, trajTime);
    mdataSend.TrajTime[1].assign(SingleArmDof, trajTime);

    mdataSend.accRateBase.resize(2);
    mdataSend.accRateBase[0].assign(SingleArmDof, accRate);
    mdataSend.accRateBase[1].assign(SingleArmDof, accRate);

    mdataSend.m_stabMsg.resize(2);
    mdataSend.m_stabMsg[0].assign(SingleArmDof, stabMsg);
    mdataSend.m_stabMsg[1].assign(SingleArmDof, stabMsg);

    //-----开关量包装-----
    mdataSend.ifJntStab.resize(2);
    mdataSend.ifJntStab[0] = lArm_ifJntStab;
    mdataSend.ifJntStab[1] = rArm_ifJntStab;

    mdataSend.ifRedefineJntTrajStab = false;

    //-----关节限位信息-----
    // 锁定身体/头部/夹爪关节 (仅解算手臂J1~J7)
    mdataSend.lArm_JntLim.ifUseJntLim = true;
    mdataSend.lArm_JntLim.limJntNum = 5;
    mdataSend.lArm_JntLim.limJntLabel.resize(mdataSend.lArm_JntLim.limJntNum);
    mdataSend.lArm_JntLim.limJntAng.resize(mdataSend.lArm_JntLim.limJntNum);
    mdataSend.lArm_JntLim.limJntLabel[0] = "body_joint_1";
    mdataSend.lArm_JntLim.limJntAng[0] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[1] = "body_joint_2";
    mdataSend.lArm_JntLim.limJntAng[1] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[2] = "head_joint_1";
    mdataSend.lArm_JntLim.limJntAng[2] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[3] = "head_joint_2";
    mdataSend.lArm_JntLim.limJntAng[3] = 0.0;
    mdataSend.lArm_JntLim.limJntLabel[4] = "arm_l_joint_8";
    mdataSend.lArm_JntLim.limJntAng[4] = 0.0;

    mdataSend.rArm_JntLim.ifUseJntLim = true;
    mdataSend.rArm_JntLim.limJntNum = 1;
    mdataSend.rArm_JntLim.limJntAng.resize(mdataSend.rArm_JntLim.limJntNum);
    mdataSend.rArm_JntLim.limJntLabel.resize(mdataSend.rArm_JntLim.limJntNum);
    mdataSend.rArm_JntLim.limJntLabel[0] = "arm_r_joint_8";
    mdataSend.rArm_JntLim.limJntAng[0] = 0.0;

    mdataSend.ifUpdateLim = false;

    //-----关节初始化信息-----
    mdataSend.ifReInit = false;
    mdataSend.lArm_stPos = lArm_stJntPos;
    mdataSend.rArm_stPos = rArm_stJntPos;

    mdataSend.ifDispJntInfo = false;
    mdataSend.ifUse_visualization = false;
    mdataSend.ifDisp_msgs = false;

    //求解器参数初始化

    mdataSend.optiWeight[0] = 50.0;
    mdataSend.optiWeight[1] = 1.0;
    mdataSend.optiWeight[2] = 0.002;
    mdataSend.optiWeight[3] = 0.01;
    mdataSend.optiWeight[4] = 0.00001;

    mdataSend.template_times = 50;
    mdataSend.errTol = 1e-6;

    mdataSend.ifPrintOptiLevel = 0;
    mdataSend.ifPrintOptiTime = false;

    mdataSend.ctlDesMsg = ctlStMsg;
}

void xr_telepercation_ctl::trans_and_outp_core(){
    // 数据转化 - 将ZYX顺序的欧拉角转换为四元数
    std::vector<double> lArm_quat_ori = eulerToQuaternionZYX(l_arm_end_pos);
    std::vector<double> rArm_quat_ori = eulerToQuaternionZYX(r_arm_end_pos);
    
    std::vector<double> base_quat_l = eulerToQuaternionZYX(gesture_trans_l);
    std::vector<double> base_quat_r = eulerToQuaternionZYX(gesture_trans_r);
    
    // 使用四元数乘法进行姿态组合
    std::vector<double> lArm_quat_world = multiplyQuaternions(base_quat_l, lArm_quat_ori);
    std::vector<double> rArm_quat_world = multiplyQuaternions(base_quat_r, rArm_quat_ori);
    
    // 将四元数转换回ZYX顺序的欧拉角
    std::vector<double> lArm_rpy_world = quaternionToEulerZYX(lArm_quat_world);
    std::vector<double> rArm_rpy_world = quaternionToEulerZYX(rArm_quat_world);
    
    // 数据点包装
    for(int i = 0; i < 3; i++){
        lArm_end_pos_rpy[i] = l_arm_end_delta[i] + pos_trans_l[i];
        rArm_end_pos_rpy[i] = r_arm_end_delta[i] + pos_trans_r[i];
        
        lArm_end_pos_rpy[i+3] = lArm_rpy_world[i];
        rArm_end_pos_rpy[i+3] = rArm_rpy_world[i];
    }
    
    ctlDesMsg = ctlStMsg;
    ctlDesMsg.desPos_l_Descartes = lArm_end_pos_rpy;
    ctlDesMsg.desPos_r_Descartes = rArm_end_pos_rpy;
    mdataSend.ctlDesMsg = ctlDesMsg;
}


//--------------位姿初始化部分--------------
//输出初始阶段轨迹的函数
void xr_telepercation_ctl::stPosTrajOutp(){

    //轨迹输出量定义
    stPos_ctlDesMsg = ctlStMsg;
    for(int i = 0; i < 6; i++){
        if(outpTimes < l_stDerPosTraj[i].positionIncrements.size()){
            stPos_ctlDesMsg.desPos_l_Descartes[i] = stPos_ctlDesMsg.desPos_l_Descartes[i] + l_stDerPosTraj[i].positionIncrements[outpTimes];
        }else{
            stPos_ctlDesMsg.desPos_l_Descartes[i] = lArm_end_pos_rpy_st[i];
        }

        if(outpTimes < r_stDerPosTraj[i].positionIncrements.size()){
            stPos_ctlDesMsg.desPos_r_Descartes[i] = stPos_ctlDesMsg.desPos_r_Descartes[i] + r_stDerPosTraj[i].positionIncrements[outpTimes];
        }else{
            stPos_ctlDesMsg.desPos_r_Descartes[i] = rArm_end_pos_rpy_st[i];
        }
    }

    outpTimes += 1;
    if(outpTimes >= (maxLen + 20)){
        ifReachXrStPos = true;
    }else{
        ifReachXrStPos = false;
    }

    //控制参数获取并修改
    stPosReach_dataSend = mdataSend;
    stPosReach_dataSend.ctlDesMsg = stPos_ctlDesMsg;
}

dataSendWrap xr_telepercation_ctl::trans_and_outp(){

    if(!ifReachXrStPos){
        stPosTrajOutp();
        return stPosReach_dataSend;
    }else{
        if(ifFirstOutp){
            std::cout << "准备开始遥操作" << std::endl;
            ifFirstOutp = false;
            for(int i = 0; i < l_arm_end_pos.size(); i++){
                l_arm_end_pos[i] = 0.0;
            }
            for(int i = 0; i < l_arm_end_delta.size(); i++){
                l_arm_end_delta[i] = 0.0;
            }

            for(int i = 0; i < r_arm_end_pos.size(); i++){
                r_arm_end_pos[i] = 0.0;
            }
            for(int i = 0; i < r_arm_end_delta.size(); i++){
                r_arm_end_delta[i] = 0.0;
            }
            l_arm_end_pos[3] = 1.0;
            r_arm_end_pos[3] = 1.0;
            ifUseStFtr = true;
            stabtimecount = 0;
        }
        
        trans_and_outp_core();

        return mdataSend;
    }
}

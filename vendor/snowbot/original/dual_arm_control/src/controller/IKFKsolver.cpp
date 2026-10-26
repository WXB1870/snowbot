#include "controller/IKFKsolver.h"



//辅助函数声明：
//====================================================================================================

std::string path_options[3] = {
    "left_arm.urdf",
    "right_arm.urdf",
    "snowbot.urdf"
};
std::string base_link[3] = {
    "body_link_2",
    "body_link_2",
    "body_link_2"
};
std::string tip_link[3] = {
    "arm_l_link_8",
    "arm_r_link_8",
    "arm_l_link_8"
};
//--------------------求解器接口封装类--------------------

//--------------------封装辅助函数--------------------

//--------------------接口函数--------------------
void IkFk_solverPino::Init(PathChoose mChoose_lArm, PathChoose mChoose_rArm, 
                           JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,
                           std::vector<double> JntAngConf_l, std::vector<double> JntAngConf_r,
                           optiPrintMsgCtl mPrintMsgPkg,
                           optimizerCtl mOptiCtl
){
    //初始化函数

    //----------信息重新包装---------
    //函数初始化信息重新包装
    std::string urdf_path = path_options[mChoose_lArm.urdf_choose];
    std::vector<double> initial_q = JntAngConf_l;
    initial_q.insert(initial_q.end(), JntAngConf_r.begin(), JntAngConf_r.end());
    std::vector<std::string> arm_end_labels = {tip_link[mChoose_lArm.tip_choose], tip_link[mChoose_rArm.tip_choose]};
    bool use_joint_limits = lArm_LimJnt.ifUseJntLim || rArm_LimJnt.ifUseJntLim;
    std::vector<std::string> limited_joints = lArm_LimJnt.limJntLabel;
    limited_joints.insert(limited_joints.end(), rArm_LimJnt.limJntLabel.begin(), rArm_LimJnt.limJntLabel.end());
    std::vector<double> limited_joint_angle = lArm_LimJnt.limJntAng;
    limited_joint_angle.insert(limited_joint_angle.end(), rArm_LimJnt.limJntAng.begin(), rArm_LimJnt.limJntAng.end());

    //程序打印控制参数重新包装
    bool ifDispJntInfo = mPrintMsgPkg.dispJntInfo;
    bool ifUseVisu = mPrintMsgPkg.use_visualization;
    bool ifDispMsgs = mPrintMsgPkg.disp_messages;
    this->ifDispJntMsg = ifDispMsgs;

    //优化器控制类信息重新包装
    std::vector<double> OptiWeightVec;
    for(int i = 0; i < 5; i++){
        OptiWeightVec.push_back(mOptiCtl.OptiWeight[i]);
    }
    int tempLate_time = mOptiCtl.template_times;
    double m_errTol = mOptiCtl.errTol;
    int m_ifPrintLevel = mOptiCtl.ifPrintLevel;
    bool m_ifPrintOptiTime = mOptiCtl.ifPrintOptiTime;

    //----------求解器python类初始化----------
    pyDualArmSolverWrapper.initialize(
        urdf_path,
        initial_q,
        arm_end_labels,
        use_joint_limits,
        limited_joints,
        limited_joint_angle,

        ifDispJntInfo,
        ifUseVisu,
        ifDispMsgs,

        OptiWeightVec,
        tempLate_time,
        m_errTol,
        m_ifPrintLevel,
        m_ifPrintOptiTime
    ); 
    
    //----------关键参数初始化----------
    this->lastTempSolve_Jnt_q = initial_q;
}

//权重更新
/*  输入量：
    update_mode -- 更新模式：1——采用初始化的默认权重；2——采用自定义权重
    optiWeight_selfDefine -- 自定义权重
*/
void IkFk_solverPino::PinoSolver_optiWeightUpdate(int update_mode, std::vector<double> optiWeight_selfDefine, bool ifUseFtr){
    pyDualArmSolverWrapper.optiWeight_def(update_mode, optiWeight_selfDefine, ifUseFtr);
}

std::vector<KDL::Frame> IkFk_solverPino::Fk_pinoSolver(KDL::JntArray lArm_JntArray, KDL::JntArray rArm_JntArray)
{
    //正解求解器
    std::pair<KDL::Frame, KDL::Frame> DualArmEndFrame = pyDualArmSolverWrapper.solveFK(lArm_JntArray, rArm_JntArray);
    std::vector<KDL::Frame> mRes;
    mRes.push_back(DualArmEndFrame.first);
    mRes.push_back(DualArmEndFrame.second);

    return mRes;
}

std::vector<KDL::JntArray> IkFk_solverPino::IK_pinoSolver(
    KDL::Frame lArm_DesPos, KDL::Frame rArm_DesPos,
    KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
    int solveMod
){
    //逆解求解器
    std::pair<KDL::JntArray, KDL::JntArray> mRes;

    if(solveMod == 1){
        std::vector<double> return_q;
        for(int i = 0; i < lArm_posRet.rows(); i++){
            return_q.push_back(lArm_posRet(i));
        }
        for(int i = 0; i < rArm_posRet.rows(); i++){
            return_q.push_back(rArm_posRet(i));
        }

        mRes = pyDualArmSolverWrapper.solveIK(lArm_DesPos, rArm_DesPos, return_q);
    }else{
        mRes = pyDualArmSolverWrapper.solveIK(lArm_DesPos, rArm_DesPos);
    }

    std::vector<KDL::JntArray> mRes_vec;
    mRes_vec.push_back(mRes.first);
    mRes_vec.push_back(mRes.second);

    return mRes_vec;
}

void IkFk_solverPino::solverUpdate(JntLimMsg mLimMsg){
    //求解器更新函数封装
    pyDualArmSolverWrapper.updateJointLimits(
        mLimMsg.ifUseJntLim,
        mLimMsg.limJntLabel,
        mLimMsg.limJntAng,
        this->ifDispJntMsg
    );
}

//--------------------辅助函数封装--------------------

// 独立函数：打印关节信息
void printJointInfo(const pinocchio::Model& model) {
    std::cout << "=== Joint Information ===" << std::endl;
    std::cout << "Number of joints: " << model.njoints << std::endl;
    std::cout << "Joint names:" << std::endl;

    for (pinocchio::JointIndex joint_id = 0; joint_id < model.njoints; ++joint_id) {
        std::cout << "  Joint " << joint_id << ": " << model.names[joint_id];
        if (model.parents[joint_id] == 0) {
            std::cout << " (root joint)";
        }
        std::cout << std::endl;
    }
}

//打印KDL::Frame类型的数据的函数
void printKDLFrameAsMatrix(const KDL::Frame& frame) {
    // 获取旋转矩阵 R (3x3) 和平移向量 p (3x1)
    const KDL::Rotation& R = frame.M;
    const KDL::Vector& p = frame.p;

    // 打印 4x4 齐次变换矩阵
    std::cout << "KDL::Frame as 4x4 Matrix:" << std::endl;
    std::cout << std::fixed << std::setprecision(4);  // 固定小数格式，保留4位

    // 打印旋转矩阵部分 (3x3)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(10) << R(i, j) << " ";
        }
        // 打印平移向量部分 (1x3)
        std::cout << " | " << std::setw(10) << p[i] << std::endl;
    }

    // 打印最后一行 [0 0 0 1]
    std::cout << std::setw(10) << 0.0 << " "
              << std::setw(10) << 0.0 << " "
              << std::setw(10) << 0.0 << " | "
              << std::setw(10) << 1.0 << std::endl;
}

#include"controller/ArmInterface.h"

//==================================================

/*基础操作封装*/

//重力补偿封装

void DualArmGravWrap::init_GravComp(
    //补偿控制参量：
    int mIfUseInpFtr, int mIfUseOutpFtr,

    //重力补偿参量
    KDL::JntArray lArm_stPos, KDL::JntArray rArm_stPos, 
    //是否使用集成stribeck摩擦补偿参量
    int mIfUseFricComp,
    
    int lArm_stJnt, int rArm_stJnt, int urdf_choose
){
    //控制参量刷新：
    this->ifUseInpFtr = mIfUseInpFtr;
    this->ifUseOutpFtr = mIfUseOutpFtr;
    this->ifUseFricComp = mIfUseFricComp;

    //重力补偿器初始化
    mForceComp.init(lArm_stPos, lArm_stJnt, rArm_stPos, rArm_stJnt, urdf_choose);
    mForceComp.FricCompInit();

    //滤波器初始化
    for(int i = 0; i < SINGLEARMDOF; i++){
        posInp_lowpassFilter_lArm[i].init(all_sampRate_posInp_lArm[i], all_cutoffFreq_posInp_lArm[i]);
        velInp_lowpassFilter_lArm[i].init(all_sampRate_velInp_lArm[i], all_cutoffFreq_velInp_lArm[i]);
        TorInp_lowpassFilter_lArm[i].init(all_sampRate_TorInp_lArm[i], all_cutoffFreq_TorInp_lArm[i]);

        grav_TorOutp_lowpassFilter_lArm[i].init(all_sampRate_TorOutp_lArm[i], all_cutoffFreq_TorOutp_lArm[i]);
        Inte_TorOutp_lowpassFilter_lArm[i].init(all_sampRate_TorOutp_lArm[i], all_cutoffFreq_TorOutp_lArm[i]);
        feedforward_TorComp_lowpassFilter_lArm[i].init(all_sampRate_TorOutp_lArm[i], all_cutoffFreq_TorOutp_lArm[i]);
    
        posInp_lowpassFilter_rArm[i].init(all_sampRate_posInp_rArm[i], all_cutoffFreq_posInp_rArm[i]);
        velInp_lowpassFilter_rArm[i].init(all_sampRate_velInp_rArm[i], all_cutoffFreq_velInp_rArm[i]);
        TorInp_lowpassFilter_rArm[i].init(all_sampRate_TorInp_rArm[i], all_cutoffFreq_TorInp_rArm[i]);

        grav_TorOutp_lowpassFilter_rArm[i].init(all_sampRate_TorOutp_rArm[i], all_cutoffFreq_TorOutp_rArm[i]);
        Inte_TorOutp_lowpassFilter_rArm[i].init(all_sampRate_TorOutp_rArm[i], all_cutoffFreq_TorOutp_rArm[i]);
        feedforward_TorComp_lowpassFilter_rArm[i].init(all_sampRate_TorOutp_rArm[i], all_cutoffFreq_TorOutp_rArm[i]);
    }
}

TorCompWrap DualArmGravWrap::GravCompOutp(
    //输入量
    std::vector<double> lArm_pos, std::vector<double> lArm_vel, std::vector<double> lArm_Tor,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel, std::vector<double> rArm_Tor
){  
    std::vector<double> torComp_grav;           //重力补偿输出
    std::vector<double> torComp_InteriaComb;    //惯性补偿输出
    std::vector<double> torFeedForward;         //力前馈输出，这里仅仅包含重力补偿模型中自带的摩擦模型

    std::vector<double> fricComp;       //接受辅助参数 -- 用于接受摩擦补偿输出

    if(ifUseInpFtr == 1){
        //对输入进行滤波处理
        std::vector<double> lArm_posRetFtred;
        std::vector<double> lArm_velRetFtred;
        std::vector<double> lArm_TorRetFtred;

        std::vector<double> rArm_posRetFtred;
        std::vector<double> rArm_velRetFtred;
        std::vector<double> rArm_TorRetFtred;

        for(int i = 0; i < SINGLEARMDOF; i++){

            lArm_posRetFtred.push_back(posInp_lowpassFilter_lArm[i].filter(lArm_pos.at(i)));
            lArm_velRetFtred.push_back(velInp_lowpassFilter_lArm[i].filter(lArm_vel.at(i)));
            lArm_TorRetFtred.push_back(TorInp_lowpassFilter_lArm[i].filter(lArm_Tor.at(i)));

            rArm_posRetFtred.push_back(posInp_lowpassFilter_rArm[i].filter(rArm_pos.at(i)));
            rArm_velRetFtred.push_back(velInp_lowpassFilter_rArm[i].filter(rArm_vel.at(i)));
            rArm_TorRetFtred.push_back(TorInp_lowpassFilter_rArm[i].filter(rArm_Tor.at(i)));

        }
        torComp_grav = mForceComp.gravComp(lArm_posRetFtred, lArm_velRetFtred,
                                        rArm_posRetFtred, rArm_velRetFtred);
        torComp_InteriaComb = mForceComp.InteriaMulComp(
            lArm_posRetFtred, lArm_velRetFtred,
            rArm_posRetFtred, rArm_velRetFtred
        );

        //对摩擦补偿进行处理
        fricComp.resize(torComp_grav.size());
        if(this->ifUseFricComp){
            //使用摩擦补偿
            fricComp = mForceComp.FricComp(
                lArm_posRetFtred, lArm_velRetFtred, lArm_TorRetFtred,
                rArm_posRetFtred, rArm_velRetFtred, rArm_TorRetFtred
            );
        }else{
            //不使用摩擦补偿：
            for(int j = 0; j < fricComp.size(); j++){
                fricComp[j] = 0.0;
            }
        }

        //前馈量赋值
        torFeedForward = fricComp;

        reserve[0] = lArm_posRetFtred.at(1);
        reserve[1] = rArm_posRetFtred.at(1);

    }else{

        //重力补偿赋值
        torComp_grav = mForceComp.gravComp(lArm_pos, lArm_vel,
                                    rArm_pos, rArm_vel);

        //惯性综合补偿赋值
        torComp_InteriaComb = mForceComp.InteriaMulComp(
            lArm_pos, lArm_vel,
            rArm_pos, rArm_vel
        );

        //摩擦补偿计算
        fricComp.resize(torComp_grav.size());
        if(this->ifUseFricComp){
            //使用摩擦补偿
            fricComp = mForceComp.FricComp(
                lArm_pos, lArm_vel, lArm_Tor,
                rArm_pos, rArm_vel, rArm_Tor
            );
        }else{
            //不使用摩擦补偿：
            for(size_t j = 0; j < fricComp.size(); j++){
                fricComp[j] = 0.0;
            } 
        }

        //摩擦补偿输出
        torFeedForward = fricComp;
    }

    //对输出进行滤波处理
    std::vector<double> grav_TorComp_Ftred;         //重力补偿（滤波后）
    std::vector<double> Intertia_TorComp_Ftred;     //惯性补偿（滤波后）
    std::vector<double> FeedForward_TorComp_Ftred;  //摩擦前馈补偿（滤波后）


    if(ifUseOutpFtr == 1){
        int m_temp = 0;
        for(int i = 0; i < SINGLEARMDOF; i++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_lArm[i].filter(torComp_grav.at(m_temp)));
            Intertia_TorComp_Ftred.push_back(Inte_TorOutp_lowpassFilter_lArm[i].filter(torComp_InteriaComb.at(m_temp)));
            FeedForward_TorComp_Ftred.push_back(feedforward_TorComp_lowpassFilter_lArm[i].filter(torFeedForward.at(m_temp)));
            m_temp += 1;
        }
        for(int j = 0; j < SINGLEARMDOF; j++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_rArm[j].filter(torComp_grav.at(m_temp)));
            Intertia_TorComp_Ftred.push_back(Inte_TorOutp_lowpassFilter_rArm[j].filter(torComp_InteriaComb.at(m_temp)));
            FeedForward_TorComp_Ftred.push_back(feedforward_TorComp_lowpassFilter_rArm[j].filter(torFeedForward.at(m_temp)));
            m_temp += 1;
        }
    }else{
        grav_TorComp_Ftred = torComp_grav;
        Intertia_TorComp_Ftred = torComp_InteriaComb;
        FeedForward_TorComp_Ftred = torFeedForward;
    }

    //结果输出包装
    TorCompWrap mRes;
    mRes.lArm_Comp_grav.assign(SINGLEARMDOF, 0.0);
    mRes.rArm_comp_grav.assign(SINGLEARMDOF, 0.0);

    mRes.lArm_Comp_InteriaComb.assign(SINGLEARMDOF, 0.0);
    mRes.rArm_Comp_InteriaComb.assign(SINGLEARMDOF, 0.0);

    mRes.lArm_feedForward.assign(SINGLEARMDOF, 0.0);
    mRes.rArm_feedForward.assign(SINGLEARMDOF, 0.0);

    for(int i = 0; i < SINGLEARMDOF; i++){
        mRes.lArm_Comp_grav[i] = grav_TorComp_Ftred[i];
        mRes.rArm_comp_grav[i] = grav_TorComp_Ftred[i + SINGLEARMDOF];

        mRes.lArm_Comp_InteriaComb[i] = Intertia_TorComp_Ftred[i];
        mRes.rArm_Comp_InteriaComb[i] = Intertia_TorComp_Ftred[i + SINGLEARMDOF];

        mRes.lArm_feedForward[i] = FeedForward_TorComp_Ftred[i];
        mRes.rArm_feedForward[i] = FeedForward_TorComp_Ftred[i + SINGLEARMDOF];
    }

    return mRes;
}

//纯粹的重力补偿函数——输出中仅包含重力补偿
TorCompWrap DualArmGravWrap::onlyGravComp(
    //输入量：
    std::vector<double> lArm_pos, std::vector<double> lArm_vel,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel
){
    std::vector<double> torComp_grav;           //重力补偿输出

    if(ifUseInpFtr == 1){
        //对输入进行滤波处理
        std::vector<double> lArm_posRetFtred;
        std::vector<double> lArm_velRetFtred;

        std::vector<double> rArm_posRetFtred;
        std::vector<double> rArm_velRetFtred;

        for(int i = 0; i < SINGLEARMDOF; i++){

            lArm_posRetFtred.push_back(posInp_lowpassFilter_lArm[i].filter(lArm_pos.at(i)));
            lArm_velRetFtred.push_back(velInp_lowpassFilter_lArm[i].filter(lArm_vel.at(i)));

            rArm_posRetFtred.push_back(posInp_lowpassFilter_rArm[i].filter(rArm_pos.at(i)));
            rArm_velRetFtred.push_back(velInp_lowpassFilter_rArm[i].filter(rArm_vel.at(i)));
        }

        torComp_grav = mForceComp.gravComp(lArm_posRetFtred, lArm_velRetFtred,
                                        rArm_posRetFtred, rArm_velRetFtred);


        reserve[0] = lArm_posRetFtred.at(1);
        reserve[1] = rArm_posRetFtred.at(1);

    }else{

        //重力补偿赋值
        torComp_grav = mForceComp.gravComp(lArm_pos, lArm_vel,
                                    rArm_pos, rArm_vel);

    }

    //对输出进行滤波处理
    std::vector<double> grav_TorComp_Ftred;         //重力补偿（滤波后）


    if(ifUseOutpFtr == 1){
        int m_temp = 0;
        for(int i = 0; i < SINGLEARMDOF; i++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_lArm[i].filter(torComp_grav.at(m_temp)));
            m_temp += 1;
        }
        for(int j = 0; j < SINGLEARMDOF; j++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_rArm[j].filter(torComp_grav.at(m_temp)));
            m_temp += 1;
        }
    }else{
        grav_TorComp_Ftred = torComp_grav;
    }

    //结果输出包装
    TorCompWrap mRes;

    mRes.lArm_Comp_grav.assign(SINGLEARMDOF, 0.0);
    mRes.rArm_comp_grav.assign(SINGLEARMDOF, 0.0);

    mRes.lArm_Comp_InteriaComb.assign(SINGLEARMDOF, 0.0);
    mRes.rArm_Comp_InteriaComb.assign(SINGLEARMDOF, 0.0);

    mRes.lArm_feedForward.assign(SINGLEARMDOF, 0.0);
    mRes.rArm_feedForward.assign(SINGLEARMDOF, 0.0);

    for(int i = 0; i < SINGLEARMDOF; i++){
        mRes.lArm_Comp_grav[i] = grav_TorComp_Ftred[i];
        mRes.rArm_comp_grav[i] = grav_TorComp_Ftred[i + SINGLEARMDOF];
    }

    return mRes;
}

//PD参数初始化 -- 从.yaml配置文件读取PD参数
void PDInte::PDInteInit(){
    // 调整所有PD数组大小
    Kp_lArm_low.resize(SINGLEARMDOF);
    Kd_lArm_low.resize(SINGLEARMDOF);
    Kp_rArm_low.resize(SINGLEARMDOF);
    Kd_rArm_low.resize(SINGLEARMDOF);

    Kp_lArm_mid.resize(SINGLEARMDOF);
    Kd_lArm_mid.resize(SINGLEARMDOF);
    Kp_rArm_mid.resize(SINGLEARMDOF);
    Kd_rArm_mid.resize(SINGLEARMDOF);

    Kp_lArm_high.resize(SINGLEARMDOF);
    Kd_lArm_high.resize(SINGLEARMDOF);
    Kp_rArm_high.resize(SINGLEARMDOF);
    Kd_rArm_high.resize(SINGLEARMDOF);

    Kp_lArm_superhigh.resize(SINGLEARMDOF);
    Kd_lArm_superhigh.resize(SINGLEARMDOF);
    Kp_rArm_superhigh.resize(SINGLEARMDOF);
    Kd_rArm_superhigh.resize(SINGLEARMDOF);

    Kp_lArm_tag.resize(SINGLEARMDOF);
    Kd_lArm_tag.resize(SINGLEARMDOF);
    Kp_rArm_tag.resize(SINGLEARMDOF);
    Kd_rArm_tag.resize(SINGLEARMDOF);

    // 从YAML全局变量拷贝PD参数 (double → float)
    for(int i = 0; i < SINGLEARMDOF; i++){
        // 低刚度 (low)
        Kp_lArm_low[i] = static_cast<float>(lArm_defaultKp_low[i]);
        Kd_lArm_low[i] = static_cast<float>(lArm_defaultKd_low[i]);
        Kp_rArm_low[i] = static_cast<float>(rArm_defaultKp_low[i]);
        Kd_rArm_low[i] = static_cast<float>(rArm_defaultKd_low[i]);

        // 中等刚度 (mid)
        Kp_lArm_mid[i] = static_cast<float>(lArm_defaultKp_mid[i]);
        Kd_lArm_mid[i] = static_cast<float>(lArm_defaultKd_mid[i]);
        Kp_rArm_mid[i] = static_cast<float>(rArm_defaultKp_mid[i]);
        Kd_rArm_mid[i] = static_cast<float>(rArm_defaultKd_mid[i]);

        // 高刚度 (high)
        Kp_lArm_high[i] = static_cast<float>(lArm_defaultKp_high[i]);
        Kd_lArm_high[i] = static_cast<float>(lArm_defaultKd_high[i]);
        Kp_rArm_high[i] = static_cast<float>(rArm_defaultKp_high[i]);
        Kd_rArm_high[i] = static_cast<float>(rArm_defaultKd_high[i]);

        // 超高刚度 (superhigh)
        Kp_lArm_superhigh[i] = static_cast<float>(lArm_defaultKp_superhigh[i]);
        Kd_lArm_superhigh[i] = static_cast<float>(lArm_defaultKd_superhigh[i]);
        Kp_rArm_superhigh[i] = static_cast<float>(rArm_defaultKp_superhigh[i]);
        Kd_rArm_superhigh[i] = static_cast<float>(rArm_defaultKd_superhigh[i]);

        // 拖拽模式 (tag) — Kp全0, Kd全12
        Kp_lArm_tag[i] = static_cast<float>(lArm_defaultKp_tag[i]);
        Kd_lArm_tag[i] = static_cast<float>(lArm_defaultKd_tag[i]);
        Kp_rArm_tag[i] = static_cast<float>(rArm_defaultKp_tag[i]);
        Kd_rArm_tag[i] = static_cast<float>(rArm_defaultKd_tag[i]);
    }

    std::cout << "PD参数已从YAML配置文件同步到PDInte" << std::endl;
}

//Kp，Kd输出封装
ArmPD PDInte::PDInteOutp(int modChoose, int rigidChoose){
    ArmPD mRes;
    if(modChoose == 0){
        //默认低刚度输出
        for(int i = 0; i < SINGLEARMDOF; i++){
            mRes.lArmKp.push_back(Kp_lArm_low[i]);
            mRes.lArmKd.push_back(Kd_lArm_low[i]);
            mRes.rArmKp.push_back(Kp_rArm_low[i]);
            mRes.rArmKd.push_back(Kd_rArm_low[i]);
        }
    }else if(modChoose == 5){
        //零力拖动模式
        for(int i = 0; i < SINGLEARMDOF; i++){
            mRes.lArmKp.push_back(Kp_lArm_tag[i]);
            mRes.lArmKd.push_back(Kd_lArm_tag[i]);
            mRes.rArmKp.push_back(Kp_rArm_tag[i]);
            mRes.rArmKd.push_back(Kd_rArm_tag[i]);
        }
    }
    else{
        if(rigidChoose == 1){
            //低刚度模式

            for(int i = 0; i < SINGLEARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_low[i]);
                mRes.lArmKd.push_back(Kd_lArm_low[i]);
                mRes.rArmKp.push_back(Kp_rArm_low[i]);
                mRes.rArmKd.push_back(Kd_rArm_low[i]);
            }
        }else if(rigidChoose == 2){
            //中等刚度模式

            for(int i = 0; i < SINGLEARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_mid[i]);
                mRes.lArmKd.push_back(Kd_lArm_mid[i]);
                mRes.rArmKp.push_back(Kp_rArm_mid[i]);
                mRes.rArmKd.push_back(Kd_rArm_mid[i]);
            }
        }else if(rigidChoose == 3){
            //高刚度模式

            for(int i = 0; i < SINGLEARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_high[i]);
                mRes.lArmKd.push_back(Kd_lArm_high[i]);
                mRes.rArmKp.push_back(Kp_rArm_high[i]);
                mRes.rArmKd.push_back(Kd_rArm_high[i]);
            }
        }
        else if(rigidChoose == 7){
            //超高刚度模式

            for(int i = 0; i < SINGLEARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_superhigh[i]);
                mRes.lArmKd.push_back(Kd_lArm_superhigh[i]);
                mRes.rArmKp.push_back(Kp_rArm_superhigh[i]);
                mRes.rArmKd.push_back(Kd_rArm_superhigh[i]);
            }
        } 
        else{
            //默认中刚度模式

            for(int i = 0; i < SINGLEARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_mid[i]);
                mRes.lArmKd.push_back(Kd_lArm_mid[i]);
                mRes.rArmKp.push_back(Kp_rArm_mid[i]);
                mRes.rArmKd.push_back(Kd_rArm_mid[i]);
            }
        }
    }
    return mRes;
}

//================================================================================
/*辅助函数*/

/*  类型转换函数：
------------------
注意：Frame类型转化对应的vector容器应当是六维度向量，其数据含义依次为：
[x, y, z, row, pitch, yaw] -- 三个平移量+三个欧拉角
*/

// 将 std::vector<double> 转换为 KDL::JntArray
KDL::JntArray vec_to_JntArray(const std::vector<double>& mVecInp) {
    KDL::JntArray jntArray;
    if (!mVecInp.empty()) {
        jntArray.resize(mVecInp.size());
        for (size_t i = 0; i < mVecInp.size(); ++i) {
            jntArray(i) = mVecInp[i];
        }
    }
    return jntArray;
}

// 将 KDL::JntArray 转换为 std::vector<double>
std::vector<double> JntArray_to_vec(const KDL::JntArray& mArrInp) {
    std::vector<double> vec;
    if (mArrInp.rows() > 0) {
        vec.resize(mArrInp.rows());
        for (unsigned int i = 0; i < mArrInp.rows(); ++i) {
            vec[i] = mArrInp(i);
        }
    }
    return vec;
}

// 将 std::vector<double> 转换为 KDL::Frame（六维向量：[x, y, z, roll, pitch, yaw]）
KDL::Frame vec_to_Frame(const std::vector<double>& mVecInp) {
    if (mVecInp.size() != 6) {
        throw std::runtime_error("Input vector must have 6 elements for Frame conversion.");
    }

    // 提取平移部分
    KDL::Vector translation(mVecInp[0], mVecInp[1], mVecInp[2]);

    // 提取旋转部分（欧拉角转旋转矩阵）
    KDL::Rotation rotation = KDL::Rotation::RPY(mVecInp[3], mVecInp[4], mVecInp[5]);

    return KDL::Frame(rotation, translation);
}

// 将 KDL::Frame 转换为 std::vector<double>（六维向量：[x, y, z, roll, pitch, yaw]）
std::vector<double> Frame_to_vec(const KDL::Frame& mFrameInp) {
    std::vector<double> vec(6);

    // 提取平移部分
    vec[0] = mFrameInp.p.x();
    vec[1] = mFrameInp.p.y();
    vec[2] = mFrameInp.p.z();

    // 提取旋转部分（旋转矩阵转欧拉角）
    mFrameInp.M.GetRPY(vec[3], vec[4], vec[5]);

    return vec;
}

//================================================================================
/*整体封装*/

int sgnFunc(double mInp){
    if(mInp > 1e-7){
        return 1;
    }else if(mInp < -1*1e-7){
        return -1;
    }else{
        return 0;
    }
}
double corUpBound(double mInp, double upperbound, double lowerbound){
    double mRes;
    double tInp = mInp;
    if(mInp > upperbound){
        mRes = upperbound;
    }else if(mInp < lowerbound){
        mRes = lowerbound;
        /**
    }else if(std::abs(std::abs(tInp) - 1.5708) < 1e-4){
        if(tInp > 0){
            mInp = 1.5707;
        }else{
            mInp = -1.5707;
        }
        /**/
    }else{
        mRes = mInp;
    }
    return mRes;
}
KDL::Frame FrameCor(KDL::Frame FrameInp){
    double row, pitch, yaw;
    FrameInp.M.GetRPY(row, pitch, yaw);
    double upperBound = 3.14159;
    double lowerBound = -3.14159;
    
    KDL::Frame resFrame;
    resFrame.p[0] = FrameInp.p[0];
    resFrame.p[1] = FrameInp.p[1];
    resFrame.p[2] = FrameInp.p[2];
    double mRow = corUpBound(row, upperBound, lowerBound);
    double mPitch = corUpBound(pitch, upperBound, lowerBound);
    double mYaw = corUpBound(yaw, upperBound, lowerBound);
    resFrame.M = KDL::Rotation::RPY(mRow, mPitch, mYaw);

    return resFrame;
}


void ArmCtlOptInterface::init(
    KDL::JntArray stPos_lArm, KDL::JntArray stPos_rArm,
    JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,
    modCtl mod_ctl_msg,
    optiPrintMsgCtl mPrintMsgPkg,   //程序打印控制参数
    optimizerCtl m_opti_ctl_msg     //优化器参数
){
    this->stPos_l = stPos_lArm;
    this->stPos_r = stPos_rArm;
    this->outpMod = mod_ctl_msg.transMod;
    this->GravCompInpChoose = mod_ctl_msg.gravComp_ctl.GravCompInpMod;

    //初始化重力补偿函数
    GravComp.init_GravComp(
        mod_ctl_msg.gravComp_ctl.mIfUseInpFtr, mod_ctl_msg.gravComp_ctl.mIfUseOutpFtr,
        stPos_lArm, stPos_rArm,
        mod_ctl_msg.gravComp_ctl.mIfUseFricComp, 
        mod_ctl_msg.gravComp_ctl.stJntID_lArm, mod_ctl_msg.gravComp_ctl.stJntID_rArm, mod_ctl_msg.gravComp_ctl.urdf_choose
    );

    //初始化关节控制模式
    JntSpace_track.init(
        stPos_lArm, stPos_rArm
    );

    //初始化笛卡尔空间轨迹求解模式
    std::vector<double> lArmStAngConf, rArmStAngConf;
    for(int i = 0; i < stPos_lArm.rows(); i++){
        lArmStAngConf.push_back(stPos_lArm(i));
    }
    for(int i = 0; i < stPos_rArm.rows(); i++){
        rArmStAngConf.push_back(stPos_rArm(i));
    }

    FrameSpace_track.init_solver(
        lArm_LimJnt, rArm_LimJnt,
        lArmStAngConf, rArmStAngConf,
        mPrintMsgPkg,
        m_opti_ctl_msg
    );

    //初始化PD参数
    mArmPDInte.PDInteInit();

    //初始化过程参数
    for(int i = 0; i < SINGLEARMDOF; i++){
        rigidityMod_l[i] = mod_ctl_msg.base_rigiditiCtl_mod_l[i];
        rigidityMod_r[i] = mod_ctl_msg.base_rigiditiCtl_mod_r[i];
    }

    //所有的上一轮响应置零
    lArmLst_outpWrap.posDes.assign(SINGLEARMDOF, 0.0);              rArmLst_outpWrap.posDes.assign(SINGLEARMDOF, 0.0);
    lArmLst_outpWrap.velDes.assign(SINGLEARMDOF, 0.0);              rArmLst_outpWrap.velDes.assign(SINGLEARMDOF, 0.0);
    lArmLst_outpWrap.KpDes.assign(SINGLEARMDOF, 0.0);               rArmLst_outpWrap.KpDes.assign(SINGLEARMDOF, 0.0);
    lArmLst_outpWrap.KdDes.assign(SINGLEARMDOF, 0.0);               rArmLst_outpWrap.KdDes.assign(SINGLEARMDOF, 0.0);
    lArmLst_outpWrap.gravComp_TorDes.assign(SINGLEARMDOF, 0.0);     rArmLst_outpWrap.gravComp_TorDes.assign(SINGLEARMDOF, 0.0);
    lArmLst_outpWrap.FeedForward_TorDes.assign(SINGLEARMDOF, 0.0);  rArmLst_outpWrap.FeedForward_TorDes.assign(SINGLEARMDOF, 0.0);
    lArmLst_outpWrap.InteriaComp_TorDes.assign(SINGLEARMDOF, 0.0);  rArmLst_outpWrap.InteriaComp_TorDes.assign(SINGLEARMDOF, 0.0);
}

void ArmCtlOptInterface::Traj_cal(
    bool ifUpdateLim,
    JntLimMsg allArmLimJnt,

    //笛卡尔空间目标值
    KDL::Frame lArm_desPos_Frame, KDL::Frame rArm_desPos_Frame,
    std::vector<double> lArm_desTor_Frame, std::vector<double> rArm_desTor_Frame,

    //关节空间的目标值
    KDL::JntArray lArm_desPos_Jnt, KDL::JntArray rArm_desPos_Jnt,
    std::vector<double> lArm_desTor_Jnt, std::vector<double> rArm_desTor_Jnt,

    //反馈值
    KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
    KDL::JntArray lArm_velRet, KDL::JntArray rArm_velRet,
    std::vector<double> lArm_iq_ret, std::vector<double> rArm_iq_ret,

    modCtl mod_ctl_msg,
    bool ifRedefineJntTrajStab
){
    //依据不同的计算模式选择不同的计算接口
    this->ifOutpUseGravComp = mod_ctl_msg.gravComp_ctl.ifUseGravComp;
    //this->GravCompInpChoose = mod_ctl_msg.gravComp_ctl.GravCompInpMod;
    this->outpMod = mod_ctl_msg.transMod;

    //利用反馈电流计算电机力矩
    std::vector<double> JntTor_l, JntTor_r;
    for(int i = 0; i < SINGLEARMDOF; i++){
        JntTor_l.push_back(lArm_iq_ret[i] * currentConstant_l[i]);
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        JntTor_r.push_back(rArm_iq_ret[i] * currentConstant_r[i]);
    }

    //进行一轮控制计算
    if(mod_ctl_msg.transMod == 2 || mod_ctl_msg.transMod == 4){
        //采用点位控制模式：
        FrameSpace_track.FrameTrack(
            ifUpdateLim,
            allArmLimJnt,

            lArm_desPos_Frame, rArm_desPos_Frame,
            lArm_desTor_Frame, rArm_desTor_Frame,

            lArm_posRet, rArm_posRet,
            lArm_velRet, rArm_velRet,
            JntTor_l, JntTor_r,

            mod_ctl_msg
        );

        //刷新刚度模式
        for(int k = 0; k < SINGLEARMDOF; k++){
            rigidityMod_l[k] = mod_ctl_msg.base_rigiditiCtl_mod_l[k];
            rigidityMod_r[k] = mod_ctl_msg.base_rigiditiCtl_mod_r[k];
        }
    }else if(mod_ctl_msg.transMod == 1 || mod_ctl_msg.transMod == 3){
        //采用关节控制模式：
        JntSpace_track.JntTrajCal(
            lArm_desPos_Jnt, rArm_desPos_Jnt,
            lArm_desTor_Jnt, rArm_desTor_Jnt,

            lArm_posRet, rArm_posRet,
            lArm_velRet, rArm_velRet,
            JntTor_l, JntTor_r,

            mod_ctl_msg,
            ifRedefineJntTrajStab
        );

        //刷新刚度模式
        for(int k = 0; k < SINGLEARMDOF; k++){
            rigidityMod_l[k] = mod_ctl_msg.base_rigiditiCtl_mod_l[k];
            rigidityMod_r[k] = mod_ctl_msg.base_rigiditiCtl_mod_r[k];
        }
    }else if(mod_ctl_msg.transMod == 5){
        //零力拖动模式

        //这里，关节目标位置与目标速度均置零，PD参数采用零力拖动对应的PD参数，仅进行重力补偿相关计算
        
    }
    else{
        //不进行计算
    }
}

std::vector<outpWrap> ArmCtlOptInterface::Traj_temp_outp(
    KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
    KDL::JntArray lArm_velRet, KDL::JntArray rArm_velRet,
    std::vector<double> lArm_iq_Ret, std::vector<double> rArm_iq_Ret
){
    outpWrap lArm_outpWrap;
    outpWrap rArm_outpWrap;

    //利用反馈电流计算反馈力矩
    std::vector<double> lArm_TorRet, rArm_TorRet;
    for(int i = 0; i < SINGLEARMDOF; i++){
        lArm_TorRet.push_back(lArm_iq_Ret[i] * currentConstant_l[i]);
        rArm_TorRet.push_back(rArm_iq_Ret[i] * currentConstant_r[i]);
    }
    //位置控制量、PD控制量计算
    std::vector<std::vector<ctlOutp>> posCtlRes;
    ArmPD allArmPDoutp;
    if(this->outpMod == 2 || this->outpMod == 4){
        //采用点位控制模式
        posCtlRes = FrameSpace_track.IKsolveResOutp(lArm_TorRet, rArm_TorRet);
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, rigidityMod_l[0]);

    }else if(this->outpMod == 1 || this->outpMod == 3){
        //采用关节轨迹控制模式
        posCtlRes = JntSpace_track.solveResOutp(lArm_TorRet, rArm_TorRet);
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, rigidityMod_l[0]);
    }else if(this->outpMod == 5){
        //零力拖动模式

        //这里，关节目标位置与目标速度均置零，PD参数采用零力拖动对应的PD参数，仅进行重力补偿相关计算

        posCtlRes.resize(2);
        posCtlRes[0].resize(SINGLEARMDOF);
        posCtlRes[1].resize(SINGLEARMDOF);

        for(int i = 0; i < SINGLEARMDOF; i++){
            posCtlRes[0][i].posDes = 0.0;
            posCtlRes[0][i].velDes = 0.0;
            posCtlRes[0][i].grav_TorDes = 0.0;
            posCtlRes[0][i].interia_TorDes = 0.0;
            posCtlRes[0][i].feedforward_TorDes = 0.0;
            posCtlRes[0][i].ifReachEnd = false;

            posCtlRes[1][i].posDes = 0.0;
            posCtlRes[1][i].velDes = 0.0;
            posCtlRes[1][i].grav_TorDes = 0.0;
            posCtlRes[1][i].interia_TorDes = 0.0;
            posCtlRes[1][i].feedforward_TorDes = 0.0;
            posCtlRes[1][i].ifReachEnd = false;
        }

        //获取PD参数
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, 0);
    }
    else{
        //此时输出为空值
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, rigidityMod_l[0]);
    }

    //输出解包
    if(this->outpMod != 0){
        for(int i = 0; i < posCtlRes[0].size(); i++){
            lArm_outpWrap.posDes.push_back(posCtlRes[0][i].posDes);
            lArm_outpWrap.velDes.push_back(posCtlRes[0][i].velDes);

            lArm_outpWrap.gravComp_TorDes.push_back(posCtlRes[0][i].grav_TorDes);
            lArm_outpWrap.InteriaComp_TorDes.push_back(posCtlRes[0][i].interia_TorDes);
            lArm_outpWrap.FeedForward_TorDes.push_back(posCtlRes[0][i].feedforward_TorDes);

            lArm_outpWrap.ifComToEnd.push_back(posCtlRes[0][i].ifReachEnd);
        }
        for(int i = 0; i < posCtlRes[1].size(); i++){
            rArm_outpWrap.posDes.push_back(posCtlRes[1][i].posDes);
            rArm_outpWrap.velDes.push_back(posCtlRes[1][i].velDes);

            rArm_outpWrap.gravComp_TorDes.push_back(posCtlRes[1][i].grav_TorDes);
            rArm_outpWrap.InteriaComp_TorDes.push_back(posCtlRes[1][i].interia_TorDes);
            rArm_outpWrap.FeedForward_TorDes.push_back(posCtlRes[1][i].feedforward_TorDes);

            rArm_outpWrap.ifComToEnd.push_back(posCtlRes[1][i].ifReachEnd);
        }

        lArm_outpWrap.KpDes = allArmPDoutp.lArmKp;
        lArm_outpWrap.KdDes = allArmPDoutp.lArmKd;

        rArm_outpWrap.KpDes = allArmPDoutp.rArmKp;
        rArm_outpWrap.KdDes = allArmPDoutp.rArmKd;

    }else{
        lArm_outpWrap.posDes.assign(SINGLEARMDOF, 0.0);
        lArm_outpWrap.velDes.assign(SINGLEARMDOF, 0.0);

        lArm_outpWrap.gravComp_TorDes.assign(SINGLEARMDOF, 0.0);
        lArm_outpWrap.InteriaComp_TorDes.assign(SINGLEARMDOF, 0.0);
        lArm_outpWrap.FeedForward_TorDes.assign(SINGLEARMDOF, 0.0);

        lArm_outpWrap.ifComToEnd.assign(SINGLEARMDOF, false);

        rArm_outpWrap.posDes.assign(SINGLEARMDOF, 0.0);
        rArm_outpWrap.velDes.assign(SINGLEARMDOF, 0.0);

        rArm_outpWrap.gravComp_TorDes.assign(SINGLEARMDOF, 0.0);
        rArm_outpWrap.InteriaComp_TorDes.assign(SINGLEARMDOF, 0.0);
        rArm_outpWrap.FeedForward_TorDes.assign(SINGLEARMDOF, 0.0);

        rArm_outpWrap.ifComToEnd.assign(SINGLEARMDOF, false);

        for(int i = 0; i < SINGLEARMDOF; i++){
            lArm_outpWrap.posDes.at(i) = stPos_l(i);
            rArm_outpWrap.posDes.at(i) = stPos_r(i);
        }

        lArm_outpWrap.KpDes = allArmPDoutp.lArmKp;
        lArm_outpWrap.KdDes = allArmPDoutp.lArmKd;

        rArm_outpWrap.KpDes = allArmPDoutp.rArmKp;
        rArm_outpWrap.KdDes = allArmPDoutp.rArmKd;
    }

    /**
    std::cout << "============================================" << std::endl;
    std::cout << "outpMod = " << this->outpMod << std::endl;
    std::cout << "gravComp_mod = " << this->GravCompInpChoose << std::endl;
    for(int i = 0; i < lArm_outpWrap.posDes.size(); i++){
        std::cout << "lArm_posDes = " << lArm_outpWrap.posDes[i] << std::endl;
    }
    for(int i = 0; i < lArm_outpWrap.velDes.size(); i++){
        std::cout << "lArm_velDes = " << lArm_outpWrap.velDes[i] << std::endl;
    }
    for(int i = 0; i < rArm_outpWrap.posDes.size(); i++){
        std::cout << "rArm_posDes = " << rArm_outpWrap.posDes[i] << std::endl;
    }
    for(int i = 0; i < rArm_outpWrap.velDes.size(); i++){
        std::cout << "rArm_velDes = " << rArm_outpWrap.velDes[i] << std::endl;
    }
    std::cout << "============================================" << std::endl;
    /**/

    //重力、惯性力综合补偿叠加
    if(this->outpMod != 5){
        //非零力拖动控制模式

        if(this->ifOutpUseGravComp){
            //采用重力补偿
            TorCompWrap TorCompRes;

            if(this->GravCompInpChoose == 2){
                //重力补偿输入解包：全部使用反馈值
                std::vector<double> lArm_posRet_vec, rArm_posRet_vec, lArm_velRet_vec, rArm_velRet_vec;
                for(int i = 0; i < lArm_posRet.rows(); i++){
                    lArm_posRet_vec.push_back(lArm_posRet(i));
                    lArm_velRet_vec.push_back(lArm_velRet(i));
                }
                for(int i = 0; i < rArm_posRet.rows(); i++){
                    rArm_posRet_vec.push_back(rArm_posRet(i));
                    rArm_velRet_vec.push_back(rArm_velRet(i));
                }

                //计算重力补偿值
                TorCompRes = GravComp.GravCompOutp(
                    lArm_posRet_vec, lArm_velRet_vec, lArm_TorRet,
                    rArm_posRet_vec, rArm_velRet_vec, rArm_TorRet
                );
            }else if(this->GravCompInpChoose == 3){
                //仅位置量采用反馈值作为输入的重力补偿
                std::vector<double> lArm_posRet_vec, rArm_posRet_vec;
                for(int i = 0; i < lArm_posRet.rows(); i++){
                    lArm_posRet_vec.push_back(lArm_posRet(i));
                }
                for(int i = 0; i < rArm_posRet.rows(); i++){
                    rArm_posRet_vec.push_back(rArm_posRet(i));
                }

                std::vector<double> lArm_totalJntTor;
                std::vector<double> rArm_totalJntTor;
                for(int i = 0; i < lArm_outpWrap.gravComp_TorDes.size(); i++){
                    lArm_totalJntTor.push_back(lArmLst_outpWrap.gravComp_TorDes[i] + lArmLst_outpWrap.InteriaComp_TorDes[i] + lArmLst_outpWrap.FeedForward_TorDes[i]);
                    rArm_totalJntTor.push_back(rArmLst_outpWrap.gravComp_TorDes[i] + rArmLst_outpWrap.InteriaComp_TorDes[i] + rArmLst_outpWrap.FeedForward_TorDes[i]);
                }

                TorCompRes = GravComp.GravCompOutp(
                    lArm_posRet_vec, lArm_outpWrap.velDes, lArm_totalJntTor,
                    rArm_posRet_vec, rArm_outpWrap.velDes, rArm_totalJntTor
                );
            }
            else{
                //直接利用控制输出值作重力补偿输入值
                std::vector<double> lArm_totalJntTor;
                std::vector<double> rArm_totalJntTor;
                for(int i = 0; i < lArm_outpWrap.gravComp_TorDes.size(); i++){
                    lArm_totalJntTor.push_back(lArmLst_outpWrap.gravComp_TorDes[i] + lArmLst_outpWrap.InteriaComp_TorDes[i] + lArmLst_outpWrap.FeedForward_TorDes[i]);
                    rArm_totalJntTor.push_back(rArmLst_outpWrap.gravComp_TorDes[i] + rArmLst_outpWrap.InteriaComp_TorDes[i] + rArmLst_outpWrap.FeedForward_TorDes[i]);
                }
                TorCompRes = GravComp.GravCompOutp(
                    lArm_outpWrap.posDes, lArm_outpWrap.velDes, lArm_totalJntTor,
                    rArm_outpWrap.posDes, rArm_outpWrap.velDes, rArm_totalJntTor
                );
            }
        

            //叠加力补偿结果
            for(int i = 0; i < lArm_outpWrap.gravComp_TorDes.size(); i++){
                //重力补偿叠加
                lArm_outpWrap.gravComp_TorDes[i] = lArm_outpWrap.gravComp_TorDes[i] + TorCompRes.lArm_Comp_grav[i];
                rArm_outpWrap.gravComp_TorDes[i] = rArm_outpWrap.gravComp_TorDes[i] + TorCompRes.rArm_comp_grav[i];

                //惯性力补偿叠加
                lArm_outpWrap.InteriaComp_TorDes[i] = lArm_outpWrap.InteriaComp_TorDes[i] + TorCompRes.lArm_Comp_InteriaComb[i];
                rArm_outpWrap.InteriaComp_TorDes[i] = rArm_outpWrap.InteriaComp_TorDes[i] + TorCompRes.rArm_Comp_InteriaComb[i];

                //力前馈叠加，将自带stribeck摩擦力补偿叠加到力前馈里面去
                lArm_outpWrap.FeedForward_TorDes[i] = lArm_outpWrap.FeedForward_TorDes[i] + TorCompRes.lArm_feedForward[i];
                rArm_outpWrap.FeedForward_TorDes[i] = rArm_outpWrap.FeedForward_TorDes[i] + TorCompRes.rArm_feedForward[i];
            }
        }

        //力前馈叠加
        /*
            此部分尚未作开发，后续会填上
        */

    }else{
        //零力拖动控制模式 -- 不论其他控制参量如何，直接执行基于反馈值的重力补偿函数
        TorCompWrap gravCompRes;
        std::vector<double> lArm_posRet_vec, rArm_posRet_vec, lArm_velRet_vec, rArm_velRet_vec;
        for(int i = 0; i < lArm_posRet.rows(); i++){
            lArm_posRet_vec.push_back(lArm_posRet(i));
            lArm_velRet_vec.push_back(lArm_velRet(i));
        }
        for(int i = 0; i < rArm_posRet.rows(); i++){
            rArm_posRet_vec.push_back(rArm_posRet(i));
            rArm_velRet_vec.push_back(rArm_velRet(i));
        }

        gravCompRes = GravComp.onlyGravComp(
            lArm_posRet_vec, lArm_velRet_vec,
            rArm_posRet_vec, rArm_velRet_vec
        );

        //叠加力补偿结果
        for(int i = 0; i < lArm_outpWrap.gravComp_TorDes.size(); i++){
            lArm_outpWrap.gravComp_TorDes[i] = lArm_outpWrap.gravComp_TorDes[i] + gravCompRes.lArm_Comp_grav[i];
            rArm_outpWrap.gravComp_TorDes[i] = rArm_outpWrap.gravComp_TorDes[i] + gravCompRes.rArm_comp_grav[i];

            lArm_outpWrap.InteriaComp_TorDes[i] = lArm_outpWrap.InteriaComp_TorDes[i] + gravCompRes.lArm_Comp_InteriaComb[i];
            rArm_outpWrap.InteriaComp_TorDes[i] = rArm_outpWrap.InteriaComp_TorDes[i] + gravCompRes.rArm_Comp_InteriaComb[i];

            lArm_outpWrap.FeedForward_TorDes[i] = lArm_outpWrap.FeedForward_TorDes[i] + gravCompRes.lArm_feedForward[i];
            rArm_outpWrap.FeedForward_TorDes[i] = rArm_outpWrap.FeedForward_TorDes[i] + gravCompRes.rArm_feedForward[i];
        }

        //可能的力前馈叠加
        /*
            此部分尚未作开发，后续会填上
        */
    }

    //如果关节静止，则要强制刷掉所有输出

    //过程量更新
    lArmLst_outpWrap = lArm_outpWrap;
    rArmLst_outpWrap = rArm_outpWrap;

    std::vector<outpWrap> mRes = {lArm_outpWrap, rArm_outpWrap};

    return mRes;
}


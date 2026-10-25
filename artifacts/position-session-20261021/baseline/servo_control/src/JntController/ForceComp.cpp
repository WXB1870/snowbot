#include "JntController/ForceComp.h"


float sgn(float x)  //符号函数sgn(x)
{
    if(std::fabs(x - 0) < 1e-3){
        return 0;
    }
    else{
        if(x < 0){
            return -1;
        }
        else{
            return 1;
        }
    }
}

//----------------------------------------------------------------------------
//简易模糊控制
/*  模糊依据 -- 速度偏差量，加速度
    模糊核心 -- 速度偏差量越大，加速度越大，阻尼
*/
std::vector<std::vector<double>> Jnt_velErr_region = {
    {0.5, 1, 1.2, 1.4, 1.6},
    {0.5, 1, 1.2, 1.4, 1.6},
    {0.5, 1, 1.2, 1.4, 1.6},
    {0.5, 1, 1.2, 1.4, 1.6},
    {0.5, 1, 1.2, 1.4, 1.6},
    {0.5, 1, 1.2, 1.4, 1.6},
    {0.5, 1, 1.2, 1.4, 1.6}
};
std::vector<std::vector<double>> Jnt_acc_region = {
    {30, 60, 90, 120, 150},
    {30, 60, 90, 120, 150},
    {30, 60, 90, 120, 150},
    {30, 60, 90, 120, 150},
    {30, 60, 90, 120, 150},
    {30, 60, 90, 120, 150},
    {30, 60, 90, 120, 150}
};

FuzzyKpComp_simp::FuzzyKpComp_simp(int JntID){

    //Kp补偿定档
    std::vector<double> KpInte = {
        0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.8, 2.0
    };

    //模糊矩阵每一行对应不同的速度区间，每一列对应不同的加速度
    fuzzy_matrix = {
        {KpInte[0], KpInte[1], KpInte[2], KpInte[3], KpInte[4]},
        {KpInte[1], KpInte[2], KpInte[3], KpInte[4], KpInte[5]},
        {KpInte[2], KpInte[3], KpInte[4], KpInte[5], KpInte[6]},
        {KpInte[3], KpInte[4], KpInte[5], KpInte[6], KpInte[7]},
        {KpInte[4], KpInte[5], KpInte[6], KpInte[7], KpInte[8]}
    };

    //解析ID输入并赋值
    int mID = JntID;
    if(mID > 7){
        mID = 7;
    }else if(mID < 1){
        mID = 1;
    }

    velErr_region = Jnt_velErr_region[mID - 1];
    acc_region = Jnt_acc_region[mID - 1]; 
}

//模糊补偿函数
double FuzzyKpComp_simp::KdComp(double JntVelErr, double JntAcc){
    
    //计算隶属度，用三角隶属函数
    std::vector<double> velMembership(velErr_region.size(), 0.0);
    std::vector<double> accMembership(acc_region.size(), 0.0);

    //速度误差隶属度
    for (size_t i = 0; i < velErr_region.size(); ++i) {
        if (i == 0) {
            // 第一个区间
            if (JntVelErr <= velErr_region[i]) {
                velMembership[i] = 1.0;
            } else if (JntVelErr <= velErr_region[i + 1]) {
                velMembership[i] = (velErr_region[i + 1] - JntVelErr) / 
                                   (velErr_region[i + 1] - velErr_region[i]);
            }
        } else if (i == velErr_region.size() - 1) {
            // 最后一个区间
            if (JntVelErr >= velErr_region[i]) {
                velMembership[i] = 1.0;
            } else if (JntVelErr >= velErr_region[i - 1]) {
                velMembership[i] = (JntVelErr - velErr_region[i - 1]) / 
                                   (velErr_region[i] - velErr_region[i - 1]);
            }
        } else {
            // 中间区间
            if (JntVelErr >= velErr_region[i - 1] && JntVelErr <= velErr_region[i]) {
                velMembership[i] = (JntVelErr - velErr_region[i - 1]) / 
                                   (velErr_region[i] - velErr_region[i - 1]);
            } else if (JntVelErr >= velErr_region[i] && JntVelErr <= velErr_region[i + 1]) {
                velMembership[i] = (velErr_region[i + 1] - JntVelErr) / 
                                   (velErr_region[i + 1] - velErr_region[i]);
            }
        }
    }

    // 加速度的隶属度计算（三角隶属函数）
    for (size_t i = 0; i < acc_region.size(); ++i) {
        if (i == 0) {
            if (JntAcc <= acc_region[i]) {
                accMembership[i] = 1.0;
            } else if (JntAcc <= acc_region[i + 1]) {
                accMembership[i] = (acc_region[i + 1] - JntAcc) / 
                                  (acc_region[i + 1] - acc_region[i]);
            }
        } else if (i == acc_region.size() - 1) {
            if (JntAcc >= acc_region[i]) {
                accMembership[i] = 1.0;
            } else if (JntAcc >= acc_region[i - 1]) {
                accMembership[i] = (JntAcc - acc_region[i - 1]) / 
                                  (acc_region[i] - acc_region[i - 1]);
            }
        } else {
            if (JntAcc >= acc_region[i - 1] && JntAcc <= acc_region[i]) {
                accMembership[i] = (JntAcc - acc_region[i - 1]) / 
                                  (acc_region[i] - acc_region[i - 1]);
            } else if (JntAcc >= acc_region[i] && JntAcc <= acc_region[i + 1]) {
                accMembership[i] = (acc_region[i + 1] - JntAcc) / 
                                  (acc_region[i + 1] - acc_region[i]);
            }
        }
    }


    // 2. 模糊推理（使用重心法）
    double numerator = 0.0;
    double denominator = 0.0;
    
    for (size_t i = 0; i < velMembership.size(); ++i) {
        for (size_t j = 0; j < accMembership.size(); ++j) {
            //double firingStrength = std::min(velMembership[i], accMembership[j]);
            double firingStrength = velMembership[i] * accMembership[j];
            
            // 确保索引在模糊矩阵范围内
            if (i < fuzzy_matrix.size() && j < fuzzy_matrix[i].size()) {
                numerator += firingStrength * fuzzy_matrix[i][j];
                denominator += firingStrength;
            }
        }
    }

    // 3. 去模糊化
    double kdCompValue = 0.0;
    if (denominator > 1e-6) {  // 避免除零
        kdCompValue = numerator / denominator + 1;
    }

    return kdCompValue;
}

//----------------------------------------------------------------------------

//简易刚度补偿：
double K_comp::posComp(double IqInp, int JointID){
    int mID = JointID;
    if(mID > 7){
        mID = 7;
    }else if(mID < 1){
        mID = 1;
    }

    if(ifStart){
        dKalmanFilter.resize(7);
        for(int i = 0; i < 7; i++){
            dKalmanFilter[i].FilterInit(IqInp, all_Qgain[i], all_Rgain[i]);
        }
        ifStart = false;
    }
    double IqInp_ftred;
    if(ifUseFtr){
        IqInp_ftred = dKalmanFilter[mID - 1].FilteredResCal(IqInp);
    }else{
        IqInp_ftred = IqInp;
    }

    double pos_comp = (IqInp_ftred * JointKt[mID - 1]) * cor[mID - 1] / Joint_K[mID - 1];

    return pos_comp;
}

//静摩擦补偿
void Stribeck_Fric::StribeckModelInit(float mfs, float mfc, float mfstic, float mvs, float mgamma, float T0)
{
    this->fs = mfs;
    this->fc = mfc;
    this->fstic = mfstic;
    this->vs = mvs;
    this->gamma = mgamma;
    this->mTo = T0;

    //过程控制来唵初始化
    last_velDirection = 0;
}
float Stribeck_Fric::StribeckFricCal(float Tinp, float vel, float posInp)
{
    
    int direction = sgn(vel);
    if(direction == 0){
        direction = last_velDirection;
    }
    last_velDirection = direction;

    float mres;

    if(vel > this->vs){
        mres = (this->fc*Tinp + ((this->fs*Tinp) - this->fc*Tinp)*std::exp(-1*std::fabs(vel/this->vs)*this->gamma))*sgn(vel) + vel*this->fstic + this->mTo * sgn(vel);
    }else{
        mres = vel * ((this->fs*Tinp) / this->vs) + this->mTo * sgn(vel);
        //mres = vel * ((this->fs*Tinp) / this->vs) + this->mTo * direction;
    }
    return mres;
}

//摩擦补偿包装
std::vector<std::vector<double>> Stribeck_FricCompWrap::fricCompRes(
    std::vector<double> lArm_tor, std::vector<double> rArm_tor,
    std::vector<double> lArm_vel, std::vector<double> rArm_vel,
    std::vector<double> lArm_pos, std::vector<double> rArm_pos
){
    std::vector<double> lArm_fricTorComp, rArm_fricTorComp;
    lArm_fricTorComp.resize(7);
    rArm_fricTorComp.resize(7);

    //满足条件，则执行初始化代码
    if(ifFirst){
        dKalmanFtr_TorInp_l.resize(7);
        dKalmanFtr_TorInp_r.resize(7);
        dKalmanFtr_velInp_l.resize(7);
        dKalmanFtr_velInp_r.resize(7);

        lArm_fricComp.resize(7);
        rArm_fricComp.resize(7);

        //初始化滤波器
        for(int i = 0; i < 7; i++){
            dKalmanFtr_TorInp_l[i].FilterInit(lArm_tor[i], all_Qgain_tor[i], all_Rgain_tor[i]);
            dKalmanFtr_TorInp_r[i].FilterInit(rArm_tor[i], all_Qgain_tor[i], all_Rgain_tor[i]);

            dKalmanFtr_velInp_l[i].FilterInit(lArm_vel[i], all_Qgain_vel[i], all_Rgain_vel[i]);
            dKalmanFtr_velInp_r[i].FilterInit(rArm_vel[i], all_Qgain_vel[i], all_Rgain_vel[i]);
        }

        //初始化补偿器
        for(int i = 0; i < 7; i++){
            lArm_fricComp[i].StribeckModelInit(lArm_fs[i], lArm_fc[i], lArm_fstic[i], lArm_vs[i], gamma[i], lArm_mT0[i]);
            rArm_fricComp[i].StribeckModelInit(rArm_fs[i], rArm_fc[i], rArm_fstic[i], rArm_vs[i], gamma[i], rArm_mT0[i]);
        }
        ifFirst = false;
    }

    std::vector<double> lArm_tor_ftred, rArm_tor_ftred, lArm_vel_ftred, rArm_vel_ftred;
    if(ifUseInpFtr){
        lArm_tor_ftred.resize(7);
        rArm_tor_ftred.resize(7);
        lArm_vel_ftred.resize(7);
        rArm_vel_ftred.resize(7);

        //执行滤波算法
        for(int i = 0; i < 7; i++){
            lArm_tor_ftred[i] = dKalmanFtr_TorInp_l[i].FilteredResCal(lArm_tor[i]);
            rArm_tor_ftred[i] = dKalmanFtr_TorInp_r[i].FilteredResCal(rArm_tor[i]);

            lArm_vel_ftred[i] = dKalmanFtr_velInp_l[i].FilteredResCal(lArm_vel[i]);
            rArm_vel_ftred[i] = dKalmanFtr_velInp_r[i].FilteredResCal(rArm_vel[i]);
        }
    }else{
        lArm_tor_ftred = lArm_tor;
        rArm_tor_ftred = rArm_tor;
        lArm_vel_ftred = lArm_vel;
        rArm_vel_ftred = rArm_vel;
    }

    for(int i = 0; i < 7; i++){
        lArm_fricTorComp[i] = lArm_fricComp[i].StribeckFricCal(lArm_tor[i], lArm_vel[i], lArm_pos[i]);
        rArm_fricTorComp[i] = rArm_fricComp[i].StribeckFricCal(rArm_tor[i], rArm_vel[i], rArm_pos[i]);
    }

    std::vector<std::vector<double>> mRes;
    mRes.push_back(lArm_fricTorComp);
    mRes.push_back(rArm_fricTorComp);

    return mRes;
}

//插补输出值过滤
std::vector<std::vector<double>> runinOutpFtr::Outp_ftr(std::vector<double> lArm_inp, std::vector<double> rArm_inp){
    std::vector<double> lArm_inpFtred;
    std::vector<double> rArm_inpFtred;

    lArm_inpFtred.resize(7);
    rArm_inpFtred.resize(7);

    if(ifFirst){
        ifFirst = false;

        dKalmanFilter_l.resize(7);
        dKalmanFilter_r.resize(7);

        for(int i = 0; i < 7; i++){
            dKalmanFilter_l[i].FilterInit(lArm_inp[i], all_Qgain[i], all_Rgain[i]);
            dKalmanFilter_r[i].FilterInit(rArm_inp[i], all_Qgain[i], all_Rgain[i]);
        }
    }
    
    for(int i = 0; i < 7; i++){
        lArm_inpFtred[i] = dKalmanFilter_l[i].FilteredResCal(lArm_inp[i]);
        rArm_inpFtred[i] = dKalmanFilter_r[i].FilteredResCal(rArm_inp[i]);
    }

    std::vector<std::vector<double>> mRes;
    mRes.push_back(lArm_inpFtred);
    mRes.push_back(rArm_inpFtred);
    return mRes;
}

//----------------------------------------------------------------------------
//滤波器类

void kalman_filter::mMatrixMul(double a[],double b[],int m,int n,int k,double c[])
{ int i,j,l,u;
    for (i=0; i<=m-1; i++){
        for (j=0; j<=k-1; j++)
        { u=i*k+j; c[u]=0.0;
            for (l=0; l<=n-1; l++)
            c[u]=c[u]+a[i*n+l]*b[l*k+j];
        }
    }
}

void kalman_filter::FilterInit(double inputVar, double mQgain, double mRgain)
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

double kalman_filter::FilteredResCal(double inputvar)
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
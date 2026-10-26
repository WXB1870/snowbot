#include "JntController/retZeroControl.h"


//==================================================================================================

TrajResult JointPos_Init::dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase)
{
    float t_half = mTrajTime / 2;
    float posDes_half = mposDes / 2;

    // ----- 计算轨迹及其增量结果储存 -----
    //加速过程
    std::vector<float> acc_traj_pos;
    std::vector<float> acc_traj_derPos;
    std::vector<float> acc_traj_vel;

    //减速过程
    std::vector<float> dAcc_traj_pos;
    std::vector<float> dAcc_traj_derPos;
    std::vector<float> dAcc_traj_vel;

    //匀速过程
    std::vector<float> stab_traj_pos;
    std::vector<float> stab_traj_derPos;
    std::vector<float> stab_traj_vel;

    //轨迹参数计算a, b, 和v_max
    if (accTimeRateBase <= 1.05) {
        std::cerr << "accTimeRateBase must be greater than 1.0 to avoid division by zero" << std::endl;
        accTimeRateBase = 1.05; // 设置默认安全值
    }
    float t_acc = t_half / accTimeRateBase;
    float t_stab = mTrajTime - 2*t_acc;
    float denominator = std::pow(t_half, 4) * (2*accTimeRateBase - 1);
    float a = -12 * std::pow(accTimeRateBase, 4) * posDes_half / denominator;
    float b = a * t_half / accTimeRateBase;

    //计算加速段轨迹
    int m_accTempLen = std::round(t_acc/this->control_T);
    float temp_T = this->control_T;
    float temp_pos_last = 0;
    for(int i = 0; i < m_accTempLen; i++){
        float temp_pos;
        temp_pos = (1.0/12.0)*a*std::pow(temp_T, 4) - (1.0/6.0)*b*std::pow(temp_T, 3);
        acc_traj_pos.push_back(temp_pos);
        acc_traj_derPos.push_back(temp_pos - temp_pos_last);
        acc_traj_vel.push_back((temp_pos - temp_pos_last)/control_T);

        temp_pos_last = temp_pos;
        temp_T = temp_T + this->control_T;
    }

    //计算匀速段轨迹
    int stab_tempTime = std::round((mTrajTime - 2*(m_accTempLen*this->control_T)) / this->control_T);
    float stabTime = this->control_T * stab_tempTime;
    float stab_trajLen = mposDes - 2*(acc_traj_pos.back());
    float stab_vel = stab_trajLen / stabTime;
    float stab_derTraj = stab_trajLen / stab_tempTime;
    for(int i = 0; i < stab_tempTime; i++){
        float temp_pos = temp_pos_last + stab_derTraj;

        stab_traj_pos.push_back(temp_pos);
        stab_traj_derPos.push_back(stab_derTraj);
        stab_traj_vel.push_back(stab_vel);

        temp_pos_last = temp_pos;
    }

    //计算减速段位移 -- 加速段位移的镜像
    for(int i = m_accTempLen - 1; i >= 0; i--){
        float temp_pos = temp_pos_last + acc_traj_derPos[i];
        float temp_vel = acc_traj_vel[i];
        dAcc_traj_pos.push_back(temp_pos);
        dAcc_traj_derPos.push_back(acc_traj_derPos[i]);
        dAcc_traj_vel.push_back(temp_vel);

        temp_pos_last = temp_pos;
    }

    //融合三段轨迹得到结果
    TrajResult mRes;
    mRes.positionIncrements = acc_traj_pos;
    mRes.VelocityTraj = acc_traj_vel;

    mRes.positionIncrements.insert(mRes.positionIncrements.end(), stab_traj_pos.begin(), stab_traj_pos.end());
    mRes.VelocityTraj.insert(mRes.VelocityTraj.end(), stab_traj_vel.begin(), stab_traj_vel.end());

    mRes.positionIncrements.insert(mRes.positionIncrements.end(), dAcc_traj_pos.begin(), dAcc_traj_pos.end());
    mRes.VelocityTraj.insert(mRes.VelocityTraj.end(), dAcc_traj_vel.begin(), dAcc_traj_vel.end());

    return mRes;
}

//插补函数
void JointPos_Init::retZero_runin(std::vector<double> lArm_stPos, std::vector<double> rArm_stPos){
    //初始化初始位置

    //刷新控制参量
    lArm_JntTraj.clear();
    rArm_JntTraj.clear();
    lArm_JntTraj.resize(LARMDOF);
    rArm_JntTraj.resize(RARMDOF);
    trajLen = 0;
    temp_times = 0;

    //轨迹长度储存
    std::vector<int> all_trajLen_l, all_trajLen_r;
    all_trajLen_l.assign(LARMDOF, 0.0);
    all_trajLen_r.assign(RARMDOF, 0.0);

    //开始插补
    for(int i = 0; i < LARMDOF; i++){
        double derPos = lArm_zeroPos[i] - lArm_stPos[i];
        TrajResult temp_result = dPos_queue_stabTime_dot(derPos, retZeroTime, 1.2);

        lArm_JntTraj[i].positionIncrements.resize(temp_result.positionIncrements.size());
        lArm_JntTraj[i].VelocityTraj.resize(temp_result.positionIncrements.size());
        all_trajLen_l[i] = temp_result.positionIncrements.size();

        for(int j = 0; j < temp_result.positionIncrements.size(); j++){
            lArm_JntTraj[i].positionIncrements[j] = temp_result.positionIncrements[j] + lArm_stPos[i];
            lArm_JntTraj[i].VelocityTraj[j] = temp_result.VelocityTraj[j];
        }
    }

    for(int i = 0; i < RARMDOF; i++){
        double derPos = rArm_zeroPos[i] - rArm_stPos[i];
        TrajResult temp_result = dPos_queue_stabTime_dot(derPos, retZeroTime, 1.2);

        rArm_JntTraj[i].positionIncrements.resize(temp_result.positionIncrements.size());
        rArm_JntTraj[i].VelocityTraj.resize(temp_result.positionIncrements.size());
        all_trajLen_r[i] = temp_result.positionIncrements.size();

        for(int j = 0; j < temp_result.positionIncrements.size(); j++){
            rArm_JntTraj[i].positionIncrements[j] = temp_result.positionIncrements[j] + rArm_stPos[i];
            rArm_JntTraj[i].VelocityTraj[j] = temp_result.VelocityTraj[j];
        }
    }

    //提取最长的轨迹以计算总输出长度
    temp_times = all_trajLen_l[0];
    for(int i = 1; i < LARMDOF; i++){
        if(all_trajLen_l[i] > temp_times){
            temp_times = all_trajLen_l[i];
        }
    }
    for(int i = 0; i < RARMDOF; i++){
        if(all_trajLen_r[i] > temp_times){
            temp_times = all_trajLen_r[i];
        }
    }
    temp_times = temp_times + 50;
}

//输出函数
TrajCtlOutp JointPos_Init::retZero_trajOutp(){
    TrajCtlOutp mRes;
    mRes.ifOutpReachEnd = false;
    mRes.lArm_desPos.assign(LARMDOF, 0.0);
    mRes.lArm_desVel.assign(LARMDOF, 0.0);
    mRes.rArm_desPos.assign(RARMDOF, 0.0);
    mRes.rArm_desVel.assign(RARMDOF, 0.0);

    if(trajLen < temp_times){
        for(int i = 0; i < LARMDOF; i++){
            if(trajLen < lArm_JntTraj[i].positionIncrements.size()){
                mRes.lArm_desPos[i] = lArm_JntTraj[i].positionIncrements[trajLen];
                mRes.lArm_desVel[i] = lArm_JntTraj[i].VelocityTraj[trajLen];
            }else{
                mRes.lArm_desPos[i] = lArm_zeroPos[i];
                mRes.lArm_desVel[i] = 0.0;
            }
        }

        for(int i = 0; i < RARMDOF; i++){
            if(trajLen < rArm_JntTraj[i].positionIncrements.size()){
                mRes.rArm_desPos[i] = rArm_JntTraj[i].positionIncrements[trajLen];
                mRes.rArm_desVel[i] = rArm_JntTraj[i].VelocityTraj[trajLen];
            }else{
                mRes.rArm_desPos[i] = rArm_zeroPos[i];
                mRes.rArm_desVel[i] = 0.0;
            }
        }
    }else{
        for(int i = 0; i < LARMDOF; i++){
            mRes.lArm_desPos[i] = lArm_zeroPos[i];
            mRes.lArm_desVel[i] = 0.0;
        }
        for(int i = 0; i < RARMDOF; i++){
            mRes.rArm_desPos[i] = rArm_zeroPos[i];
            mRes.rArm_desVel[i] = 0.0;
        }
        mRes.ifOutpReachEnd = true;
    }
    mRes.lArm_Kp = lArm_defaultKp;
    mRes.lArm_Kd = lArm_defaultKd;
    mRes.rArm_Kp = rArm_defaultKp;
    mRes.rArm_Kd = rArm_defaultKd;
    trajLen += 1;

    return mRes;
}

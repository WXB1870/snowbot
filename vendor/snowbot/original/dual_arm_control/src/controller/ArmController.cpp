#include "controller/ArmController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

// 读取文件并解析 package:// 路径
static std::string loadAndResolveURDF(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Cannot open URDF: " << filepath << std::endl;
        return "";
    }
    std::stringstream buf;
    buf << f.rdbuf();
    std::string content = buf.str();

    // 替换 package://xxx/ → file://实际路径/
    const std::string prefix = "package://";
    size_t pos = 0;
    while ((pos = content.find(prefix, pos)) != std::string::npos) {
        size_t slash = content.find('/', pos + prefix.length());
        std::string pkg = content.substr(pos + prefix.length(), slash - pos - prefix.length());
        try {
            std::string share = ament_index_cpp::get_package_share_directory(pkg);
            content.replace(pos, slash - pos, "file://" + share);
        } catch (...) {
            pos++;  // 找不到包，跳过
        }
    }
    return content;
}


//======================================================================
/*一级控制算法*/

std::vector<double> dbg_reserve_extern(14, 0.0);
std::vector<double> dbg_reserve_extern_2(14, 0.0);
std::vector<double> dbg_reserve_extern_3(14, 0.0);

//--------------------插补类：--------------------

//一次插补
TrajResult runiner::dPos_queue_stabTime_single(float mposDes, float mTrajTime)
{
    //圆整时间
    int tempTimes = std::round(mTrajTime/this->control_T);
    if(tempTimes < 1){
        tempTimes = 1;
    }
    float trajTime_round = this->control_T * tempTimes;

    //计算位置增量与速度
    float derPos = mposDes / tempTimes;
    float vel = mposDes / trajTime_round;

    //装入结构体
    TrajResult mRes;
    float tempPos = 0;
    for(int i = 0; i < tempTimes; i++){
    //for(int i = 0; i < 1; i++){
        
        tempPos = tempPos + derPos;
        mRes.positionIncrements.push_back(tempPos);
        mRes.VelocityTraj.push_back(vel);
    }

    //记录最后一轮的速度
    this->lastTempPosOutp = mRes.positionIncrements.back();
    this->lastTempVelOutp = mRes.VelocityTraj.back();
    return mRes;
}

//二次插补
TrajResult runiner::dPos_queue_stabTime_double(float mposDes, float velSt, float mtrajTime)
{
    //注意：输入量均为弧度制
    float posDes_half;
    float trajTime;
    //计算半切轨迹
    posDes_half = mposDes;
    trajTime = mtrajTime;

    //计算半切轨迹
    //核心——保证始末加速度为0（或近似0），末位置恰好达到目标位置
    float a = 12 * (velSt*trajTime - posDes_half) / std::pow(trajTime, 4);
    float b = a * trajTime;

    //据此生成增量轨迹
    std::vector<float> mResTraj;
    std::vector<float> mVelTraj;
    int m_tempLen = std::floor(trajTime / control_T);
    m_tempLen = std::max(1, m_tempLen); 

    float temp_T = this->control_T;
    for(int i = 0; i < m_tempLen; i++){
        float temp_pos = std::pow(temp_T, 4)*a*1.0/12 - std::pow(temp_T, 3)*b*1.0/6 + velSt*temp_T;
        float temp_vel = a * std::pow(temp_T, 3) - b * std::pow(temp_T, 2) / 2 + velSt;
        mResTraj.push_back(temp_pos);
        //mVelTraj.push_back(temp_vel);
        mVelTraj.push_back(0.0f);
        temp_T += control_T;
    }

    TrajResult mRes;
    mRes.positionIncrements = mResTraj;
    mRes.VelocityTraj = mVelTraj;

    this->lastTempPosOutp = mRes.positionIncrements.back();
    this->lastTempVelOutp = mRes.VelocityTraj.back();

    //return mRes_Interpolated;
    return mRes;
}

//点位插补
TrajResult runiner::dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase)
{
    //轨迹一半的时间用来加速
    //由曲线推到可得，当加速过程时间占到时间的一半时，按照钟型曲线加速过程和匀速过程走过距离比例为1：2

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

    this->lastTempPosOutp = mRes.positionIncrements.back();
    this->lastTempVelOutp = mRes.VelocityTraj.back();

    return mRes;
}

std::pair<float, float> runiner::lastPosVelGet(float posSt)
{
    float posOutpCor = this->lastTempPosOutp + posSt;
    std::pair<float, float> mRes = {posOutpCor, this->lastTempVelOutp};
    return mRes;
}

//--------------------力补偿类：--------------------
/**
double mGravCompInte[ALLJNTNUM] = {1.2,   1.4,   1.0,   1.4,   2.2,   2.2,   2.2, 
                                   1.6,   1.4,   1.0,   1.4,   2.2,   2.2,   2.2};
/**/
double mGravCompInte[ALLJNTNUM] = {2.4,   2.4,   2.2,   2.4,   2.2,   2.2,   2.2, 
                                   2.4,   2.4,   2.2,   2.4,   2.2,   2.2,   2.2};
double mCoriousCompInte[ALLJNTNUM] = {2.0,   2.0,   2.2,   2.2,   2.2,   2.2,   2.2, 
                                      2.0,   2.0,   2.2,   2.2,   2.2,   2.2,   2.2};
double mInertiaCompInte[ALLJNTNUM] = {2.0,   2.0,   2.2,   2.2,   2.2,   2.2,   2.2, 
                                      2.0,   2.0,   2.2,   2.2,   2.2,   2.2,   2.2};

//摩擦补偿参数
float lArm_fs[SINGLEARMDOF] = {0.07, 0.07, 0.07, 0.07, 0.07, 0.07, 0.07};
float lArm_fc[SINGLEARMDOF] = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
float lArm_fstic[SINGLEARMDOF] = {0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03};
float lArm_vs[SINGLEARMDOF] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
float lArm_gama[SINGLEARMDOF] = {1, 1, 1, 1, 1, 1, 1};
float lArm_JntT0[SINGLEARMDOF] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
int ifUseLeftFricComp[SINGLEARMDOF] = {1, 1, 1, 1, 1, 1, 1};

float rArm_fs[SINGLEARMDOF] = {0.07, 0.07, 0.07, 0.07, 0.07, 0.07, 0.07};
float rArm_fc[SINGLEARMDOF] = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
float rArm_fstic[SINGLEARMDOF] = {0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03};
float rArm_vs[SINGLEARMDOF] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
float rArm_gama[SINGLEARMDOF] = {1, 1, 1, 1, 1, 1, 1};
float rArm_JntT0[SINGLEARMDOF] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
int ifUseRightFricComp[SINGLEARMDOF] = {1, 1, 1, 1, 1, 1, 1};

//电流-力矩常数，用于将反馈电流转化成力矩量
float Kt_lArm[SINGLEARMDOF] = {1, 1, 1, 1, 1, 1, 1};     //左臂的电流常数
float Kt_rArm[SINGLEARMDOF] = {1, 1, 1, 1, 1, 1, 1};     //右臂的电流常数

//低通滤波器参数
//暂时采用一阶低通滤波器处理传感器返回的数据
double all_sampRate_posInp_lArm[SINGLEARMDOF] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};      //左臂的采样率
double all_cutoffFreq_posInp_lArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //左臂时间常数
double all_sampRate_velInp_lArm[SINGLEARMDOF] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};      //左臂的采样率
double all_cutoffFreq_velInp_lArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //左臂时间常数
double all_sampRate_TorInp_lArm[SINGLEARMDOF] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};      //左臂的采样率
double all_cutoffFreq_TorInp_lArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //左臂时间常数
double all_sampRate_TorOutp_lArm[SINGLEARMDOF] = {3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0};      //左臂的采样率
double all_cutoffFreq_TorOutp_lArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //左臂时间常数

double all_sampRate_posInp_rArm[SINGLEARMDOF] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};      //右臂的采样率
double all_cutoffFreq_posInp_rArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //右臂时间常数
double all_sampRate_velInp_rArm[SINGLEARMDOF] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};      //右臂的采样率
double all_cutoffFreq_velInp_rArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //右臂时间常数
double all_sampRate_TorInp_rArm[SINGLEARMDOF] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};      //右臂的采样率
double all_cutoffFreq_TorInp_rArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //右臂时间常数
double all_sampRate_TorOutp_rArm[SINGLEARMDOF] = {3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0};      //右臂的采样率
double all_cutoffFreq_TorOutp_rArm[SINGLEARMDOF] = {20, 20, 20, 20, 20, 20, 20};     //右臂时间常数


std::vector<double> Tor2Iq(float Kt[], std::vector<double> Tor, int Len)
{
    //输入量：Kt -- 电流常数； Tor -- 力矩； Len -- 数组长度，注意长度的匹配，不匹配就报错
    //力矩转化成电流
    std::vector<double> mres;
    for(int i = 0; i < Len; i++){
        mres.push_back(Tor.at(i)/Kt[i]);
    }
    return mres;
}

std::vector<double> Iq2Tor(float Kt[], std::vector<double> Tor, int Len)
{
    //输入量：Kt -- 电流常数； Tor -- 力矩； Len -- 数组长度，注意长度的匹配，不匹配就报错
    //力矩转化成电流
    std::vector<double> mres;
    for(int i = 0; i < Len; i++){
        mres.push_back(Tor.at(i) * Kt[i]);
    }
    return mres;
}

void printAllJointsInfo_old(const pinocchio::Model& model) {
    std::cout << "\n========== 模型关节信息 ==========" << std::endl;
    std::cout << "模型总关节数 (model.njoints): " << model.njoints << std::endl;
    std::cout << "配置空间维度 (model.nq): " << model.nq << std::endl;
    std::cout << "速度空间维度 (model.nv): " << model.nv << std::endl;
    std::cout << "\n关节详细信息：" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const std::string& joint_name = model.names[i];
        const pinocchio::JointModel& joint = model.joints[i];
        
        std::cout << "关节索引: " << i << std::endl;
        std::cout << "  名称: " << joint_name << std::endl;
        std::cout << "  关节类型: " << joint.shortname() << std::endl;
        std::cout << "  父关节索引: " << model.parents[i] << std::endl;
        
        // 获取关节类型信息
        
        std::cout << "  配置维度 (nq): " << joint.nq() << std::endl;
        std::cout << "  速度维度 (nv): " << joint.nv() << std::endl;
        
        // 输出关节限制信息（如果存在）
        if (i < model.upperPositionLimit.size() && i < model.lowerPositionLimit.size()) {
            if (model.upperPositionLimit[i] != std::numeric_limits<double>::max() ||
                model.lowerPositionLimit[i] != -std::numeric_limits<double>::max()) {
                std::cout << "  位置限制: [" << model.lowerPositionLimit[i] 
                         << ", " << model.upperPositionLimit[i] << "]" << std::endl;
            }
        }
        
        std::cout << "-------------------------------------------------------" << std::endl;
    }
    
    std::cout << "=======================================================" << std::endl;
}
void printAllJointsInfo(const pinocchio::Model& model) {
    std::cout << "\n========== 模型关节信息 ==========" << std::endl;
    std::cout << "模型总关节数 (model.njoints): " << model.njoints << std::endl;
    std::cout << "配置空间维度 (model.nq): " << model.nq << std::endl;
    std::cout << "速度空间维度 (model.nv): " << model.nv << std::endl;
    
    std::cout << "\n========== 配置空间映射 ==========" << std::endl;
    
    // 创建一个向量来跟踪配置索引
    int current_nq_index = 0;  // 当前配置向量的起始索引
    int current_nv_index = 0;  // 当前速度向量的起始索引
    
    std::cout << "\n关节详细信息：" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const std::string& joint_name = model.names[i];
        const pinocchio::JointModel& joint = model.joints[i];
        int joint_nq = joint.nq();
        int joint_nv = joint.nv();
        
        std::cout << "关节索引: " << i << std::endl;
        std::cout << "  名称: " << joint_name << std::endl;
        std::cout << "  关节类型: " << joint.shortname() << std::endl;
        std::cout << "  父关节索引: " << model.parents[i] << std::endl;
        
        // 打印配置空间映射信息
        if (joint_nq > 0) {
            std::cout << "  配置维度 (nq): " << joint_nq << std::endl;
            std::cout << "  在q向量中的索引: [" << current_nq_index 
                     << " 到 " << current_nq_index + joint_nq - 1 << "]" << std::endl;
            
            // 解释每个配置变量的含义
            std::cout << "  配置变量含义: ";
            if (joint.shortname().find("Revolute") != std::string::npos) {
                std::cout << "旋转角度 (弧度)" << std::endl;
            } else if (joint.shortname().find("Prismatic") != std::string::npos) {
                std::cout << "平移距离 (米)" << std::endl;
            } else if (joint.shortname().find("FreeFlyer") != std::string::npos) {
                std::cout << "位置(3) + 四元数姿态(4)" << std::endl;
            } else if (joint.shortname().find("Spherical") != std::string::npos) {
                std::cout << "四元数姿态 (x,y,z,w)" << std::endl;
            } else if (joint.shortname().find("Planar") != std::string::npos) {
                std::cout << "平面位置(2) + 旋转角度(1)" << std::endl;
            } else {
                std::cout << "未知类型" << std::endl;
            }
        } else {
            std::cout << "  配置维度 (nq): 0 (固定关节)" << std::endl;
        }
        
        if (joint_nv > 0) {
            std::cout << "  速度维度 (nv): " << joint_nv << std::endl;
            std::cout << "  在v向量中的索引: [" << current_nv_index 
                     << " 到 " << current_nv_index + joint_nv - 1 << "]" << std::endl;
        } else {
            std::cout << "  速度维度 (nv): 0 (固定关节)" << std::endl;
        }
        
        // 输出关节限制信息
        if (i < model.upperPositionLimit.size() && i < model.lowerPositionLimit.size() && joint_nq > 0) {
            bool has_limits = false;
            for (int j = 0; j < joint_nq; ++j) {
                int idx = current_nq_index + j;
                if (idx < model.lowerPositionLimit.size() && 
                    idx < model.upperPositionLimit.size()) {
                    double lower = model.lowerPositionLimit[idx];
                    double upper = model.upperPositionLimit[idx];
                    if (upper > lower) {  // 有效的限制
                        has_limits = true;
                        std::cout << "  配置变量 " << j << " 限制: [" 
                                 << lower << ", " << upper << "]" << std::endl;
                    }
                }
            }
            if (!has_limits) {
                std::cout << "  位置限制: 无限制" << std::endl;
            }
        }
        
        // 输出关节ID映射（如果存在）
        if (model.joints[i].id() >= 0) {
            std::cout << "  关节ID: " << model.joints[i].id() << std::endl;
        }
        
        // 输出坐标系信息
        if (i < model.frames.size()) {
            for (size_t frame_idx = 0; frame_idx < model.frames.size(); ++frame_idx) {
                if (model.frames[frame_idx].parentJoint == i) {
                    std::cout << "  附加框架: " << model.frames[frame_idx].name 
                             << " (类型: " << model.frames[frame_idx].type << ")" << std::endl;
                }
            }
        }
        
        std::cout << "-------------------------------------------------------" << std::endl;
        
        // 更新索引
        current_nq_index += joint_nq;
        current_nv_index += joint_nv;
    }
    
    // 验证总数
    std::cout << "\n========== 配置空间摘要 ==========" << std::endl;
    std::cout << "总配置变量数 (应等于nq): " << current_nq_index 
              << " (model.nq = " << model.nq << ")" << std::endl;
    std::cout << "总速度变量数 (应等于nv): " << current_nv_index 
              << " (model.nv = " << model.nv << ")" << std::endl;
    
    if (current_nq_index != model.nq) {
        std::cerr << "警告：计算的总配置变量数与model.nq不匹配！" << std::endl;
    }
    if (current_nv_index != model.nv) {
        std::cerr << "警告：计算的总速度变量数与model.nv不匹配！" << std::endl;
    }
    
    // 添加完整的配置向量结构说明
    std::cout << "\n========== 配置向量q的结构 ==========" << std::endl;
    current_nq_index = 0;
    current_nv_index = 0;
    
    std::cout << "q向量 (size = " << model.nq << "):" << std::endl;
    std::cout << "索引 | 关节名称 | 类型 | 变量含义" << std::endl;
    std::cout << "-----|----------|------|----------" << std::endl;
    
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const pinocchio::JointModel& joint = model.joints[i];
        int joint_nq = joint.nq();
        
        if (joint_nq > 0) {
            for (int j = 0; j < joint_nq; ++j) {
                std::cout << std::setw(3) << current_nq_index << "  | "
                         << std::setw(10) << model.names[i] << " | "
                         << std::setw(6) << joint.shortname();
                
                if (joint.shortname().find("Revolute") != std::string::npos) {
                    std::cout << " | 角度 θ" << (j+1) << " (rad)" << std::endl;
                } else if (joint.shortname().find("Prismatic") != std::string::npos) {
                    std::cout << " | 距离 d" << (j+1) << " (m)" << std::endl;
                } else if (joint.shortname().find("FreeFlyer") != std::string::npos) {
                    if (j < 3) std::cout << " | 位置 p" << (j+1) << " (m)" << std::endl;
                    else std::cout << " | 姿态四元数 q" << (j-2) << std::endl;
                } else {
                    std::cout << " | 变量 " << (j+1) << std::endl;
                }
                
                current_nq_index++;
            }
        }
    }
    
    std::cout << "\nv向量 (size = " << model.nv << "):" << std::endl;
    std::cout << "索引 | 关节名称 | 类型 | 变量含义" << std::endl;
    std::cout << "-----|----------|------|----------" << std::endl;
    
    current_nv_index = 0;
    for (pinocchio::JointIndex i = 0; i < model.njoints; ++i) {
        const pinocchio::JointModel& joint = model.joints[i];
        int joint_nv = joint.nv();
        
        if (joint_nv > 0) {
            for (int j = 0; j < joint_nv; ++j) {
                std::cout << std::setw(3) << current_nv_index << "  | "
                         << std::setw(10) << model.names[i] << " | "
                         << std::setw(6) << joint.shortname();
                
                if (joint.shortname().find("Revolute") != std::string::npos) {
                    std::cout << " | 角速度 ω" << (j+1) << " (rad/s)" << std::endl;
                } else if (joint.shortname().find("Prismatic") != std::string::npos) {
                    std::cout << " | 速度 v" << (j+1) << " (m/s)" << std::endl;
                } else if (joint.shortname().find("FreeFlyer") != std::string::npos) {
                    if (j < 3) std::cout << " | 线速度 v" << (j+1) << " (m/s)" << std::endl;
                    else std::cout << " | 角速度 ω" << (j-2) << " (rad/s)" << std::endl;
                } else {
                    std::cout << " | 速度 " << (j+1) << std::endl;
                }
                
                current_nv_index++;
            }
        }
    }
    
    std::cout << "=======================================================" << std::endl;
}

//初始化函数
bool ForceComp::init(KDL::JntArray stPos_lArm, int stJntID_lArm, 
                       KDL::JntArray stPos_rArm, int stJntID_rArm, 
                       int urdf_choose, int ArmNum)
{
    //销毁旧有的模型
    if (model_.njoints > 0) {
        // 清除现有的模型和数据
        model_ = pinocchio::Model();
        data_ = pinocchio::Data();
    }

    //加载urdf模型
    std::string urdf_xml = loadAndResolveURDF(path_options[urdf_choose]);
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);

    // 锁定非手臂关节 (身体/头部/夹爪)，构建简化模型
    {
        std::vector<std::string> locked_names = {
            "body_joint_1", "body_joint_2",
            "head_joint_1", "head_joint_2",
            "arm_l_joint_8", "arm_r_joint_8"
        };
        std::vector<pinocchio::JointIndex> locked_ids;
        for (const auto& name : locked_names) {
            if (model_.existJointName(name)) {
                locked_ids.push_back(model_.getJointId(name));
            }
        }
        if (!locked_ids.empty()) {
            Eigen::VectorXd ref_config = Eigen::VectorXd::Zero(model_.nq);
            pinocchio::Model reduced_model;
            pinocchio::buildReducedModel(model_, locked_ids, ref_config, reduced_model);
            model_ = reduced_model;
        }
    }

    data_ = pinocchio::Data(model_);

    grav_compInte.resize(JntNum);
    corious_compInte.resize(JntNum);
    interia_compInte.resize(JntNum);
    for(int i = 0; i < ALLJNTNUM; i++){
        grav_compInte[i] = mGravCompInte[i];
        corious_compInte[i] = mCoriousCompInte[i];
        interia_compInte[i] = mInertiaCompInte[i];
    }

    //-----调试-----

    /**
    // 调用打印关节信息的函数
    printAllJointsInfo(model_);

    // 打印更多的模型摘要信息
    std::cout << "\n========== 模型摘要信息 ==========" << std::endl;
    std::cout << "模型名称: " << model_.name << std::endl;
    std::cout << "总关节数: " << model_.njoints << std::endl;
    std::cout << "配置空间维度 (nq): " << model_.nq << std::endl;
    std::cout << "速度空间维度 (nv): " << model_.nv << std::endl;
    std::cout << "运动链根节点: " << model_.names[model_.getFrameId("root_joint")] 
              << " (索引: " << model_.getFrameId("root_joint") << ")" << std::endl;
    std::cout << "====================================\n" << std::endl;
    //-------------
    /**/

    //初始化关节状态：
    //注意：关节分为三类：单向平移关节、旋转关节、浮动基关节。浮动基关节，一个关节要占用六个自由度，可以等效成多个虚拟关节
    //多自由度关节也可等效成多个单自由度虚拟关节并分析
    q_ = Eigen::VectorXd::Zero(model_.nq);      //位置向量配置
    int q_len = ArmNum * SINGLEARMDOF;          //配置关节总数量
    if(q_len != model_.nq){
        std::cerr << "错误：关节数量不匹配！目标：" << ArmNum * SINGLEARMDOF 
                  << " 或 " << ArmNum * SINGLEARMDOF + 6 << "（浮动基座），获取：" << model_.nq << std::endl;
        return false;
    }

    //装入关节角度
    int temp_1 = 0;
    for(int i = 0; i < SINGLEARMDOF; i++){
        q_(temp_1) = stPos_lArm(i);
        temp_1 += 1;
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        q_(temp_1) = stPos_rArm(i);
        temp_1 += 1; 
    }
    v_ = Eigen::VectorXd::Zero(model_.nv);      //初始化关节速度为0
    a_ = Eigen::VectorXd::Zero(model_.nv);      //初始化关节加速度为0
    v_lst_ = Eigen::VectorXd::Zero(model_.nv);  //初始化上一轮的关节速度为0

    pinocchio::forwardKinematics(model_, data_, q_);
    //pinocchio::forwardKinematics(model_, data_, q_, v_);

    //初始化加速度滤波器
    accFtr.resize(q_len);
    for(int i = 0; i < q_len; i++){
        accFtr[i].init(sampRate[i], cutoffFreq[i]);
    }

    //printModelInfo();
    return true;
}

//补偿函数
std::vector<double> ForceComp::gravComp(std::vector<double> lArm_pos, std::vector<double> lArm_vel,
                         std::vector<double> rArm_pos, std::vector<double> rArm_vel
                          )
{
    //注意：机器人关节自由度的协调

    //处理状态矩阵
    //位置量处理
    float q_comp_l[SINGLEARMDOF]={0,0,0,0,0,0,0};
    float q_comp_r[SINGLEARMDOF]={0,0,0,0,0,0,0};
    int temp_1 = 0;
    for(int i = 0; i < SINGLEARMDOF; i++){
        q_(temp_1) = (lArm_pos[i] - q_comp_l[i]) * inte_inp[temp_1];
        temp_1 += 1;
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        q_(temp_1) = (rArm_pos[i] - q_comp_r[i]) * inte_inp[temp_1];
        temp_1 += 1;
    }

    Eigen::VectorXd tau = pinocchio::computeGeneralizedGravity(model_, data_, q_);

    //数据类型转化并输出
    std::vector<double> tau_vector(tau.size());
    for (int i = 0; i < tau.size(); ++i) {
        tau_vector[i] = tau[i] / grav_compInte[i] * inte_outp[i];

    }
    return tau_vector;
}

//惯性综合补偿(预留空函数作为接口)(惯性力+科氏力)
std::vector<double> ForceComp::InteriaMulComp(
    std::vector<double> lArm_pos, std::vector<double> lArm_vel,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel
){
    //速度量处理
    int temp_2 = 0;
    for(int i = 0; i < SINGLEARMDOF; i++){
        v_(temp_2) = lArm_vel[i] * inte_inp[temp_2];
        temp_2 += 1;
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        v_(temp_2) = rArm_vel[i] * inte_inp[temp_2];
        temp_2 += 1;
    }

    //加速度处理
    int temp_3 = 0;
    for(int i = 0; i < tempLen_1; i++){
        a_(temp_3) = (v_(temp_3) - v_lst_(temp_3)) / temp_T;
        //a_(temp_3) = 0;

        float temp_a = (v_(temp_3) - v_lst_(temp_3)) / temp_T;
       // a_(temp_3) = accFtr[temp_3].filter(temp_a);

        v_lst_(temp_3) = v_(temp_3);
        temp_3 += 1;
    }

    //科氏力项处理：
    Eigen::VectorXd coriolis_only = pinocchio::computeCoriolisMatrix(model_, data_, q_, v_) * v_;

    //惯性力项处理：
    Eigen::MatrixXd mass_matrix = pinocchio::crba(model_, data_, q_);
    mass_matrix.triangularView<Eigen::StrictlyLower>() = mass_matrix.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::VectorXd inertia_only = mass_matrix * a_;

    //###################################debug
    /**/
    for(int i = 0; i < coriolis_only.size(); i++){
        //dbg_reserve_extern_2[i] = coriolis_only[i];
        dbg_reserve_extern_2[i] = inertia_only[i];
    }/**/

    //科氏力与惯性力叠加输出：
    std::vector<double> tau_vec;
    tau_vec.resize(coriolis_only.size());
    for(int i = 0; i < tau_vec.size(); i++){
        tau_vec[i] = coriolis_only[i]/corious_compInte[i]*inte_outp[i] + inertia_only[i]/interia_compInte[i]*inte_outp[i];
        //tau_vec[i] = coriolis_only[i]/corious_compInte[i]*inte_outp[i];
    }

    //返回值，暂定为0,等待其他代码debug结束后
    std::vector<double> mOutp;
    mOutp.assign(ALLJNTNUM, 0.0);
    //return mOutp;
    return tau_vec;

}

//摩擦补偿
void ForceComp::FricCompInit()
{
    for(int i = 0; i < SINGLEARMDOF; i++){
        Kernel::StribeckFriction mFricComp;
        mFricComp.StribeckModelInit(lArm_fs[i], lArm_fc[i], lArm_fstic[i], lArm_vs[i], lArm_gama[i], lArm_JntT0[i]);
        allFricComp.push_back(mFricComp);
    }
    for(int i = 0; i < SINGLEARMDOF; i++){
        Kernel::StribeckFriction mFricComp;
        mFricComp.StribeckModelInit(rArm_fs[i], rArm_fc[i], rArm_fstic[i], rArm_vs[i], rArm_gama[i], rArm_JntT0[i]);
        allFricComp.push_back(mFricComp);
    }
}

std::vector<double> ForceComp::FricComp(std::vector<double> lArm_pos, std::vector<double> lArm_vel, std::vector<double> lArm_Tor,
                      std::vector<double> rArm_pos, std::vector<double> rArm_vel, std::vector<double> rArm_Tor
        )
{
    std::vector<double> mRes;

    for(int i = 0; i < SINGLEARMDOF; i++){
        float temp_Tinp = static_cast<float>(lArm_Tor.at(i));
        float temp_velInp = static_cast<float>(lArm_vel.at(i));
        mRes.push_back(allFricComp.at(i).StribeckFricCal(temp_Tinp, temp_velInp));
    }

    for(int i = 0; i < SINGLEARMDOF; i++){
        float temp_Tinp = static_cast<float>(rArm_Tor.at(i));
        float temp_velInp = static_cast<float>(rArm_vel.at(i));
        mRes.push_back(allFricComp.at(i + SINGLEARMDOF).StribeckFricCal(temp_Tinp, temp_velInp));
    }

    return mRes;
}

void ForceComp::printModelInfo() {
    std::cout << "Pinocchio Model Information:" << std::endl;
    std::cout << "Number of joints: " << model_.njoints << std::endl;
    std::cout << "Number of frames: " << model_.nframes << std::endl;
    std::cout << "Joint positions (q) size: " << q_.size() << std::endl;
    std::cout << "Joint velocities (v) size: " << v_.size() << std::endl;
    std::cout << "Joint accelerations (a) size: " << a_.size() << std::endl;
    std::cout << "Previous velocities (v_lst) size: " << v_lst_.size() << std::endl;

    // 打印关节名称
    std::cout << "\nJoint names:" << std::endl;
    for (size_t i = 0; i < model_.names.size(); ++i) {
        std::cout << "  Joint " << i << ": " << model_.names[i] << std::endl;
    }

    // 打印惯性参数（第一个关节）
    std::cout << "\nInertia parameters (first joint):" << std::endl;
    if (!model_.inertias.empty()) {
        std::cout << "  Mass: " << model_.inertias[0].mass() << std::endl;
        std::cout << "  Inertia matrix (local frame):\n" 
                  << model_.inertias[0].inertia().matrix() << std::endl;
        std::cout << "  CoM offset (local frame): " 
                  << model_.inertias[0].lever().transpose() << std::endl;
    }

    // 打印关节到基座的变换矩阵（data_.oMi）
    std::cout << "\nJoint transformation matrices (oMi) w.r.t. base:" << std::endl;
    for (size_t i = 0; i < static_cast<size_t>(model_.njoints); ++i) {
        std::cout << "  Joint " << i << " (" << model_.names[i] << "):\n" 
                  << data_.oMi[i].rotation() << " "<<data_.oMi[i].translation().transpose() << std::endl;
    }

    // 打印关节质心在基座坐标系中的位置（手动计算）
    std::cout << "\nJoint CoM positions in base frame:" << std::endl;
    for (size_t i = 0; i < static_cast<size_t>(model_.njoints); ++i) {
        Eigen::Vector3d com_pos = data_.oMi[i].act(model_.inertias[i].lever());
        std::cout << "  Joint " << i << " (" << model_.names[i] << "): " 
                  << com_pos.transpose() << std::endl;
    }
}



//======================================================================
/*单关节点位控制代码
-------------------------------------------
  单关节点位控制代码仅包含定时轨迹控制代码
*/

//静止关节代码
void jnt_stabOutp::init(float mJntDeg){
    this->JntDeg = mJntDeg;
}
ctlOutp jnt_stabOutp::StabOutp()
{
    //输出计算
    ctlOutp mOutp;
    mOutp.posDes = this->JntDeg;
    mOutp.velDes = 0.0f;

    mOutp.grav_TorDes = 0.0f;
    mOutp.interia_TorDes = 0.0f;
    mOutp.feedforward_TorDes = 0.0f;

    mOutp.ifReachEnd = true;

    return mOutp;  
}

//运动关节代码
void moveJntCtl_stabTime::init_stabTime(runinMsgDotMod mCtlMsg, runinStabMsg mstabMsg)
{
    clear();

    this->startPos = mCtlMsg.mPosSt;
    this->endPos = mCtlMsg.mPosDes;
    this->ifUseStab = mstabMsg.ifUseStab;

    this->ifUseStab = mstabMsg.ifUseStab;       //是否在初始位置停留
    this->stabTimes = mstabMsg.des_stabTimes;   //在目标位置停留时间
    float mstabRanges = mstabMsg.start_stabTimes;     //在初始位置停留时间

    float dertaPos = mCtlMsg.mPosDes - mCtlMsg.mPosSt;
    TrajResult res_traj = mlocater.dPos_queue_stabTime_dot(dertaPos, mCtlMsg.mTrajTime, mCtlMsg.accTimeRateBase);   //计算出的全切轨迹——从A点到达B点的轨迹
    std::vector<float> derPos_traj = res_traj.positionIncrements;
    derPos_traj.push_back(dertaPos);
    std::vector<float> derPos_traj_bf;      //依据全切轨迹计算往复轨迹
    derPos_traj_bf = derPos_traj;   //先正序复制
    for(int i = 0; i < this->stabTimes; i++){
        derPos_traj_bf.push_back(dertaPos);
    }
    std::vector<float> traj_temp(derPos_traj.rbegin(), derPos_traj.rend());
    derPos_traj_bf.insert(derPos_traj_bf.end(), traj_temp.begin(), traj_temp.end());
    derPos_traj_bf.push_back(0);

    //计算位置、速度与力矩
    //先装入一定量的静止值
    if(this->ifUseStab){
        posTraj_stabTime.assign(mstabRanges, startPos);
        velTraj_stabTime.assign(mstabRanges, 0.0f);
        TorTraj_stabTime.assign(mstabRanges, 0.0f);
        posTraj_stabTime_half.assign(mstabRanges, startPos);
        velTraj_stabTime_half.assign(mstabRanges, 0.0f);
        TorTraj_stabTime_half.assign(mstabRanges, 0.0f);
    }
    posTraj_stabTime.push_back(startPos);
    velTraj_stabTime.push_back(0.0f);
    TorTraj_stabTime.push_back(0.0f);
    posTraj_stabTime_half.push_back(startPos);
    velTraj_stabTime_half.push_back(0.0f);
    TorTraj_stabTime_half.push_back(0.0f);

    //再开始装入计算结果
    int all_tempLen = derPos_traj.size();
    int all_tempLen_bf = derPos_traj_bf.size();
    float pos_lst = startPos;
    for(int i = 0; i < all_tempLen_bf; i++){
        float temp_pos = derPos_traj_bf.at(i) + startPos;
        float temp_vel = (temp_pos - pos_lst)/this->temp_T;
        posTraj_stabTime.push_back(temp_pos);
        velTraj_stabTime.push_back(temp_vel);
        pos_lst = temp_pos;

        TorTraj_stabTime.push_back(0.0f);
    }

    float pos_lst_2 = startPos;
    for(int i = 0; i < all_tempLen; i++){
        float temp_pos_2 = derPos_traj.at(i) + startPos;
        float temp_vel_2 = (temp_pos_2 - pos_lst_2)/this->temp_T;
        posTraj_stabTime_half.push_back(temp_pos_2);
        velTraj_stabTime_half.push_back(temp_vel_2);
        pos_lst_2 = temp_pos_2;

        TorTraj_stabTime_half.push_back(0.0f);
    }
    TrajLen = posTraj_stabTime_half.size();
}

ctlOutp moveJntCtl_stabTime::desPos_get_stabTime()
{
    ctlOutp mOutp;
    mOutp.posDes = this->posTraj_stabTime.at(this->TempTimes);
    mOutp.velDes = this->velTraj_stabTime.at(this->TempTimes);

    //所有目标力矩均用占位符填充
    mOutp.grav_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
    mOutp.feedforward_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
    mOutp.interia_TorDes = this->TorTraj_stabTime.at(this->TempTimes);

    mOutp.ifReachEnd = false;

    this->TempTimes += 1;
    int tempTimeMaxSize = this->posTraj_stabTime.size();
    if(tempTimeMaxSize < 1){
        tempTimeMaxSize = 1;
    }
    if(this->TempTimes >= tempTimeMaxSize - 1){
        this->TempTimes = 0;
    }
    return mOutp;
}
ctlOutp moveJntCtl_stabTime::desPos_get_stabTimeStop()
{
    ctlOutp mOutp;
    if(TempTimes < this->posTraj_stabTime_half.size()){
        mOutp.posDes = this->posTraj_stabTime_half.at(this->TempTimes);
        mOutp.velDes = this->velTraj_stabTime_half.at(this->TempTimes);

        //力输出占位填充
        mOutp.grav_TorDes = this->TorTraj_stabTime_half.at(this->TempTimes);
        mOutp.interia_TorDes = this->TorTraj_stabTime_half.at(this->TempTimes);
        mOutp.feedforward_TorDes = this->TorTraj_stabTime_half.at(this->TempTimes);

        mOutp.ifReachEnd = false;
        if(TempTimes >= this->posTraj_stabTime_half.size()-1){
            mOutp.ifReachEnd = true;
        }
    }else{
        mOutp.posDes = this->endPos;
        mOutp.velDes = 0.0;

        mOutp.grav_TorDes = 0.0;
        mOutp.interia_TorDes = 0.0;
        mOutp.feedforward_TorDes = 0.0;

        mOutp.ifReachEnd = true;
    }

    if(this->TempTimes < 1e10){
        this->TempTimes += 1;
    }
    return mOutp;
}

void moveJntCtl_stabTime::init_stabTime_continue(runinMsg mCtlMsg)
{
    clear();

    //控制模式解包
    this->startPos = mCtlMsg.mPosSt;
    this->endPos = mCtlMsg.mPosDes;
    float derPos = this->endPos - this->startPos;
    this->posTraj_last = this->startPos;
    //计算轨迹
    TrajResult mRes;
    if(mCtlMsg.runinMod == 1){
        //插补模式1：线性插补
        mRes = mlocater.dPos_queue_stabTime_single(derPos, mCtlMsg.mTrajTime);
    }else{
        //插补模式2：二次插补
        mRes = mlocater.dPos_queue_stabTime_double(derPos, mCtlMsg.mVelSt, mCtlMsg.mTrajTime);
    }

    //装入计算结果
    for(int i = 0; i < mRes.positionIncrements.size(); i++){
        posTraj_stabTime.push_back(mRes.positionIncrements[i] + this->startPos);
        velTraj_stabTime.push_back(mRes.VelocityTraj[i]);
        posTraj_last = posTraj_stabTime[i];

        TorTraj_stabTime.push_back(0.0f);
    }
    TrajLen_Continue = mRes.positionIncrements.size();
}

ctlOutp moveJntCtl_stabTime::desPos_get_stab_Continue(){
    ctlOutp mOutp;

    if(TempTimes < this->posTraj_stabTime.size()){
        mOutp.posDes = this->posTraj_stabTime.at(this->TempTimes);
        mOutp.velDes = this->velTraj_stabTime.at(this->TempTimes);
        
        //占位填充
        mOutp.grav_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
        mOutp.interia_TorDes = this->TorTraj_stabTime.at(this->TempTimes);
        mOutp.feedforward_TorDes = this->TorTraj_stabTime.at(this->TempTimes);

        mOutp.ifReachEnd = false;
        if(TempTimes >= this->posTraj_stabTime.size()-1){
            mOutp.ifReachEnd = true;
        }
    }else{
        mOutp.posDes = this->endPos;
        //mOutp.posDes = this->posTraj_stabTime.at(*posTraj_stabTime.end());
        mOutp.velDes = 0.0;

        mOutp.grav_TorDes = 0.0;
        mOutp.interia_TorDes = 0.0;
        mOutp.feedforward_TorDes = 0.0;

        mOutp.ifReachEnd = true;
    }

    if(this->TempTimes < 1e5){
        this->TempTimes += 1;
    }
    return mOutp;
}

std::pair<float, float> moveJntCtl_stabTime::lastTempPosVel(){
    std::pair<float, float> mRes = mlocater.lastPosVelGet(this->startPos);
    return mRes;
}

//======================================================================
/*综合控制算法*/

float double2Float(double inp){
    float mres = static_cast<float>(inp);
    return mres;
}

/*针对机器人整体直接传输关节参数的求解代码封装*/

void JntTrackSol::init(KDL::JntArray posSt_lArm, KDL::JntArray posSt_rArm){
    topic_stream_ = false;
    topic_left_.assign(posSt_lArm.rows(), {});
    topic_right_.assign(posSt_rArm.rows(), {});
    for (unsigned i = 0; i < posSt_lArm.rows(); ++i)
        topic_left_[i].position = topic_left_[i].target = posSt_lArm(i);
    for (unsigned i = 0; i < posSt_rArm.rows(); ++i)
        topic_right_[i].position = topic_right_[i].target = posSt_rArm(i);
    posLst_lArm = posSt_lArm;
    posLst_rArm = posSt_rArm;
    
    velLst_lArm.resize(posSt_lArm.rows());
    velLst_rArm.resize(posSt_rArm.rows());
    for(int i = 0; i < posSt_lArm.rows(); i++){
        velLst_lArm(i) = 0.0f;
    }
    for(int i = 0; i < posSt_rArm.rows(); i++){
        velLst_rArm(i) = 0.0f;
    }

    lArmJntTrajSol.resize(posSt_lArm.rows());
    rArmJntTrajSol.resize(posSt_rArm.rows());
    lArmStab.resize(posSt_lArm.rows());
    rArmStab.resize(posSt_rArm.rows());

    lArmJntTorTrajSol.resize(posSt_lArm.rows());
    rArmJntTorTrajSol.resize(posSt_rArm.rows());
}

void JntTrackSol::JntTrajCal(
    //目标量
    KDL::JntArray posDes_lArm, KDL::JntArray posDes_rArm,
    std::vector<double> JntTorDes_l, std::vector<double> JntTorDes_r,

    //反馈量
    KDL::JntArray posRet_lArm, KDL::JntArray posRet_rArm,
    KDL::JntArray velRet_lArm, KDL::JntArray velRet_rArm,
    std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r,

    modCtl mod_ctl_msg,
    bool ifRedefineStab
){
    lArmTrajLen.clear();
    rArmTrajLen.clear();

    topic_stream_ = mod_ctl_msg.ifStreamingJointPosition;
    if (topic_stream_) {
        ifJntStab = mod_ctl_msg.ifJntStab;
        for (unsigned i = 0; i < posDes_lArm.rows(); ++i)
            topic_left_[i].target = posDes_lArm(i);
        for (unsigned i = 0; i < posDes_rArm.rows(); ++i)
            topic_right_[i].target = posDes_rArm(i);
        return; // Receiving a target must not reset the current output state.
    }

    //先获取初始角度与角速度输入值
    KDL::JntArray posInp_l, posInp_r;
    KDL::JntArray velInp_l, velInp_r;

    if(mod_ctl_msg.ifUseRet){
        //采用返回值作为上一轮的输入
        posInp_l = posRet_lArm;
        posInp_r = posRet_rArm;
        velInp_l = velRet_lArm;
        velInp_r = velRet_rArm;
    }else{
        //采用上一轮输出作为本轮输入
        posInp_l = posLst_lArm;
        posInp_r = posLst_rArm;
        velInp_l = velLst_lArm;
        velInp_r = velLst_rArm;
    }

    this->ifUseTemp = mod_ctl_msg.ifUseTemp;

    //设定静止控制参量：
    bool m_ifRedefineStab = ifFirstTemp || ifRedefineStab;
    ifFirstTemp = false;

    //更新输出控制参量
    ifJntStab = mod_ctl_msg.ifJntStab;

    //再计算轨迹
    //依据输入模式计算插补轨迹
    if(mod_ctl_msg.runinMod == 3){
        //点到点模式

        //先包装参量
        std::vector<runinMsgDotMod> runin_ctl_msg_dot_l;
        std::vector<runinMsgDotMod> runin_ctl_msg_dot_r;
        runin_ctl_msg_dot_l.resize(posRet_lArm.rows());
        runin_ctl_msg_dot_r.resize(posRet_rArm.rows());
        for(int i = 0; i < posRet_lArm.rows(); i++){
            runin_ctl_msg_dot_l[i].mPosSt = posInp_l(i);
            runin_ctl_msg_dot_l[i].mPosDes = posDes_lArm(i);
            runin_ctl_msg_dot_l[i].accTimeRateBase = mod_ctl_msg.accRateBase[0][i];
            runin_ctl_msg_dot_l[i].mTrajTime = mod_ctl_msg.TrajTime[0][i];
        }
        for(int i = 0; i < posRet_rArm.rows(); i++){
            runin_ctl_msg_dot_r[i].mPosSt = posInp_r(i);
            runin_ctl_msg_dot_r[i].mPosDes = posDes_rArm(i);
            runin_ctl_msg_dot_r[i].accTimeRateBase = mod_ctl_msg.accRateBase[1][i];
            runin_ctl_msg_dot_r[i].mTrajTime = mod_ctl_msg.TrajTime[1][i];
        }

        //再计算输出值
        for(int i = 0; i < lArmJntTrajSol.size(); i++){
            lArmJntTrajSol.at(i).init_stabTime(runin_ctl_msg_dot_l[i], mod_ctl_msg.m_stabMsg[0][i]);
            lArmTrajLen.push_back(lArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                lArmStab[i].init(runin_ctl_msg_dot_l[i].mPosSt);
            }
        }
        for(int i = 0; i < rArmJntTrajSol.size(); i++){
            rArmJntTrajSol.at(i).init_stabTime(runin_ctl_msg_dot_r[i], mod_ctl_msg.m_stabMsg[1][i]);
            rArmTrajLen.push_back(rArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                rArmStab[i].init(runin_ctl_msg_dot_r[i].mPosSt);
            }
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }else{
        //连续求解插补模式

        //先包装参量
        std::vector<runinMsg> runin_ctl_msg_l;
        std::vector<runinMsg> runin_ctl_msg_r;
        runin_ctl_msg_l.resize(posRet_lArm.rows());
        runin_ctl_msg_r.resize(posRet_rArm.rows());

        for(int i = 0; i < posRet_lArm.rows(); i++){
            runin_ctl_msg_l[i].mPosSt = posInp_l(i);
            runin_ctl_msg_l[i].mPosDes = posDes_lArm(i);
            runin_ctl_msg_l[i].mVelSt = velInp_l(i);
            runin_ctl_msg_l[i].mTrajTime = mod_ctl_msg.upperMonitor_time;
            runin_ctl_msg_l[i].mTrajTime = mod_ctl_msg.TrajTime[0][i];
            runin_ctl_msg_l[i].runinMod = mod_ctl_msg.runinMod;
        }
        for(int i = 0; i < posRet_rArm.rows(); i++){
            runin_ctl_msg_r[i].mPosSt = posInp_r(i);
            runin_ctl_msg_r[i].mPosDes = posDes_rArm(i);
            runin_ctl_msg_r[i].mVelSt = velInp_r(i);
            runin_ctl_msg_r[i].mTrajTime = mod_ctl_msg.upperMonitor_time;
            runin_ctl_msg_r[i].mTrajTime = mod_ctl_msg.TrajTime[1][i];
            runin_ctl_msg_r[i].runinMod = mod_ctl_msg.runinMod;
        }

        //再计算输出值
        for(int i = 0; i < lArmJntTrajSol.size(); i++){
            lArmJntTrajSol.at(i).init_stabTime_continue(runin_ctl_msg_l[i]);
            lArmTrajLen.push_back(lArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                lArmStab[i].init(runin_ctl_msg_l[i].mPosSt);
            }
        }
        for(int i = 0; i < rArmJntTrajSol.size(); i++){
            rArmJntTrajSol.at(i).init_stabTime_continue(runin_ctl_msg_r[i]);
            rArmTrajLen.push_back(rArmJntTrajSol.at(i).TrajLen);

            if(m_ifRedefineStab){
                rArmStab[i].init(runin_ctl_msg_r[i].mPosSt);
            }
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }

    /*  力前馈控制接口
    -------------------
    在这里添加力前馈控制函数
    这部分先空着，留着接口，以后做编辑
    */
    std::vector<double> TorFeedForward_l, TorFeedForward_r;
    TorFeedForward_l.assign(SINGLEARMDOF, 0.0);
    TorFeedForward_r.assign(SINGLEARMDOF, 0.0);

    if(mod_ctl_msg.runinMod == 3){
        //点到点控制 -- 
    }else{

    }

    //更新控制参量
    for(int i = 0; i < this->lArmJntTrajSol.size(); i++){
        std::pair<float, float> mRes = lArmJntTrajSol[i].lastTempPosVel();
        posLst_lArm(i) = mRes.first;
        velLst_lArm(i) = mRes.second;
    }
    for(int i = 0; i < this->rArmJntTrajSol.size(); i++){
        std::pair<float, float> mRes = rArmJntTrajSol[i].lastTempPosVel();
        posLst_rArm(i) = mRes.first;
        velLst_rArm(i) = mRes.second;
    }

}

std::vector<std::vector<ctlOutp>> JntTrackSol::solveResOutp(
    std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r
){
    std::vector<ctlOutp> mRes_l;
    std::vector<ctlOutp> mRes_r;
    std::vector<std::vector<ctlOutp>> mRes;

    if (topic_stream_) {
        const auto advance = [](std::vector<TopicJointState>& states,
                                const std::vector<int>& locked) {
            std::vector<ctlOutp> output(states.size());
            for (size_t i = 0; i < states.size(); ++i) {
                auto& state = states[i];
                if (locked[i] == 1) {
                    state.target = state.position;
                    state.velocity = 0.0;
                } else {
                    state.step(TEMPTIME);
                }
                output[i].posDes = state.position;
                output[i].velDes = state.velocity;
                output[i].ifReachEnd = std::abs(state.position - state.target) < 1e-5 &&
                                       std::abs(state.velocity) < 1e-5;
            }
            return output;
        };
        return {advance(topic_left_, ifJntStab[0]), advance(topic_right_, ifJntStab[1])};
    }

    if(this->outp_mod == 3){
        //获取左右臂杆的输出
        if(this->ifUseTemp){
            //循环输出
            for(int i = 0; i < lArmJntTrajSol.size(); i++){
                if(ifJntStab[0][i] != 1){
                    mRes_l.push_back(lArmJntTrajSol[i].desPos_get_stabTime());
                }else{
                    mRes_l.push_back(lArmStab[i].StabOutp());
                }
            }
            for(int i = 0; i < rArmJntTrajSol.size(); i++){
                if(ifJntStab[1][i] != 1){
                    mRes_r.push_back(rArmJntTrajSol[i].desPos_get_stabTime());
                }else{
                    mRes_r.push_back(rArmStab[i].StabOutp());
                }
            }            
        }else{
            //正常输出
            for(int i = 0; i < lArmJntTrajSol.size(); i++){
                if(ifJntStab[0][i] != 1){
                    mRes_l.push_back(lArmJntTrajSol[i].desPos_get_stabTimeStop());
                }else{
                    mRes_l.push_back(lArmStab[i].StabOutp());
                }
            }
            for(int i = 0; i < rArmJntTrajSol.size(); i++){
                if(ifJntStab[1][i] != 1){
                    mRes_r.push_back(rArmJntTrajSol[i].desPos_get_stabTimeStop());
                }else{
                    mRes_r.push_back(rArmStab[i].StabOutp());
                }
            }
        }
    }else{
        //获取左右臂杆的输出
        for(int i = 0; i < lArmJntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                mRes_l.push_back(lArmJntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_l.push_back(lArmStab[i].StabOutp());
            }
        }
        for(int i = 0; i < rArmJntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                mRes_r.push_back(rArmJntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_r.push_back(rArmStab[i].StabOutp());
            }
        }
    }

    /*  左右臂力控结果叠加部分
    --------------------------
    力控部分尚未开发，因此这一部分先空着
    */

    // Keep the last emitted state available when Topic takes control.
    for (size_t i = 0; i < mRes_l.size(); ++i) {
        topic_left_[i].position = mRes_l[i].posDes;
        topic_left_[i].velocity = mRes_l[i].velDes;
    }
    for (size_t i = 0; i < mRes_r.size(); ++i) {
        topic_right_[i].position = mRes_r[i].posDes;
        topic_right_[i].velocity = mRes_r[i].velDes;
    }
    mRes.push_back(mRes_l);
    mRes.push_back(mRes_r);
    return mRes;
}


/*逆解轨迹与正解算法封装*/
void FrameTrackSol::init_solver(
    JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,       //关节限制配置
    std::vector<double> JntAngConf_l, std::vector<double> JntAngConf_r,     //关节初始角度配置
    optiPrintMsgCtl mPrintMsgPkg,           //程序打印控制参数
    optimizerCtl m_opti_ctl_msg         //优化器参数
){
    //初始化求解器配置
    PathChoose mChoose_l, mChoose_r;
    mChoose_l.urdf_choose = 2;      mChoose_r.urdf_choose = 2;
    mChoose_l.base_choose = 0;      mChoose_r.base_choose = 1;
    mChoose_l.tip_choose = 0;       mChoose_r.tip_choose = 1;

    //初始化滤波器
    allJntPosFilter_l.assign(SINGLEARMDOF, posOutpWtr);
    allJntVelFilter_l.assign(SINGLEARMDOF, velOutpWtr);
    allJntPosFilter_r.assign(SINGLEARMDOF, posOutpWtr);
    allJntVelFilter_r.assign(SINGLEARMDOF, velOutpWtr);

    mSol_Pino.Init(
        mChoose_l, mChoose_r,
        lArm_LimJnt, rArm_LimJnt,
        JntAngConf_l, JntAngConf_r,
        mPrintMsgPkg,
        m_opti_ctl_msg
    );

    //初始化过程参量
    lArmLstPos.resize(JntAngConf_l.size());
    rArmLstPos.resize(JntAngConf_r.size());
    lArmLstVel.resize(JntAngConf_l.size());
    rArmLstVel.resize(JntAngConf_r.size());
    for(int i = 0; i < JntAngConf_l.size(); i++){
        lArmLstPos(i) = JntAngConf_l[i];
        lArmLstVel(i) = 0.0;
    }
    for(int j = 0; j < JntAngConf_r.size(); j++){
        rArmLstPos(j) = JntAngConf_r[j];
        rArmLstVel(j) = 0.0;
    }

    lArm_JntTrajSol.resize(JntAngConf_l.size());
    rArm_JntTrajSol.resize(JntAngConf_r.size());

    if_not_firstTemp = false;
    ifUseSolve = false;

    //修正参量初始化
    lArm_posLst_velCor = JntAngConf_l;
    rArm_posLst_velCor = JntAngConf_r;
}

void FrameTrackSol::FrameTrack(
    bool ifUpdateLim,               //判断量 -- 是否更新限制信息
    JntLimMsg allArm_LimJnt,        //求解关节限制信息，将所有的关节信息都放在这里面

    KDL::Frame lArm_desPos, KDL::Frame rArm_desPos,         //求解目标位置
    std::vector<double> lArm_desTor, std::vector<double> rArm_desTor,   //目标前馈力矩

    KDL::JntArray lArmPosRet, KDL::JntArray rArmPosRet,     //位置返回值
    KDL::JntArray lArmVelRet, KDL::JntArray rArmVelRet,     //速度返回值
    std::vector<double> lArmTorRet, std::vector<double> rArmTorRet,     //关节力矩反馈值

    modCtl mod_ctl_msg          //数据传输控制过程
){
    //更新模型封装
    if(ifUpdateLim && if_not_firstTemp){
        mSol_Pino.solverUpdate(allArm_LimJnt);
    }
    if_not_firstTemp = true;
    this->ifUseTemp = mod_ctl_msg.ifUseTemp;

    //输入量修正
    //lArm_desPos = FrameCor(lArm_desPos);
    //rArm_desPos = FrameCor(rArm_desPos);

    //模型返回值解包
    KDL::JntArray posInp_l, posInp_r;
    KDL::JntArray velInp_l, velInp_r;
    if(mod_ctl_msg.ifUseRet){
        //使用返回值
        posInp_l = lArmPosRet;
        posInp_r = rArmPosRet;
        velInp_l = lArmVelRet;
        velInp_r = rArmVelRet;
    }else{
        //不使用返回值
        posInp_l = lArmLstPos;
        posInp_r = rArmLstPos;
        velInp_l = lArmLstVel;
        velInp_r = lArmLstVel;
    }

    for(int i = 0; i < 7; i++){
        //dbg_reserve_extern[i] = posInp_l(i);
    }

    /*权重更新
    原则：如果本轮是点到点控制模式，需要更新权重为点到点适配的权重；如果本轮是连续轨迹求解且上一轮是点到点模式，需要将权重刷新为
         预设权重
    */
   /**/
    if(mod_ctl_msg.runinMod == 3){
        mSol_Pino.PinoSolver_optiWeightUpdate(2, p2p_optiWeight, false);
    }else if(mod_ctl_msg.runinMod != 3 && controlMod_lastTemp == 3){
        mSol_Pino.PinoSolver_optiWeightUpdate(1, p2p_optiWeight, true);
    }
    /**/
    controlMod_lastTemp = mod_ctl_msg.runinMod;

    //计算逆解
    std::vector<KDL::JntArray> all_posRes = mSol_Pino.IK_pinoSolver(
        lArm_desPos, rArm_desPos,
        posInp_l, posInp_r,
        mod_ctl_msg.ifUseRet
    );
    lArmDesPos = all_posRes[0];
    rArmDesPos = all_posRes[1];

    for(int i = 0; i < 7; i++){
        //dbg_reserve_extern_2[i] = lArmDesPos(i);
    }
    std::vector<double> DescInp = {
        lArm_desPos.p[0], lArm_desPos.p[1], lArm_desPos.p[2] 
    };
    for(int i = 0; i < DescInp.size(); i++){
        //dbg_reserve_extern_2[i+7] = DescInp[i];
    }

    //依据输入模式计算插补轨迹
    if(mod_ctl_msg.runinMod == 3){
        //点到点模式

        //先包装参量
        std::vector<runinMsgDotMod> runin_ctl_msg_dot_l;
        std::vector<runinMsgDotMod> runin_ctl_msg_dot_r;
        runin_ctl_msg_dot_l.resize(lArmPosRet.rows());
        runin_ctl_msg_dot_r.resize(rArmPosRet.rows());
        for(int i = 0; i < lArmPosRet.rows(); i++){
            runin_ctl_msg_dot_l[i].mPosSt = posInp_l(i);
            runin_ctl_msg_dot_l[i].mPosDes = all_posRes[0](i);
            runin_ctl_msg_dot_l[i].accTimeRateBase = mod_ctl_msg.accRateBase[0][i];
            runin_ctl_msg_dot_l[i].mTrajTime = mod_ctl_msg.TrajTime[0][i];
        }
        for(int i = 0; i < rArmPosRet.rows(); i++){
            runin_ctl_msg_dot_r[i].mPosSt = posInp_r(i);
            runin_ctl_msg_dot_r[i].mPosDes = all_posRes[1](i);
            runin_ctl_msg_dot_r[i].accTimeRateBase = mod_ctl_msg.accRateBase[1][i];
            runin_ctl_msg_dot_r[i].mTrajTime = mod_ctl_msg.TrajTime[1][i];
        }

        //再计算输出值
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            lArm_JntTrajSol.at(i).init_stabTime(runin_ctl_msg_dot_l[i], mod_ctl_msg.m_stabMsg[0][i]);
            lArm_TrajLen.push_back(lArm_JntTrajSol.at(i).TrajLen);
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            rArm_JntTrajSol.at(i).init_stabTime(runin_ctl_msg_dot_r[i], mod_ctl_msg.m_stabMsg[1][i]);
            rArm_TrajLen.push_back(rArm_JntTrajSol.at(i).TrajLen);
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }else{
        //连续求解插补模式

        //先包装参量
        std::vector<runinMsg> runin_ctl_msg_l;
        std::vector<runinMsg> runin_ctl_msg_r;
        runin_ctl_msg_l.resize(lArmPosRet.rows());
        runin_ctl_msg_r.resize(rArmPosRet.rows());

        for(int i = 0; i < lArmPosRet.rows(); i++){
            runin_ctl_msg_l[i].mPosSt = posInp_l(i);
            runin_ctl_msg_l[i].mPosDes = all_posRes[0](i);
            runin_ctl_msg_l[i].mVelSt = velInp_l(i);
            runin_ctl_msg_l[i].mTrajTime = mod_ctl_msg.upperMonitor_time;
            runin_ctl_msg_l[i].runinMod = mod_ctl_msg.runinMod;
        }
        for(int i = 0; i < rArmPosRet.rows(); i++){
            runin_ctl_msg_r[i].mPosSt = posInp_r(i);
            runin_ctl_msg_r[i].mPosDes = all_posRes[1](i);
            runin_ctl_msg_r[i].mVelSt = velInp_r(i);
            runin_ctl_msg_r[i].mTrajTime = mod_ctl_msg.upperMonitor_time;
            runin_ctl_msg_r[i].runinMod = mod_ctl_msg.runinMod;
        }

        //再计算输出值
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            runinMsg temp_runinMsg = runin_ctl_msg_l[i];
            temp_runinMsg.mTrajTime = runin_ctl_msg_l[i].mTrajTime;
            lArm_JntTrajSol.at(i).init_stabTime_continue(temp_runinMsg);
            lArm_TrajLen.push_back(lArm_JntTrajSol.at(i).TrajLen);
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            runinMsg temp_runinMsg = runin_ctl_msg_r[i];
            temp_runinMsg.mTrajTime = runin_ctl_msg_r[i].mTrajTime;
            rArm_JntTrajSol.at(i).init_stabTime_continue(temp_runinMsg);
            rArm_TrajLen.push_back(rArm_JntTrajSol.at(i).TrajLen);
        }

        //储存输出模式
        this->outp_mod = mod_ctl_msg.runinMod;
    }

    //关节静止模式初始化
    lArmStab.resize(SINGLEARMDOF);
    rArmStab.resize(SINGLEARMDOF);
    ifJntStab = mod_ctl_msg.ifJntStab;
    if(ifFirstTemp){
        for(int i = 0; i < ifJntStab[0].size(); i++){
            if(ifJntStab[0][i] == 1){
                lArmStab[i].init(lArmLstPos(i));
            }
        }
        for(int i = 0; i < ifJntStab[1].size(); i++){
            if(ifJntStab[0][i] == 1){
                lArmStab[1].init(lArmLstPos(i));
            }
        }
    }
    ifFirstTemp = false;

    /*  力控部分
    ---------------
    此部分依据笛卡尔空间力计算关节力
    此部分尚未开发，因此先空着
    */

    /**/
    if(this->outp_mod != -1){
        //更新控制参量
        for(int i = 0; i < this->lArm_JntTrajSol.size(); i++){
            std::pair<float, float> mRes = lArm_JntTrajSol[i].lastTempPosVel();
            lArmLstPos(i) = mRes.first;
            lArmLstVel(i) = mRes.second;
        }
        for(int i = 0; i < this->rArm_JntTrajSol.size(); i++){
            std::pair<float, float> mRes = rArm_JntTrajSol[i].lastTempPosVel();
            rArmLstPos(i) = mRes.first;
            rArmLstVel(i) = mRes.second;
        }
    }
    /**/
}

std::vector<std::vector<ctlOutp>> FrameTrackSol::IKsolveResOutp(
    std::vector<double> lArm_TorRet, std::vector<double> rArm_TorRet
){
    //输出逆解轨迹函数 -- 依据求解模式

    std::vector<ctlOutp> mRes_l;
    std::vector<ctlOutp> mRes_r;
    std::vector<std::vector<ctlOutp>> mRes;

    if(this->outp_mod == 3){
        //获取左右臂杆的输出
        if(this->ifUseTemp){
            //循环输出
            for(int i = 0; i < lArm_JntTrajSol.size(); i++){
                if(ifJntStab[0][i] != 1){
                    mRes_l.push_back(lArm_JntTrajSol[i].desPos_get_stabTime());
                }else{
                    mRes_l.push_back(lArmStab[i].StabOutp());
                }
            }

            for(int i = 0; i < rArm_JntTrajSol.size(); i++){
                if(ifJntStab[1][i] != 1){
                    mRes_r.push_back(rArm_JntTrajSol[i].desPos_get_stabTime());
                }else{
                    mRes_r.push_back(rArmStab[i].StabOutp());
                }
            }
        }else{
            //正常输出
            for(int i = 0; i < lArm_JntTrajSol.size(); i++){
                if(ifJntStab[0][i] != 1){
                    mRes_l.push_back(lArm_JntTrajSol[i].desPos_get_stabTimeStop());
                }else{
                    mRes_l.push_back(lArmStab[i].StabOutp());
                }
            }
            for(int i = 0; i < rArm_JntTrajSol.size(); i++){
                if(ifJntStab[1][i] != 1){
                    mRes_r.push_back(rArm_JntTrajSol[i].desPos_get_stabTimeStop());
                }else{
                    mRes_r.push_back(rArmStab[i].StabOutp());
                }
            }
        }
    }else if(this->outp_mod == -1){
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                ctlOutp temp_res_l;
                temp_res_l.posDes = lArmDesPos(i);
                if(ifUseSolve){
                    temp_res_l.velDes = (lArmDesPos(i) - lArmLstPos(i))/tempT;
                }else{
                    temp_res_l.velDes = 0.0;
                }

                temp_res_l.grav_TorDes = 0.0;
                temp_res_l.interia_TorDes = 0.0;
                temp_res_l.feedforward_TorDes = 0.0;

                temp_res_l.ifReachEnd = true;
                mRes_l.push_back(temp_res_l);
            }else{
                mRes_l.push_back(lArmStab[i].StabOutp());
            }
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                ctlOutp temp_res_r;
                temp_res_r.posDes = rArmDesPos(i);
                if(ifUseSolve){
                    temp_res_r.velDes = (rArmDesPos(i) - rArmLstPos(i))/tempT;
                }else{
                    temp_res_r.velDes = 0.0;
                }

                temp_res_r.grav_TorDes = 0.0;
                temp_res_r.interia_TorDes = 0.0;
                temp_res_r.feedforward_TorDes = 0.0;

                temp_res_r.ifReachEnd = true;
                mRes_r.push_back(temp_res_r);
            }else{
                mRes_r.push_back(rArmStab[i].StabOutp());
            }
        }
        if(ifUseSolve){
            ifUseSolve = false;
        }

    }else{
        //获取左右臂杆的输出
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                mRes_l.push_back(lArm_JntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_l.push_back(lArmStab[i].StabOutp());
            }
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                mRes_r.push_back(rArm_JntTrajSol[i].desPos_get_stab_Continue());
            }else{
                mRes_r.push_back(rArmStab[i].StabOutp());
            }
        }

        //修正速度输出 -- 治标不治本
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                mRes_l[i].velDes = (mRes_l[i].posDes - lArm_posLst_velCor[i]) / tempT;
            }
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                mRes_r[i].velDes = (mRes_r[i].posDes - rArm_posLst_velCor[i]) / tempT;
            }
        }
        for(int i = 0; i < lArm_JntTrajSol.size(); i++){
            if(ifJntStab[0][i] != 1){
                lArm_posLst_velCor[i] = mRes_l[i].posDes;
            }
        }
        for(int i = 0; i < rArm_JntTrajSol.size(); i++){
            if(ifJntStab[1][i] != 1){
                rArm_posLst_velCor[i] = mRes_r[i].posDes;
            }
        }
    }

    for(int i = 0; i < this->lArm_JntTrajSol.size(); i++){
        std::pair<float, float> mResFrame_l = lArm_JntTrajSol[i].lastTempPosVel();
        lArmLstPos(i) = mResFrame_l.first;
        lArmLstVel(i) = mResFrame_l.second;
    }
    for(int i = 0; i < this->rArm_JntTrajSol.size(); i++){
        std::pair<float, float> mResFrame_r = rArm_JntTrajSol[i].lastTempPosVel();
        rArmLstPos(i) = mResFrame_r.first;
        rArmLstVel(i) = mResFrame_r.second;
    }

    //滤波后输出

    /*  力控叠加部分
    ------------------
    此部分尚未作开发，因此先空着
    */

    reserve[0] = mRes_l[0].velDes;
    reserve[1] = (mRes_l[0].posDes - dbg_a)/0.02;
    dbg_a = mRes_l[0].posDes;

    std::vector<ctlOutp> mRes_l_ftred;
    std::vector<ctlOutp> mRes_r_ftred;
    if(ifUseFtr){
        for(int i = 0; i < mRes_l.size(); i++){
            ctlOutp temp_res_l;
            temp_res_l.posDes = allJntPosFilter_l[i].filter(mRes_l[i].posDes);
            temp_res_l.velDes = allJntVelFilter_l[i].filter(mRes_l[i].velDes);

            temp_res_l.grav_TorDes = mRes_l[i].grav_TorDes;
            temp_res_l.interia_TorDes = mRes_l[i].interia_TorDes;
            temp_res_l.feedforward_TorDes = mRes_l[i].feedforward_TorDes;
            temp_res_l.ifReachEnd = mRes_l[i].ifReachEnd;

            mRes_l_ftred.push_back(temp_res_l);
        }
        for(int i = 0; i < mRes_r.size(); i++){
            ctlOutp temp_res_r;
            temp_res_r.posDes = allJntPosFilter_r[i].filter(mRes_r[i].posDes);
            temp_res_r.velDes = allJntVelFilter_r[i].filter(mRes_r[i].velDes);

            temp_res_r.grav_TorDes = mRes_r[i].grav_TorDes;
            temp_res_r.interia_TorDes = mRes_r[i].interia_TorDes;
            temp_res_r.feedforward_TorDes = mRes_r[i].feedforward_TorDes;
            temp_res_r.ifReachEnd = mRes_r[i].ifReachEnd;

            mRes_r_ftred.push_back(temp_res_r);
        }

        mRes.push_back(mRes_l_ftred);
        mRes.push_back(mRes_r_ftred);
    }else{
        mRes.push_back(mRes_l);
        mRes.push_back(mRes_r);
    }
    return mRes;
}

double WeightedMovingAverage::calculateWeightedAverage(){
    double weighted_sum = 0.0;
        
    for (size_t i = 0; i < window_size_; ++i) {
        weighted_sum += data_buffer_[i] * weights_[i];
    }
    
    return weighted_sum / sum_weights_;
}

WeightedMovingAverage::WeightedMovingAverage(size_t window_size, const std::vector<double>& weights)
    : window_size_(window_size), weights_(weights) {
    
    if (window_size == 0) {
        throw std::invalid_argument("Window size must be greater than 0");
    }
    
    if (weights.size() != window_size) {
        throw std::invalid_argument("Weights size must match window size");
    }
    
    // 计算权重总和
    sum_weights_ = std::accumulate(weights.begin(), weights.end(), 0.0);
    
    if (sum_weights_ == 0.0) {
        throw std::invalid_argument("Sum of weights cannot be zero");
    }
}

// 添加新数据并返回滤波后的值
double WeightedMovingAverage::filter(double new_value) {
    // 添加新数据
    data_buffer_.push_back(new_value);
    
    // 如果缓冲区超过窗口大小，移除最旧的数据
    if (data_buffer_.size() > window_size_) {
        data_buffer_.pop_front();
    }
    
    // 如果数据不足，返回原始值
    if (data_buffer_.size() < window_size_) {
        return new_value;
    }
    
    // 计算加权平均值
    return calculateWeightedAverage();
}

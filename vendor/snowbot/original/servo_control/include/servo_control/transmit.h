#ifndef TRANSMIT_H
#define TRANSMIT_H

#include <inttypes.h>
#include <vector>
#include <string>
#include <utility>

#include "ros_controller/rosLoopMsgDefine.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "ethercat.h"
#ifdef __cplusplus
}
#endif

// 关节拓扑
typedef struct {
    int board_count;
    std::vector<int> joints_per_board;
} JntTopology;

extern JntTopology g_jnt_topology;

// EtherCAT 接口
void EtherCAT_Init(char *ifname);
void EtherCAT_Stop();
void EtherCAT_BeginArmShutdownHandoff();
void EtherCAT_EndArmShutdownHandoff();
bool EtherCAT_isOperational();
bool EtherCAT_communicationHealthy();
void EtherCAT_Run();
void EtherCAT_Command_Set();
void EtherCAT_direGetFdback();
void ecatcheck();

void EtherCAT_transInp(std::vector<std::vector<joint_desMsg>> allJnt_desMsg);
std::vector<std::vector<joint_retMsg>> EtherCAT_transOutp();
std::vector<std::vector<joint_retMsg>> runMsgGet();
std::vector<std::vector<joint_retMsg>> runImpl(std::vector<std::vector<joint_desMsg>> allJnt_desMsg);

void startRun();

// 电机使能
void EtherCAT_setMotorEnable(bool en);
void EtherCAT_setPerMotorEnable(const bool mask[22]);
void EtherCAT_setSingleMotorEnable(int idx, bool en);
// 方案 E: 设置使能时刻重力补偿前馈 (CiA402 使能序列最后一步注入)
void EtherCAT_setGravityFF(int idx, float tor);

// 夹爪
void setLeftGripperCmd(double pos, double spd, double tor, double Kp, double Kd);
void setRightGripperCmd(double pos, double spd, double tor, double Kp, double Kd);
double getGripperFbPos(int arm);  // 0=右, 1=左, 返回 rad
void getGripperFb(int arm, float& pos, float& spd, float& cur, uint16_t& status);
void getSensorData(int arm, float& fx, float& fy, float& fz, float& mx, float& my, float& mz);

// 躯干/头部电机
void setTorsoMotorCmd(int idx, double pos, double spd, double tor, double kp, double kd);
void setHeadMotorCmd(int idx, double pos, double spd, double tor, double kp, double kd);
void getMotorFb_torso(int idx, float& pos, float& spd, float& cur, uint16_t& st);
void getMotorFb_head(int idx, float& pos, float& spd, float& cur, uint16_t& st);

// 辅助
JntTopology get_jnt_topology();
std::vector<double> joint_Kt_get();
std::vector<std::vector<joint_desMsg>> JntDesMsg_inContainer_Outp_v1();

#endif

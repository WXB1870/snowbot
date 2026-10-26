/*
 * @brief EtherCAT 桥接层 — 旧 transmit.h 接口 → 新 EtherCATDriver
 */
#include "ethercat.h"
#include "servo_control/transmit.h"
#include "servo_control/ethercat_driver.h"

#include <cstdio>
#include <cstring>
#include <vector>
#include <iostream>

#define KT_VALUE 0.5f

static EtherCATDriver* g_ecat_driver = nullptr;
static float g_last_pos[14] = {0};
static float g_last_spd[14] = {0};
static float g_last_kp[14] = {0};
static float g_last_kd[14] = {0};
static float g_last_fb_pos[14] = {0};
static float g_last_fb_spd[14] = {0};
static float g_last_fb_cur[14] = {0};
JntTopology g_jnt_topology;
std::atomic<bool> running{false};

// ===== 接口实现 =====

void EtherCAT_Init(char *ifname) {
    delete g_ecat_driver;
    g_ecat_driver = new EtherCATDriver();
    if (!g_ecat_driver->init(std::string(ifname))) {
        printf("[EtherCAT] Init failed, debug mode\n");
        delete g_ecat_driver;
        g_ecat_driver = nullptr;
    }
}

bool EtherCAT_isOperational() {
    return g_ecat_driver && g_ecat_driver->isOperational();
}

bool EtherCAT_communicationHealthy() {
    return g_ecat_driver && g_ecat_driver->communicationHealthy();
}

void EtherCAT_Stop() {
    delete g_ecat_driver;
    g_ecat_driver = nullptr;
}

void EtherCAT_BeginArmShutdownHandoff() {
    if (g_ecat_driver) g_ecat_driver->beginArmShutdownHandoff();
}

void EtherCAT_EndArmShutdownHandoff() {
    if (g_ecat_driver) g_ecat_driver->endArmShutdownHandoff();
}

void ecatcheck() {
    if (!g_ecat_driver) return;
    static auto last_report = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_report < std::chrono::seconds(1)) return;
    last_report = now;
    g_ecat_driver->printCommunicationStatus();
}
void EtherCAT_direGetFdback() {}
void EtherCAT_Run() {}
void EtherCAT_Transmit() {}

[[maybe_unused]] static void print_pdo_slave() {
    if (!g_ecat_driver || !g_ecat_driver->isOperational()) return;
    printf("[PDO FB] pos/spd/cur\n");
    printf("  L: ");
    for (int i = 0; i < 7; i++) printf("J%d=%+6.3f ", i+1, g_last_fb_pos[i]);
    printf("\n     ");
    for (int i = 0; i < 7; i++) printf("spd=%5.1f ", g_last_fb_spd[i]);
    printf("\n     ");
    for (int i = 0; i < 7; i++) printf("cur=%5.2f ", g_last_fb_cur[i]);
    printf("\n  R: ");
    for (int i = 0; i < 7; i++) printf("J%d=%+6.3f ", i+1, g_last_fb_pos[7+i]);
    printf("\n     ");
    for (int i = 0; i < 7; i++) printf("spd=%5.1f ", g_last_fb_spd[7+i]);
    printf("\n     ");
    for (int i = 0; i < 7; i++) printf("cur=%5.2f ", g_last_fb_cur[7+i]);
    printf("\n");
}

void EtherCAT_Command_Set() {
}

void EtherCAT_transInp(std::vector<std::vector<joint_desMsg>> allJnt_desMsg) {
    int idx = 0;
    for (auto& board : allJnt_desMsg) {
        for (auto& cmd : board) {
            if (idx < 14) {
                g_last_pos[idx] = (float)cmd.joint_pos;
                g_last_spd[idx] = (float)cmd.joint_spd;
                g_last_kp[idx]  = (float)cmd.joint_Kp;
                g_last_kd[idx]  = (float)cmd.joint_Kd;
            }
            // ROS: L_J1~J7(0-6) R_J1~J7(7-13)
            // ECAT driver: L_J1~J7(0-6) L_J8=7(夹爪, 单独) R_J1~J7(8-14) R_J8=15(夹爪)
            int eidx = (idx < 7) ? idx : (idx + 1);  // 跳过 L_J8 slot
            if (g_ecat_driver && g_ecat_driver->isOperational())
                g_ecat_driver->setMotorCmd(eidx, cmd.joint_pos, cmd.joint_spd,
                                           cmd.joint_tor, cmd.joint_Kp, cmd.joint_Kd);
            idx++;
        }
    }
}

std::vector<std::vector<joint_retMsg>> EtherCAT_transOutp() { return runMsgGet(); }

std::vector<std::vector<joint_retMsg>> runMsgGet() {
    std::vector<std::vector<joint_retMsg>> ret;
    for (int b = 0; b < g_jnt_topology.board_count; b++)
        ret.push_back(std::vector<joint_retMsg>(g_jnt_topology.joints_per_board[b]));
    if (!g_ecat_driver || !g_ecat_driver->isOperational()) return ret;
    int idx = 0;
    for (auto& board : ret) {
        for (auto& j : board) {
            int eidx = (idx < 7) ? idx : (idx + 1);  // 跳过 L_J8 slot
            float pos, spd, cur; uint16_t st;
            g_ecat_driver->getMotorFb(eidx, pos, spd, cur, st);
            j.joint_posRet = pos; j.joint_spdRet = spd;
            j.joint_currentRet = cur; j.joint_temperature = 0;
            if (idx < 14) {
                g_last_fb_pos[idx] = pos;
                g_last_fb_spd[idx] = spd;
                g_last_fb_cur[idx] = cur;
            }
            idx++;
        }
    }
    return ret;
}

std::vector<std::vector<joint_retMsg>> runImpl(std::vector<std::vector<joint_desMsg>> allJnt_desMsg) {
    EtherCAT_transInp(allJnt_desMsg);
    // PDO 收发由 EtherCATDriver 的独立实时线程完成。
    ecatcheck();
    return runMsgGet();
}

void startRun() {
    std::cout << "[EtherCAT] Direct driver mode" << std::endl;
    g_jnt_topology.board_count = 1;
    g_jnt_topology.joints_per_board = { 14 };  // L-J1~J7, R-J1~J7 (夹爪单独控制)
}

// 电机使能
void EtherCAT_setMotorEnable(bool en) {
    if (g_ecat_driver) g_ecat_driver->setMotorEnable(en);
}
void EtherCAT_setPerMotorEnable(const bool mask[22]) {
    if (g_ecat_driver) g_ecat_driver->setPerMotorEnable(mask);
}
void EtherCAT_setSingleMotorEnable(int idx, bool en) {
    if (g_ecat_driver) g_ecat_driver->setSingleMotorEnable(idx, en);
}
void EtherCAT_setGravityFF(int idx, float tor) {
    if (g_ecat_driver) g_ecat_driver->setGravityFF(idx, tor);
}

// 传感器
void getSensorData(int arm, float& fx, float& fy, float& fz, float& mx, float& my, float& mz) {
    if (g_ecat_driver) g_ecat_driver->getSensorData(arm, fx, fy, fz, mx, my, mz);
}

// 夹爪 (arm 0 = 左夹爪, arm 1 = 右夹爪)
void setLeftGripperCmd(double pos, double spd, double tor, double Kp, double Kd) {
    if (g_ecat_driver) g_ecat_driver->setGripperCmd(0, (float)pos, (float)spd, (float)tor, (float)Kp, (float)Kd);
}
void setRightGripperCmd(double pos, double spd, double tor, double Kp, double Kd) {
    if (g_ecat_driver) g_ecat_driver->setGripperCmd(1, (float)pos, (float)spd, (float)tor, (float)Kp, (float)Kd);
}
double getGripperFbPos(int arm) {
    float pos = 0.0f, spd = 0.0f, cur = 0.0f;
    uint16_t status = 0;
    getGripperFb(arm, pos, spd, cur, status);
    return static_cast<double>(pos);
}

void getGripperFb(int arm, float& pos, float& spd, float& cur, uint16_t& status) {
    pos = spd = cur = 0.0f;
    status = 0;
    if (!g_ecat_driver || !g_ecat_driver->isOperational()) return;
    const int eidx = (arm == 0) ? 7 : 15;  // 左夹爪=7，右夹爪=15
    g_ecat_driver->getMotorFb(eidx, pos, spd, cur, status);
}

// 躯干/头部电机 (EtherCAT index: 头部=16~17, 躯干=18~19)
void setTorsoMotorCmd(int idx, double pos, double spd, double tor, double kp, double kd) {
    if (g_ecat_driver) g_ecat_driver->setMotorCmd(18 + idx, (float)pos, (float)spd, (float)tor, (float)kp, (float)kd);
}
void setHeadMotorCmd(int idx, double pos, double spd, double tor, double kp, double kd) {
    if (g_ecat_driver) g_ecat_driver->setMotorCmd(16 + idx, (float)pos, (float)spd, (float)tor, (float)kp, (float)kd);
}
void getMotorFb_torso(int idx, float& pos, float& spd, float& cur, uint16_t& st) {
    if (g_ecat_driver) g_ecat_driver->getMotorFb(18 + idx, pos, spd, cur, st);
}
void getMotorFb_head(int idx, float& pos, float& spd, float& cur, uint16_t& st) {
    if (g_ecat_driver) g_ecat_driver->getMotorFb(16 + idx, pos, spd, cur, st);
}

// 辅助
JntTopology get_jnt_topology() { return g_jnt_topology; }
std::vector<double> joint_Kt_get() { return std::vector<double>(14, KT_VALUE); }
std::vector<std::vector<joint_desMsg>> JntDesMsg_inContainer_Outp_v1() {
    return { std::vector<joint_desMsg>(14) };
}

/*
 * @brief EtherCAT 直接驱动 — 替代旧 CAN 网关方案
 *        单主站 4 链 (左臂/右臂/躯干/头部)
 *        传感器 + 电机 PDO 同 SOEM_Demo 协议
 */
#ifndef ETHERCAT_DRIVER_H
#define ETHERCAT_DRIVER_H

#include <inttypes.h>
#include <vector>
#include <string>
#include <atomic>
#include <chrono>
#include <thread>

#ifdef __cplusplus
extern "C" {
#endif
#include "ethercat.h"
#ifdef __cplusplus
}
#endif

// ========== 从站数量 ==========
#define MOTORS_PER_ARM   8       // 单臂电机数 (J8 夹爪)
#define MOTORS_TORSO     2
#define MOTORS_HEAD      2

#define TOTAL_SLAVES     22      // 驱动内部支持的最大电机槽位数

// 电机索引约定: 
//   0~7  : 左臂 J1~J8 (7=夹爪)
//   8~15 : 右臂 J1~J8 (15=夹爪)
//   16~17: 头部 J1~J2
//   18~19: 腰部/躯干 J1~J2

// ========== PDO 结构体 (与 SOEM_Demo 一致, 必须 packed) ==========

#pragma pack(push, 1)

// 传感器 RxPDO (主站→从站), 6 字节, 对象 0x1602
typedef struct {
    uint16_t Res1;
    uint16_t Res2;
    uint16_t Res3;
} sensor_rx_pdo_t;

// 传感器 TxPDO (从站→主站), 26 字节, 对象 0x1A02
typedef struct {
    uint16_t DataNo;
    float    Fx, Fy, Fz;
    float    Mx, My, Mz;
} sensor_tx_pdo_t;

// 电机 RxPDO (主站→从站), 22 字节, 对象 0x1600
typedef struct {
    uint16_t controlword;       // 0x6040
    float    KP;                // 0x200F:01
    float    KD;                // 0x200F:02
    float    target_position;   // 0x200F:03  rad
    float    target_speed;      // 0x200F:04  rad/s
    float    target_torque;     // 0x200F:05  Nm
} motor_rx_pdo_t;

// 电机 TxPDO (从站→主站), 14 字节, 对象 0x1A00
typedef struct {
    uint16_t statusword;         // 0x6041
    float    actual_position;    // 0x2010  rad
    float    actual_speed;       // 0x2011  rad/s
    float    actual_current;     // 0x2012  A
} motor_tx_pdo_t;

// 头/腰 标准 CSP RxPDO (0x1600, 与 csp_test 一致): 14 字节
typedef struct {
    uint16_t controlword;        // 0x6040
    int32_t  position;           // 0x607A (CSP encoder counts)
    int32_t  velocity;           // 0x60FF
    int16_t  torque;             // 0x6071
    int8_t   ModeOp;             // 0x6060 (8 = CSP)
    int8_t   reserved;
} csp_rx_t;

// 头/腰 标准 CSP TxPDO (0x1A00, 与 csp_test 一致): 14 字节
typedef struct {
    uint16_t statusword;         // 0x6041
    int32_t  position;           // 0x6064 (actual encoder counts)
    int32_t  velocity;           // 0x606C
    int16_t  torque;             // 0x6077
    int8_t   ModeOp;             // 0x6061 (mode display)
    int8_t   reserved;
} csp_tx_t;

#pragma pack(pop)

// ========== 电机控制参数 ==========
#define MODE_CSP 0x08
#define MODE_PT  0xFF

// ========== EtherCAT 控制周期 ==========
#define EC_CYCLETIME_US   1000    // 1kHz

// ========== EtherCAT 驱动类 ==========
class EtherCATDriver {
public:
    EtherCATDriver();
    ~EtherCATDriver();

    bool init(const std::string& ifname);
    void cycle();             // 单次 PDO 收发 (由 ROS loop 调用)
    void stop();

    // 设置电机命令 (索引: 0=左臂J1~J7, 7=左臂夹爪, 8=右臂J1~J7, 15=右臂夹爪, 16=躯干1, 17=躯干2, 18=头部1, 19=头部2)
    void setMotorCmd(int idx, float pos, float spd, float tor, float kp, float kd);
    // 方案 E: 设置使能时刻重力补偿前馈 (CiA402 使能序列最后一步注入)
    void setGravityFF(int idx, float tor) {
        if (idx >= 0 && idx < TOTAL_SLAVES) gravity_ff_[idx].store(tor);
    }
    // 获取电机反馈
    void getMotorFb(int idx, float& pos, float& spd, float& current, uint16_t& status);

    // 获取力传感器 (0=右臂, 1=左臂)
    void getSensorData(int arm, float& fx, float& fy, float& fz,
                       float& mx, float& my, float& mz);
    // 传感器归零
    void zeroSensor(int arm = 0);

    // 电机使能/抱闸控制
    void setMotorEnable(bool en) { motor_enable_ = en; }
    void beginArmShutdownHandoff() { arm_shutdown_handoff_.store(true); }
    void endArmShutdownHandoff() { arm_shutdown_handoff_.store(false); }
    bool getMotorEnable() const { return motor_enable_; }
    void setPerMotorEnable(const bool mask[TOTAL_SLAVES]) {
        for (int i = 0; i < TOTAL_SLAVES; i++) motor_enable_mask_[i].store(mask[i]);
    }
    void setSingleMotorEnable(int idx, bool en) {
        if (idx >= 0 && idx < TOTAL_SLAVES) motor_enable_mask_[idx].store(en);
    }

    // 夹爪控制
    void setGripperCmd(int arm, float pos, float spd, float tor, float kp, float kd);

    // 状态查询
    bool isOperational() const { return inOP_; }
    int  getWkc() const { return wkc_; }
    bool communicationHealthy() const { return expectedWKC_ > 0 && wkc_.load() == expectedWKC_; }
    void printCommunicationStatus() const; // Cached values only; call outside RT thread.

private:
    void realtimeLoop();
    void motorPDOConfig(int slave);
    void cspPDOConfig(int slave);
    void motorPTConfig(int slave);
    void sensorEEPROMFix(int slave);
    void motorEnable(int motorIndex, int step);
    void processCycle();

    char ifname_[64];
    char IOmap_[16384];
    int  expectedWKC_;
    std::atomic<int> wkc_{0};
    std::atomic<bool> inOP_{false};
    std::atomic<bool> running_{false};

    // PDO 指针
    sensor_rx_pdo_t* s_rx_[2];
    sensor_tx_pdo_t* s_tx_[2];
    motor_rx_pdo_t*  m_rx_[TOTAL_SLAVES];
    motor_tx_pdo_t*  m_tx_[TOTAL_SLAVES];
    csp_rx_t*        csp_rx_[TOTAL_SLAVES];
    csp_tx_t*        csp_tx_[TOTAL_SLAVES];
    bool             is_csp_motor_[TOTAL_SLAVES]{};

    // 电机命令缓存
    std::atomic<float> motor_cmd_pos_[TOTAL_SLAVES]{};
    std::atomic<float> motor_cmd_spd_[TOTAL_SLAVES]{};
    std::atomic<float> motor_cmd_tor_[TOTAL_SLAVES]{};
    std::atomic<float> motor_cmd_kp_[TOTAL_SLAVES]{};
    std::atomic<float> motor_cmd_kd_[TOTAL_SLAVES]{};
    std::atomic<float> gravity_ff_[TOTAL_SLAVES]{};  // 方案 E: 使能时刻重力补偿前馈

    int cycle_count_;
    std::atomic<bool> motor_enable_{true};  // 电机使能: true=使能, false=抱闸
    std::atomic<bool> motor_enable_mask_[TOTAL_SLAVES]{};  // 单电机使能 mask
    std::atomic<bool> arm_shutdown_handoff_{false};

    // 反馈快照，ROS 线程只读取快照，不直接读取 PDO 内存。
    std::atomic<float> motor_fb_pos_[TOTAL_SLAVES]{};
    std::atomic<float> motor_fb_spd_[TOTAL_SLAVES]{};
    std::atomic<float> motor_fb_cur_[TOTAL_SLAVES]{};
    std::atomic<uint16_t> motor_fb_status_[TOTAL_SLAVES]{};
    std::atomic<float> sensor_fb_[2][6]{};

    std::thread cycle_thread_;
    std::atomic<bool> stop_cycle_thread_{false};
    std::atomic<uint64_t> completed_pdo_cycles_{0};
    std::atomic<uint64_t> bad_wkc_cycles_{0};
    std::atomic<int64_t> last_good_pdo_ns_{0};
    std::atomic<int64_t> handshake_last_pdo_ns_{0};
    std::atomic<int64_t> first_rt_pdo_ns_{0};
    std::atomic<int> handshake_last_wkc_{0};
    std::atomic<int> first_rt_wkc_{0};
    std::atomic<uint16_t> motor_sent_cw_[TOTAL_SLAVES]{};

    // OP 握手 → 实时线程无缝接力（避免 std::thread 创建期 PDO 空白导致从站退 OP）
    std::atomic<bool> rt_ready_to_take_over_{false};  // cycle_thread_ 已就绪可接管
    std::atomic<bool> handoff_done_{false};           // 主线程让出 PDO 控制权

    // PDO 周期与收发耗时统计（仅由 cycle() 所在线程访问）
    std::chrono::steady_clock::time_point last_pdo_send_time_{};
    bool pdo_timing_initialized_{false};
    uint64_t timing_cycles_{0};
    uint64_t timing_io_cycles_{0};
    uint64_t timing_over_1ms_{0};
    uint64_t timing_period_min_us_{0};
    uint64_t timing_period_max_us_{0};
    uint64_t timing_period_sum_us_{0};
    uint64_t timing_io_min_us_{0};
    uint64_t timing_io_max_us_{0};
    uint64_t timing_io_sum_us_{0};
};

#endif // ETHERCAT_DRIVER_H

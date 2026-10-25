/*
 * @brief EtherCAT 直接驱动实现
 *        基于 SNOWBOT_SOEM_Demo/sensor_motor_test.c
 *        单主站 4 链 (左臂/右臂/躯干/头部)
 */
#include "servo_control/ethercat_driver.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

extern "C" {
#include "ethercat.h"
}

// ========== 使能序列步长 ==========
#define EN_STEP            5       // 每步周期数 (5ms)
constexpr int EC_OP_HANDSHAKE_CYCLETIME_US = 1000;

static volatile int g_keep_running = 1;
static void sig_handler(int) { g_keep_running = 0; }

static bool is_force_sensor_slave(int slave) {
    return ec_slave[slave].eep_man == 0x00000000 &&
           ec_slave[slave].eep_id == 0x00000259;
}

constexpr int64_t kEcatCycleNs = static_cast<int64_t>(EC_CYCLETIME_US) * 1000;
constexpr int kEcatPriority = 50;

static int64_t dc_phase_correction_ns(int64_t dc_time, int64_t cycle_ns,
                                      int64_t& integral) {
    if (dc_time <= 0 || cycle_ns <= 0) return 0;
    int64_t delta = (dc_time - 50000) % cycle_ns;
    if (delta > cycle_ns / 2) delta -= cycle_ns;
    if (delta > 0) ++integral;
    if (delta < 0) --integral;
    return -(delta / 100) - (integral / 20);
}

static void add_ns(timespec& ts, int64_t ns) {
    ts.tv_sec += ns / 1000000000LL;
    ts.tv_nsec += ns % 1000000000LL;
    if (ts.tv_nsec >= 1000000000L) {
        ++ts.tv_sec;
        ts.tv_nsec -= 1000000000L;
    } else if (ts.tv_nsec < 0) {
        --ts.tv_sec;
        ts.tv_nsec += 1000000000L;
    }
}

// ========== 构造/析构 ==========
EtherCATDriver::EtherCATDriver()
    : expectedWKC_(0), cycle_count_(0)
{
    memset(ifname_, 0, sizeof(ifname_));
    memset(IOmap_, 0, sizeof(IOmap_));
    memset(s_rx_, 0, sizeof(s_rx_));
    memset(s_tx_, 0, sizeof(s_tx_));
    memset(m_rx_, 0, sizeof(m_rx_));
    memset(m_tx_, 0, sizeof(m_tx_));
}

EtherCATDriver::~EtherCATDriver() { stop(); }

// ========== 电机 PDO 配置 (SDO, PREOP) ==========
void EtherCATDriver::motorPDOConfig(int slave) {
    int s = slave;
    uint32_t entry;
    uint8_t zero = 0;

    // RxPDO 0x1600: 6 entries
    ec_SDOwrite(s, 0x1600, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6040 << 16) | (0x00 << 8) | 0x10;  // Controlword
    ec_SDOwrite(s, 0x1600, 1, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x200F << 16) | (0x01 << 8) | 0x20;  // KP
    ec_SDOwrite(s, 0x1600, 2, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x200F << 16) | (0x02 << 8) | 0x20;  // KD
    ec_SDOwrite(s, 0x1600, 3, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x200F << 16) | (0x03 << 8) | 0x20;  // Target position
    ec_SDOwrite(s, 0x1600, 4, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x200F << 16) | (0x04 << 8) | 0x20;  // Target speed
    ec_SDOwrite(s, 0x1600, 5, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x200F << 16) | (0x05 << 8) | 0x20;  // Target torque
    ec_SDOwrite(s, 0x1600, 6, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    uint8_t n = 6;
    ec_SDOwrite(s, 0x1600, 0, FALSE, sizeof(n), &n, EC_TIMEOUTSAFE);

    // TxPDO 0x1A00: 4 entries
    ec_SDOwrite(s, 0x1A00, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6041 << 16) | (0x00 << 8) | 0x10;  // Statusword
    ec_SDOwrite(s, 0x1A00, 1, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x2010 << 16) | (0x00 << 8) | 0x20;  // Actual position
    ec_SDOwrite(s, 0x1A00, 2, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x2011 << 16) | (0x00 << 8) | 0x20;  // Actual speed
    ec_SDOwrite(s, 0x1A00, 3, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x2012 << 16) | (0x00 << 8) | 0x20;  // Actual current
    ec_SDOwrite(s, 0x1A00, 4, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    n = 4;
    ec_SDOwrite(s, 0x1A00, 0, FALSE, sizeof(n), &n, EC_TIMEOUTSAFE);

    // SM PDO assignment
    uint16_t rx_pdo = 0x1600, tx_pdo = 0x1A00;
    uint8_t one = 1;
    ec_SDOwrite(s, 0x1C12, 1, FALSE, sizeof(rx_pdo), &rx_pdo, EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(one),   &one,    EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C13, 1, FALSE, sizeof(tx_pdo), &tx_pdo, EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(one),   &one,    EC_TIMEOUTSAFE);

    printf("[PDO] Motor slave %d configured\n", s);
}

// ========== 电机模式参数配置 (SDO, SAFE_OP) ==========
void EtherCATDriver::motorPTConfig(int slave) {
    int s = slave;
    // 检查电机 Station Alias，判断是手臂还是头腰
    uint16_t alias = 0;
    int size = sizeof(alias);
    ec_SDOread(s, 0x2002, 0, FALSE, &size, &alias, EC_TIMEOUTSAFE);

    int8_t mode = MODE_PT;
    if (alias >= 17 && alias <= 20) {
        // 头腰(17~20)使用标准 CSP 纯位置模式 (0x6060 = 8)
        mode = MODE_CSP;
        ec_SDOwrite(s, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
        uint16_t torque_limit = 1000;
        ec_SDOwrite(s, 0x60E0, 0, FALSE, sizeof(torque_limit), &torque_limit, EC_TIMEOUTSAFE);
        ec_SDOwrite(s, 0x60E1, 0, FALSE, sizeof(torque_limit), &torque_limit, EC_TIMEOUTSAFE);
        printf("[SDO] Motor %d (alias %d): CSP mode (8) set, torque_limit=%d\n", s, alias, torque_limit);
    } else {
        // 手臂(1~16)使用 PT 阻抗模式 (0x6060 = 0xFF)
        mode = MODE_PT;
        ec_SDOwrite(s, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
        uint16_t torque_limit = 950;  // 95% 最大转矩
        ec_SDOwrite(s, 0x60E0, 0, FALSE, sizeof(torque_limit), &torque_limit, EC_TIMEOUTSAFE);
        ec_SDOwrite(s, 0x60E1, 0, FALSE, sizeof(torque_limit), &torque_limit, EC_TIMEOUTSAFE);
        printf("[SDO] Motor %d (alias %d): PT mode set\n", s, alias);
    }
}

// ========== 标准 CSP 电机 PDO 配置 (0x1600/0x1A00, 与 csp_test 一致) ==========
void EtherCATDriver::cspPDOConfig(int slave) {
    int s = slave;
    uint32_t entry;
    uint8_t zero = 0;

    // 1. 禁用 SM PDO
    ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);

    // 2. RxPDO 0x1600 (outputs_t): 0x6040(16bit) + 0x607A(32bit) + 0x60FF(32bit) + 0x6071(16bit) + 0x6060(8bit) + reserved(8bit) = 14 bytes
    ec_SDOwrite(s, 0x1600, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6040 << 16) | (0x00 << 8) | 0x10;  // Controlword
    ec_SDOwrite(s, 0x1600, 1, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x607A << 16) | (0x00 << 8) | 0x20;  // Target Position (counts)
    ec_SDOwrite(s, 0x1600, 2, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x60FF << 16) | (0x00 << 8) | 0x20;  // Target Velocity
    ec_SDOwrite(s, 0x1600, 3, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6071 << 16) | (0x00 << 8) | 0x10;  // Target Torque
    ec_SDOwrite(s, 0x1600, 4, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6060 << 16) | (0x00 << 8) | 0x08;  // ModeOp
    ec_SDOwrite(s, 0x1600, 5, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x2000 << 16) | (0x00 << 8) | 0x08;  // Reserved padding
    ec_SDOwrite(s, 0x1600, 6, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    uint8_t n_rx = 6;
    ec_SDOwrite(s, 0x1600, 0, FALSE, sizeof(n_rx), &n_rx, EC_TIMEOUTSAFE);

    // 3. TxPDO 0x1A00 (inputs_t): 0x6041(16bit) + 0x6064(32bit) + 0x606C(32bit) + 0x6077(16bit) + 0x6061(8bit) + reserved(8bit) = 14 bytes
    ec_SDOwrite(s, 0x1A00, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6041 << 16) | (0x00 << 8) | 0x10;  // Statusword
    ec_SDOwrite(s, 0x1A00, 1, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6064 << 16) | (0x00 << 8) | 0x20;  // Actual Position (counts)
    ec_SDOwrite(s, 0x1A00, 2, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x606C << 16) | (0x00 << 8) | 0x20;  // Actual Velocity
    ec_SDOwrite(s, 0x1A00, 3, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6077 << 16) | (0x00 << 8) | 0x10;  // Actual Torque
    ec_SDOwrite(s, 0x1A00, 4, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x6061 << 16) | (0x00 << 8) | 0x08;  // ModeDisplay
    ec_SDOwrite(s, 0x1A00, 5, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    entry = ((uint32_t)0x2000 << 16) | (0x00 << 8) | 0x08;  // Reserved padding
    ec_SDOwrite(s, 0x1A00, 6, FALSE, sizeof(entry), &entry, EC_TIMEOUTSAFE);
    uint8_t n_tx = 6;
    ec_SDOwrite(s, 0x1A00, 0, FALSE, sizeof(n_tx), &n_tx, EC_TIMEOUTSAFE);

    // 4. 指派 SM2/SM3
    uint16_t rx_pdo = 0x1600, tx_pdo = 0x1A00;
    uint8_t one = 1;
    ec_SDOwrite(s, 0x1C12, 1, FALSE, sizeof(rx_pdo), &rx_pdo, EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C12, 0, FALSE, sizeof(one),   &one,    EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C13, 1, FALSE, sizeof(tx_pdo), &tx_pdo, EC_TIMEOUTSAFE);
    ec_SDOwrite(s, 0x1C13, 0, FALSE, sizeof(one),   &one,    EC_TIMEOUTSAFE);

    printf("[PDO] CSP Motor slave %d configured (0x607A/0x6064)\n", s);
}

// ========== 传感器 EEPROM 修复 (ec_config_map 之前) ==========
void EtherCATDriver::sensorEEPROMFix(int slave) {
    ec_slave[slave].SMtype[2] = 3;   // SM2 = Outputs
    ec_slave[slave].SMtype[3] = 4;   // SM3 = Inputs
    ec_slave[slave].Obits  = 48;     // 6 bytes
    ec_slave[slave].Ibits  = 208;    // 26 bytes
    ec_slave[slave].Obytes = 6;
    ec_slave[slave].Ibytes = 26;
    ec_slave[slave].configindex = 1;
    printf("[Fix] Sensor slave %d EEPROM corrected\n", slave);
}

// ========== 初始化 ==========
bool EtherCATDriver::init(const std::string& ifname) {
    strncpy(ifname_, ifname.c_str(), sizeof(ifname_) - 1);

    // 重置接力标志（支持 stop 后再次 init）
    rt_ready_to_take_over_.store(false);
    handoff_done_.store(false);

    if (!ec_init(ifname_)) {
        printf("[EtherCAT] Error: No socket on %s\n", ifname_);
        return false;
    }

    if (ec_config_init(FALSE) <= 0) {
        printf("[EtherCAT] Error: No slaves found\n");
        ec_close();
        return false;
    }

    printf("[EtherCAT] %d slaves found\n", ec_slavecount);
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("  Slave %d: %s  Man=0x%08X ID=0x%08X\n",
               i, ec_slave[i].name, ec_slave[i].eep_man, ec_slave[i].eep_id);
    }

    // 电机与传感器映射表
    int motor_slave_map[TOTAL_SLAVES] = {0};
    int sensor_slave_map[2] = {0};  // 0: 左臂传感器, 1: 右臂传感器

    // === 1. 扫描 Alias (0x2002) 并识别设备 ===
    for (int s = 1; s <= ec_slavecount; s++) {
        // 检查是否为 HANGKAI 六维力传感器 (Man=0x00000000 ID=0x00000259)
        if (ec_slave[s].eep_man == 0x00000000 && ec_slave[s].eep_id == 0x00000259) {
            sensorEEPROMFix(s);

            // 读取紧邻的下一个从站 (s+1) 的 Alias 来判断左右手
            uint16_t next_alias = 0;
            int size = sizeof(next_alias);
            if (s + 1 <= ec_slavecount) {
                ec_SDOread(s + 1, 0x2002, 0, FALSE, &size, &next_alias, EC_TIMEOUTSAFE);
            }

            if (next_alias >= 1 && next_alias <= 8) {
                sensor_slave_map[0] = s;  // 左手传感器 (arm 0)
                printf("  [Sensor] Slave %d mapped as Left Arm Sensor (arm 0, next motor alias=%d)\n", s, next_alias);
            } else if (next_alias >= 9 && next_alias <= 16) {
                sensor_slave_map[1] = s;  // 右手传感器 (arm 1)
                printf("  [Sensor] Slave %d mapped as Right Arm Sensor (arm 1, next motor alias=%d)\n", s, next_alias);
            } else {
                // 备用 fallback
                if (sensor_slave_map[0] == 0) {
                    sensor_slave_map[0] = s;
                    printf("  [Sensor] Slave %d fallback mapped as Left Arm Sensor (arm 0)\n", s);
                } else if (sensor_slave_map[1] == 0) {
                    sensor_slave_map[1] = s;
                    printf("  [Sensor] Slave %d fallback mapped as Right Arm Sensor (arm 1)\n", s);
                }
            }
            continue;
        }

        // 尝试读取电机的 Station Alias (0x2002:00)
        uint16_t alias = 0;
        int size = sizeof(alias);
        int ret = ec_SDOread(s, 0x2002, 0, FALSE, &size, &alias, EC_TIMEOUTSAFE);

        if (ret > 0 && alias > 0) {
            int motor_idx = -1;
            if (alias >= 1 && alias <= 8) {
                motor_idx = alias - 1;                // 左臂 1~8 -> 0~7
                printf("  [Motor] Slave %d -> Left Arm J%d (alias=%d, idx=%d)\n", s, alias, alias, motor_idx);
            } else if (alias >= 9 && alias <= 16) {
                motor_idx = 8 + (alias - 9);          // 右臂 9~16 -> 8~15
                printf("  [Motor] Slave %d -> Right Arm J%d (alias=%d, idx=%d)\n", s, alias - 8, alias, motor_idx);
            } else if (alias >= 17 && alias <= 18) {
                motor_idx = 16 + (alias - 17);        // 头部 17~18 -> 16~17
                printf("  [Motor] Slave %d -> Head J%d (alias=%d, idx=%d)\n", s, alias - 16, alias, motor_idx);
            } else if (alias >= 19 && alias <= 20) {
                motor_idx = 18 + (alias - 19);        // 躯干 19~20 -> 18~19
                printf("  [Motor] Slave %d -> Torso J%d (alias=%d, idx=%d)\n", s, alias - 18, alias, motor_idx);
            }

            if (motor_idx >= 0 && motor_idx < TOTAL_SLAVES) {
                motor_slave_map[motor_idx] = s;
            }
        }
    }

    // === 2. 电机 PDO 配置 (PREOP) ===
    for (int m = 0; m < TOTAL_SLAVES; m++) {
        if (motor_slave_map[m] > 0) {
            if (m >= 16 && m <= 19) {
                is_csp_motor_[m] = true;
                cspPDOConfig(motor_slave_map[m]);
            } else {
                is_csp_motor_[m] = false;
                motorPDOConfig(motor_slave_map[m]);
            }
        }
    }

    // === 3. IO 映射 + DC 同步 ===
    ecx_context.manualstatechange = 1;
    ec_config_map(&IOmap_);
    // 配置 EtherCAT 分布式时钟，Sync0 周期与主站 PDO 周期保持 1 ms。
    ec_configdc();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc)
            ec_dcsync0(i, 1, EC_CYCLETIME_US * 1000, 0);
    }

    // === 4. SAFE_OP ===
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("[EtherCAT] SAFE_OP reached\n");

    // === 4a. 在 mlockall 之前预创建 cycle_thread_ ===
    // 关键：std::thread 构造会分配 ~8MB 栈。如果在 mlockall(MCL_FUTURE) 之后创建，
    // 每页 4KB 栈分配都触发 page-fault locking，8MB 栈 = ~7ms 阻塞，期间无 PDO，
    // 从站 sync0 watchdog 超时退 OP。在 mlockall 之前创建则栈按正常速度分配，
    // mlockall(MCL_CURRENT) 之后会一次性锁定已存在的页（包括 cycle_thread_ 栈）。
    stop_cycle_thread_ = false;
    rt_ready_to_take_over_.store(false);
    handoff_done_.store(false);
    cycle_thread_ = std::thread(&EtherCATDriver::realtimeLoop, this);
    // cycle_thread_ 启动后会自行设置 SCHED_FIFO 并 spin-wait handoff_done_。

    // Move memory-locking work out of the OP-to-RT handoff.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        printf("[RT] mlockall failed before OP: %s\n", std::strerror(errno));
    } else {
        printf("[RT] mlockall enabled before OP\n");
    }

    // 主线程也提升为 SCHED_FIFO，保证 OP 握手 1ms 周期精度。
    {
        sched_param sp{};
        sp.sched_priority = kEcatPriority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            printf("[RT] main-thread SCHED_FIFO failed: %s\n", std::strerror(errno));
        } else {
            printf("[RT] main-thread SCHED_FIFO enabled, priority=%d\n", kEcatPriority);
        }
    }

    // === 5. 电机 PT/CSP 参数 (SAFE_OP) ===
    for (int m = 0; m < TOTAL_SLAVES; m++) {
        if (motor_slave_map[m] > 0) {
            motorPTConfig(motor_slave_map[m]);
        }
    }

    // === 6. 获取 IO 指针 (SAFE_OP 后 IOmap 已稳定，提前到 OP 之前) ===
    // 传感器指针
    for (int a = 0; a < 2; a++) {
        if (sensor_slave_map[a] > 0) {
            s_rx_[a] = (sensor_rx_pdo_t*)ec_slave[sensor_slave_map[a]].outputs;
            s_tx_[a] = (sensor_tx_pdo_t*)ec_slave[sensor_slave_map[a]].inputs;
        } else {
            s_rx_[a] = nullptr;
            s_tx_[a] = nullptr;
        }
    }

    // 电机指针 (区分 PT 电机与 CSP 电机)
    for (int m = 0; m < TOTAL_SLAVES; m++) {
        int s = motor_slave_map[m];
        if (s > 0) {
            if (is_csp_motor_[m]) {
                csp_rx_[m] = (csp_rx_t*)ec_slave[s].outputs;
                csp_tx_[m] = (csp_tx_t*)ec_slave[s].inputs;
                m_rx_[m] = nullptr;
                m_tx_[m] = nullptr;
            } else {
                m_rx_[m] = (motor_rx_pdo_t*)ec_slave[s].outputs;
                m_tx_[m] = (motor_tx_pdo_t*)ec_slave[s].inputs;
                csp_rx_[m] = nullptr;
                csp_tx_[m] = nullptr;
            }
        } else {
            m_rx_[m] = nullptr;
            m_tx_[m] = nullptr;
            csp_rx_[m] = nullptr;
            csp_tx_[m] = nullptr;
        }
    }

    // === 7. OP ===
    expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("[EtherCAT] Expected WKC=%d, requesting OP...\n", expectedWKC_);

    // 传感器只提供 SAFE_OP 下的过程数据，不参与 OP 状态切换。
    std::vector<int> op_slaves;
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (is_force_sensor_slave(i)) {
            printf("[State] Skip OP for force sensor slave %d\n", i);
            continue;
        }
        ec_slave[i].state = EC_STATE_OPERATIONAL;
        ec_writestate(i);
        op_slaves.push_back(i);
    }

    int chk = 200;
    bool all_op = false;
    do {
        const auto handshake_pdo_start = std::chrono::steady_clock::now();
        ec_send_processdata();
        int w = ec_receive_processdata(EC_CYCLETIME_US);
        wkc_ = w;
        handshake_last_wkc_.store(w);
        handshake_last_pdo_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            handshake_pdo_start.time_since_epoch()).count());
        ec_readstate();
        all_op = true;
        for (int slave : op_slaves) {
            if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
                all_op = false;
                break;
            }
        }
        // OP 握手阶段也按 1 ms 周期收发。
        std::this_thread::sleep_for(
            std::chrono::microseconds(EC_OP_HANDSHAKE_CYCLETIME_US));
    } while (chk-- && !all_op);

    if (!all_op) {
        printf("[EtherCAT] Error: Cannot reach OP\n");
        for (int slave : op_slaves) {
            if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
                printf("  Slave %d State=0x%02X Status=0x%04X: %s\n",
                       slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode,
                       ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
            }
        }
        ec_close();
        return false;
    }

    inOP_ = true;
    printf("[EtherCAT] OP reached\n");

    // === 8. 切换到 RT 线程 ===
    // cycle_thread_ 已在 mlockall 之前预创建，此时正在 spin-wait handoff_done_。
    // 等待 cycle_thread_ 报告就绪（已设置 SCHED_FIFO 并进入 spin），再让出 PDO 控制权。
    // 期间主线程继续发 PDO 保持从站 OP。
    {
        timespec handoff_next{};
        clock_gettime(CLOCK_MONOTONIC, &handoff_next);
        while (!rt_ready_to_take_over_.load(std::memory_order_acquire) &&
               !stop_cycle_thread_.load()) {
            const auto handshake_pdo_start = std::chrono::steady_clock::now();
            ec_send_processdata();
            int w = ec_receive_processdata(EC_CYCLETIME_US);
            wkc_ = w;
            handshake_last_wkc_.store(w);
            handshake_last_pdo_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                handshake_pdo_start.time_since_epoch()).count());
            add_ns(handoff_next, kEcatCycleNs);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &handoff_next, nullptr);
        }
    }

    // 最后一帧 PDO（确保接力点有最新数据）
    {
        const auto handshake_pdo_start = std::chrono::steady_clock::now();
        ec_send_processdata();
        int w = ec_receive_processdata(EC_CYCLETIME_US);
        wkc_ = w;
        handshake_last_wkc_.store(w);
        handshake_last_pdo_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            handshake_pdo_start.time_since_epoch()).count());
    }

    // cycle_thread_ 已就绪并在 spin-wait，设置 handoff_done_，cycle_thread_ 微秒级响应并接管。
    handoff_done_.store(true, std::memory_order_release);

    // 主线程降级回 SCHED_OTHER，把 RT CPU 让给 cycle_thread_。
    sched_param sp_other{};
    sp_other.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp_other);
    printf("[RT] main-thread downgraded to SCHED_OTHER\n");

    printf("[EtherCAT] Init complete, ready for control\n");
    return true;
}

void EtherCATDriver::realtimeLoop() {
    // cycle_thread_ 在 mlockall 之前创建，栈已分配好。
    // 这里设置 SCHED_FIFO（mlockall 已执行，但本线程栈已 mlockall(MCL_CURRENT) 锁定，
    // 无 page fault）。
    sched_param param{};
    param.sched_priority = kEcatPriority;
    const int sched_result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (sched_result != 0) {
        printf("[RT] SCHED_FIFO setup failed: %s\n", std::strerror(sched_result));
    } else {
        printf("[RT] SCHED_FIFO enabled, priority=%d\n", kEcatPriority);
    }

    // 通知主线程：本线程已就绪（SCHED_FIFO 已设置），可以放让 PDO 控制权。
    rt_ready_to_take_over_.store(true, std::memory_order_release);

    // Phase 1: spin-wait 等待主线程让出 PDO 控制权。
    // 主线程在 OP 握手成功后继续 1ms PDO 周期直到看到 rt_ready_to_take_over_，
    // 然后设置 handoff_done_。本线程微秒级响应，无缝接管。
    while (!handoff_done_.load(std::memory_order_acquire) &&
           !stop_cycle_thread_.load()) {
        // spin-wait (不 yield 不 sleep)
    }
    if (stop_cycle_thread_.load()) return;

    // Phase 2: 主线程已让出，立即开始 PDO 周期，无缝接管。
    cycle();

    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    add_ns(next, kEcatCycleNs);
    int64_t dc_integral = 0;
    int64_t dc_adjust_ns = 0;

    while (!stop_cycle_thread_.load()) {
        int ret;
        do {
            ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        } while (ret == EINTR && !stop_cycle_thread_.load());
        if (stop_cycle_thread_.load()) break;

        cycle();

        // ec_receive_processdata() 更新 ec_DCtime；用其相位修正下一周期。
        dc_adjust_ns = dc_phase_correction_ns(ec_DCtime, kEcatCycleNs, dc_integral);
        add_ns(next, kEcatCycleNs + dc_adjust_ns);
    }
}

// ========== 设置电机命令 ==========
void EtherCATDriver::setMotorCmd(int idx, float pos, float spd, float tor, float kp, float kd) {
    if (idx < 0 || idx >= TOTAL_SLAVES) return;
    if (!m_rx_[idx] && !csp_rx_[idx]) return;
    motor_cmd_pos_[idx].store(pos);
    motor_cmd_spd_[idx].store(spd);
    motor_cmd_tor_[idx].store(tor);
    motor_cmd_kp_[idx].store(kp);
    motor_cmd_kd_[idx].store(kd);
}

// ========== 获取电机反馈 ==========
void EtherCATDriver::getMotorFb(int idx, float& pos, float& spd, float& current, uint16_t& status) {
    if (idx < 0 || idx >= TOTAL_SLAVES) {
        pos = spd = current = 0;
        status = 0;
        return;
    }
    if (is_csp_motor_[idx]) {
        if (!csp_tx_[idx]) {
            pos = spd = current = 0;
            status = 0;
            return;
        }
        // 编码器计数值转换为弧度: 65536 pulses = 2 * PI rad
        pos = motor_fb_pos_[idx].load();
        spd = motor_fb_spd_[idx].load();
        current = motor_fb_cur_[idx].load();
        status = motor_fb_status_[idx].load();
    } else {
        if (!m_tx_[idx]) {
            pos = spd = current = 0;
            status = 0;
            return;
        }
        pos = motor_fb_pos_[idx].load();
        spd = motor_fb_spd_[idx].load();
        current = motor_fb_cur_[idx].load();
        status = motor_fb_status_[idx].load();
    }
}

// ========== 传感器数据 ==========
void EtherCATDriver::getSensorData(int arm, float& fx, float& fy, float& fz,
                                    float& mx, float& my, float& mz) {
    if (arm < 0 || arm > 1 || !s_tx_[arm]) {
        fx = fy = fz = mx = my = mz = 0;
        return;
    }
    fx = sensor_fb_[arm][0].load(); fy = sensor_fb_[arm][1].load();
    fz = sensor_fb_[arm][2].load(); mx = sensor_fb_[arm][3].load();
    my = sensor_fb_[arm][4].load(); mz = sensor_fb_[arm][5].load();
}

void EtherCATDriver::zeroSensor(int arm) {
    if (arm >= 0 && arm < 2 && s_rx_[arm]) {
        s_rx_[arm]->Res1 = 1;
    }
}

// ========== 夹爪控制 ==========
void EtherCATDriver::setGripperCmd(int arm, float pos, float spd, float tor, float kp, float kd) {
    if (arm < 0 || arm > 1) return;
    // Alias 映射规则: 左臂末端/夹爪(Alias 8) -> idx 7; 右臂末端/夹爪(Alias 16) -> idx 15
    constexpr int LEFT_GRIPPER_IDX  = 7;
    constexpr int RIGHT_GRIPPER_IDX = 15;
    int idx = (arm == 0) ? LEFT_GRIPPER_IDX : RIGHT_GRIPPER_IDX;
    setMotorCmd(idx, pos, spd, tor, kp, kd);
}

// ========== 单次 PDO 收发 (由 ROS loop 按 1kHz 调用) ==========
void EtherCATDriver::cycle() {
    if (!inOP_) return;
    cycle_count_++;

    static int enable_step[TOTAL_SLAVES] = {0};
    // 使能开始时锁定的目标位置 (抱闸释放期间保持不变, 防止位置环跟随下掉)
    static float enable_hold_pos[TOTAL_SLAVES] = {0};
    static bool  enable_hold_captured[TOTAL_SLAVES] = {false};

    // --- 传感器归零 (暂注释备用) ---
    // if (cycle_count_ == 100) {
    //     if (s_rx_[0]) { s_rx_[0]->Res1 = 1; s_rx_[0]->Res2 = 0; s_rx_[0]->Res3 = 0; }
    //     if (s_rx_[1]) { s_rx_[1]->Res1 = 1; s_rx_[1]->Res2 = 0; s_rx_[1]->Res3 = 0; }
    // } else if (cycle_count_ == 101) {
    //     if (s_rx_[0]) memset((void*)s_rx_[0], 0, sizeof(sensor_rx_pdo_t));
    //     if (s_rx_[1]) memset((void*)s_rx_[1], 0, sizeof(sensor_rx_pdo_t));
    // }

    // --- 电机 CiA402 使能 + 命令下发 (per-motor mask) ---
    for (int m = 0; m < TOTAL_SLAVES; m++) {
        if (is_csp_motor_[m]) {
            // ==================== CSP 模式 (头/腰) ====================
            if (!csp_rx_[m] || !csp_tx_[m]) continue;

            if (!motor_enable_.load() || !motor_enable_mask_[m].load()) {
                // 抱闸: 该电机关闭使能
                csp_rx_[m]->controlword = 0x0006;
                csp_rx_[m]->ModeOp = MODE_CSP;
                enable_step[m] = 0;
                continue;
            }

            uint16_t st = csp_tx_[m]->statusword;
            csp_rx_[m]->ModeOp = MODE_CSP;

            // 故障复位
            if ((st & 0x0008) && !(st & 0x0040)) {
                csp_rx_[m]->controlword = 0x0086;
                enable_step[m] = 0;
            } else if (enable_step[m] < EN_STEP * 6) {
                // CiA402 状态机切换
                if      (enable_step[m] < EN_STEP)     csp_rx_[m]->controlword = 0x0006;
                else if (enable_step[m] < EN_STEP * 2) csp_rx_[m]->controlword = 0x0086;
                else if (enable_step[m] < EN_STEP * 3) csp_rx_[m]->controlword = 0x0006;
                else if (enable_step[m] < EN_STEP * 4) csp_rx_[m]->controlword = 0x0007;
                else if (enable_step[m] < EN_STEP * 5) csp_rx_[m]->controlword = 0x000F;
                else                                    csp_rx_[m]->controlword = 0x001F;
                enable_step[m]++;
            } else {
                csp_rx_[m]->controlword = 0x001F;
            }

            // ROS2 目标位置转换 (弧度 -> 编码器脉冲数: 2*PI rad = 65536 counts)
            if (enable_step[m] < EN_STEP * 6) {
                // 使能过程中保持当前实际位置，防止跳变
                csp_rx_[m]->position = csp_tx_[m]->position;
                csp_rx_[m]->velocity = 0;
                csp_rx_[m]->torque   = 0;
            } else {
                int32_t target_counts = (int32_t)std::round((double)motor_cmd_pos_[m].load() * (65536.0 / (2.0 * M_PI)));
                csp_rx_[m]->position = target_counts;
                csp_rx_[m]->velocity = (int32_t)std::round((double)motor_cmd_spd_[m].load() * (65536.0 / (2.0 * M_PI)));
                csp_rx_[m]->torque   = (int16_t)std::round(motor_cmd_tor_[m].load() * 1000.0f);
            }
        } else {
            // ==================== PT 模式 (手臂) ====================
            if (!m_rx_[m] || !m_tx_[m]) continue;

            if (m < 16 && arm_shutdown_handoff_.load()) {
                m_rx_[m]->controlword = 0x0006;
                m_rx_[m]->KP = 0.0f;
                m_rx_[m]->KD = 0.0f;
                m_rx_[m]->target_position = 0.0f;
                m_rx_[m]->target_speed = 0.0f;
                m_rx_[m]->target_torque = 0.0f;
                enable_step[m] = 0;
                continue;
            }

            if (!motor_enable_.load() || !motor_enable_mask_[m].load()) {
                // 抱闸: 该电机关闭使能
                m_rx_[m]->controlword = 0x0006;
                enable_step[m] = 0;
                enable_hold_captured[m] = false;  // 重置, 下次使能时重新捕获
                continue;
            }

            uint16_t st = m_tx_[m]->statusword;
            // 故障复位
            if ((st & 0x0008) && !(st & 0x0040)) {
                m_rx_[m]->controlword = 0x0086;
                enable_step[m] = 0;
            } else if (enable_step[m] < EN_STEP * 6) {
                // 使能序列开始时锁定当前位置作为 hold_position,
                // 整个序列期间保持不变, 防止位置环跟随下掉.
                if (!enable_hold_captured[m]) {
                    enable_hold_pos[m] = m_tx_[m]->actual_position;
                    enable_hold_captured[m] = true;
                }
                // ★ 方案 F: 若驱动器已进入 Operation Enabled (bit2=1),
                //   立即跳到 ROS 命令阶段, 下发 motor_cmd_tor_ (阻抗力矩含 g_sub_).
                if ((st & 0x0004) && enable_step[m] < EN_STEP * 5) {
                    // printf("[DBG-CIA] m=%d OpEnabled at step=%d, skip to ROS cmd (st=0x%04X)\n",
                    //        m, enable_step[m], st);
                    enable_step[m] = EN_STEP * 6;
                }
                // CiA402: 06 → 86 → 06 → 07 → 0F → 1F
                uint16_t cw_prev = m_rx_[m]->controlword;
                if      (enable_step[m] < EN_STEP)     m_rx_[m]->controlword = 0x0006;
                else if (enable_step[m] < EN_STEP * 2) m_rx_[m]->controlword = 0x0086;
                else if (enable_step[m] < EN_STEP * 3) m_rx_[m]->controlword = 0x0006;
                else if (enable_step[m] < EN_STEP * 4) m_rx_[m]->controlword = 0x0007;
                else if (enable_step[m] < EN_STEP * 5) m_rx_[m]->controlword = 0x000F;
                else                                    m_rx_[m]->controlword = 0x001F;
                // if (cw_prev != m_rx_[m]->controlword || (st & 0x0004)) {
                //     printf("[DBG-CIA] m=%d step=%d cw=0x%04X st=0x%04X op_en=%d tor_ff=%.3f KP=%.3f KD=%.3f\n",
                //            m, enable_step[m], m_rx_[m]->controlword, st,
                //            (st & 0x0004) ? 1 : 0, gravity_ff_[m].load(),
                //         motor_cmd_kp_[m].load(),motor_cmd_kd_[m].load());
                // }
                enable_step[m]++;
            } else {
                m_rx_[m]->controlword = 0x001F;
            }

            // ROS2 下发命令 (使能中先给零，使能后切 ROS 命令)
            if (enable_step[m] < EN_STEP * 6) {
                // 方案 E++: 使能期间启用位置环 (保持当前位置) + 重力前馈,
                //   防止 PT 模式下 KP=0 时 target_torque 被驱动器忽略.
                //   target_position 用使能开始时锁定的位置 (hold_position),
                //   若机械臂下掉, 位置误差产生回弹力矩, 阻止继续下掉.
                m_rx_[m]->KP = motor_cmd_kp_[m].load();
                m_rx_[m]->KD = motor_cmd_kd_[m].load();
                m_rx_[m]->target_position = enable_hold_pos[m];
                m_rx_[m]->target_speed    = 0.0f;
                m_rx_[m]->target_torque   = gravity_ff_[m].load();
            } else {
                m_rx_[m]->KP              = motor_cmd_kp_[m].load();
                m_rx_[m]->KD              = motor_cmd_kd_[m].load();
                m_rx_[m]->target_position = motor_cmd_pos_[m].load();
                m_rx_[m]->target_speed    = motor_cmd_spd_[m].load();
                m_rx_[m]->target_torque   = motor_cmd_tor_[m].load();
            }
        }
    }

    // --- PDO 收发 ---
    const auto io_start = std::chrono::steady_clock::now();
    if (pdo_timing_initialized_) {
        const auto period_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                io_start - last_pdo_send_time_).count());
        ++timing_cycles_;
        timing_period_min_us_ = timing_period_min_us_ == 0
            ? period_us : std::min(timing_period_min_us_, period_us);
        timing_period_max_us_ = std::max(timing_period_max_us_, period_us);
        timing_period_sum_us_ += period_us;
        if (period_us > EC_CYCLETIME_US) ++timing_over_1ms_;
    } else {
        pdo_timing_initialized_ = true;
    }
    last_pdo_send_time_ = io_start;

    ec_send_processdata();
    wkc_ = ec_receive_processdata(EC_CYCLETIME_US);
    const auto io_end = std::chrono::steady_clock::now();
    if (timing_io_cycles_ == 0) {
        first_rt_wkc_.store(wkc_.load());
        first_rt_pdo_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            io_start.time_since_epoch()).count());
    }
    if (wkc_.load() == expectedWKC_ && expectedWKC_ > 0) {
        last_good_pdo_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            io_end.time_since_epoch()).count());
    } else {
        ++bad_wkc_cycles_;
    }

    // 只在实时线程读取 PDO，并发布到无锁反馈快照。
    for (int m = 0; m < TOTAL_SLAVES; ++m) {
        if (is_csp_motor_[m] && csp_rx_[m])
            motor_sent_cw_[m].store(csp_rx_[m]->controlword);
        else if (!is_csp_motor_[m] && m_rx_[m])
            motor_sent_cw_[m].store(m_rx_[m]->controlword);
        if (is_csp_motor_[m] && csp_tx_[m]) {
            motor_fb_pos_[m].store(
                static_cast<float>(csp_tx_[m]->position * (2.0 * M_PI / 65536.0)));
            motor_fb_spd_[m].store(
                static_cast<float>(csp_tx_[m]->velocity * (2.0 * M_PI / 65536.0)));
            motor_fb_cur_[m].store(static_cast<float>(csp_tx_[m]->torque) * 0.001f);
            motor_fb_status_[m].store(csp_tx_[m]->statusword);
        } else if (!is_csp_motor_[m] && m_tx_[m]) {
            motor_fb_pos_[m].store(m_tx_[m]->actual_position);
            motor_fb_spd_[m].store(m_tx_[m]->actual_speed);
            motor_fb_cur_[m].store(m_tx_[m]->actual_current);
            motor_fb_status_[m].store(m_tx_[m]->statusword);
        }
    }
    for (int a = 0; a < 2; ++a) {
        if (s_tx_[a]) {
            sensor_fb_[a][0].store(s_tx_[a]->Fx);
            sensor_fb_[a][1].store(s_tx_[a]->Fy);
            sensor_fb_[a][2].store(s_tx_[a]->Fz);
            sensor_fb_[a][3].store(s_tx_[a]->Mx);
            sensor_fb_[a][4].store(s_tx_[a]->My);
            sensor_fb_[a][5].store(s_tx_[a]->Mz);
        }
    }
    ++completed_pdo_cycles_;

    const auto io_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            io_end - io_start).count());
    ++timing_io_cycles_;
    timing_io_min_us_ = timing_io_min_us_ == 0
        ? io_us : std::min(timing_io_min_us_, io_us);
    timing_io_max_us_ = std::max(timing_io_max_us_, io_us);
    timing_io_sum_us_ += io_us;

}

void EtherCATDriver::printCommunicationStatus() const {
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const int64_t good_ns = last_good_pdo_ns_.load();
    const int64_t first_ns = first_rt_pdo_ns_.load();
    const int64_t last_ns = handshake_last_pdo_ns_.load();
    const double handoff_ms = first_ns > 0 && last_ns > 0
        ? (first_ns - last_ns) / 1e6 : -1.0;
    printf("[ECAT-MON] pdo=%llu WKC=%d/%d bad_wkc=%llu good_age_ms=%.1f "
           "handshake_wkc=%d first_rt_wkc=%d handoff_ms=%.3f "
           "L4{cw=0x%04X sw=0x%04X pos=%.6f cur=%.3f} "
           "R4{cw=0x%04X sw=0x%04X pos=%.6f cur=%.3f}\n",
           static_cast<unsigned long long>(completed_pdo_cycles_.load()),
           wkc_.load(), expectedWKC_,
           static_cast<unsigned long long>(bad_wkc_cycles_.load()),
           good_ns > 0 ? (now_ns - good_ns) / 1e6 : -1.0,
           handshake_last_wkc_.load(), first_rt_wkc_.load(), handoff_ms,
           static_cast<unsigned>(motor_sent_cw_[3].load()),
           static_cast<unsigned>(motor_fb_status_[3].load()),
           motor_fb_pos_[3].load(), motor_fb_cur_[3].load(),
           static_cast<unsigned>(motor_sent_cw_[11].load()),
           static_cast<unsigned>(motor_fb_status_[11].load()),
           motor_fb_pos_[11].load(), motor_fb_cur_[11].load());
}

// ========== 停止 ==========
void EtherCATDriver::stop() {
    running_ = false;
    stop_cycle_thread_ = true;
    if (cycle_thread_.joinable()) cycle_thread_.join();
    if (timing_cycles_ > 0) {
        printf("[ECAT-TIMING] total=%llu period_us[min/avg/max]=%llu/%.1f/%llu "
               "over_1ms=%llu io_us[min/avg/max]=%llu/%.1f/%llu\n",
               static_cast<unsigned long long>(timing_cycles_),
               static_cast<unsigned long long>(timing_period_min_us_),
               static_cast<double>(timing_period_sum_us_) / timing_cycles_,
               static_cast<unsigned long long>(timing_period_max_us_),
               static_cast<unsigned long long>(timing_over_1ms_),
               static_cast<unsigned long long>(timing_io_min_us_),
               static_cast<double>(timing_io_sum_us_) / timing_io_cycles_,
               static_cast<unsigned long long>(timing_io_max_us_));
    }
    if (inOP_) {
        // Thread is joined: no concurrent SOEM access. Read AL before leaving OP.
        printCommunicationStatus();
        ec_readstate();
        for (int s = 1; s <= ec_slavecount; ++s) {
            const auto expected = is_force_sensor_slave(s)
                ? EC_STATE_SAFE_OP : EC_STATE_OPERATIONAL;
            if (ec_slave[s].state != expected || ec_slave[s].ALstatuscode != 0) {
                printf("[ECAT-STOP] slave=%d state=0x%02X AL=0x%04X: %s\n",
                       s, ec_slave[s].state, ec_slave[s].ALstatuscode,
                       ec_ALstatuscode2string(ec_slave[s].ALstatuscode));
            }
        }
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        ec_close();
        inOP_ = false;
        printf("[ECAT-STOP] PDO thread joined, INIT requested, socket closed; brake state not verified\n");
    }
}

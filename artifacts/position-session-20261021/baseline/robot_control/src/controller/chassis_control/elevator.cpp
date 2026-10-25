#include "controller/chassis_control/elevator.h"

// CRC16 计算工具函数 (Modbus)
uint16_t calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < data.size(); ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if ((crc & 1) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

//--------------------底盘通讯类--------------------

ElevatorController::ElevatorController(SerialPort& sp) : serial(sp), currentHeight(0), presetHeight(-1) {
    // 优先使用用户家目录下的隐藏目录，避免权限问题与相对路径不确定性
    const char* home = std::getenv("HOME");
    if (home) {
        std::filesystem::path dir = std::filesystem::path(home) / ".lifter";
        try {
            std::filesystem::create_directories(dir);
        } catch (...) {
            // 如果目录创建失败，回退到当前目录
        }
        presetFile = (std::filesystem::path(home) / ".lifter" / "preset_position.txt").string();
    } else {
        // 兜底
        presetFile = "preset_position.txt";
    }        
    loadPreset();
}

void ElevatorController::sendCommand(std::vector<uint8_t> cmd) {
    uint16_t crc = calculateCRC(cmd);
    cmd.push_back(crc & 0xFF);        
    cmd.push_back((crc >> 8) & 0xFF); 

    serial.sendData(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
}

// ---------------------------------------------------------
// 动作指令
// ---------------------------------------------------------

//上升
void ElevatorController::moveUp() {
    // 原始指令(不带CRC): 01 06 00 01 00 02
    sendCommand({0x01, 0x06, 0x00, 0x01, 0x00, 0x02});
}

//下降
void ElevatorController::moveDown() {
    // 原始指令(不带CRC): 01 06 00 01 00 04
    sendCommand({0x01, 0x06, 0x00, 0x01, 0x00, 0x04});
}

//停止
void ElevatorController::stop() {
    // 原始指令(不带CRC): 01 06 00 01 00 01
    sendCommand({0x01, 0x06, 0x00, 0x01, 0x00, 0x01});
}

//回原点
void ElevatorController::reset() {
    std::cout << "[动作] 重置/回原点" << std::endl;
    // 原始指令(不带CRC): 01 06 00 01 00 08
    sendCommand({0x01, 0x06, 0x00, 0x01, 0x00, 0x08});
}

// ---------------------------------------------------------
// 读取和状态管理 (修改为带返回值的形式，方便内部调用)
// ---------------------------------------------------------

bool ElevatorController::queryHeight(bool print) {
    // 原始指令(不带CRC): 01 03 00 02 00 01
    sendCommand({0x01, 0x03, 0x00, 0x02, 0x00, 0x01});
    
    // 适当增加等待时间，有些设备反应慢
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
    std::vector<uint8_t> response = serial.readData();

    // 调试：打印收到的原始 HEX 数据
    if (print) {
        if (response.empty()) {
            std::cout << ">>> 调试警告: 串口未收到任何数据 (0 bytes)。" << std::endl;
        } else {
            std::cout << ">>> 调试-收到数据[" << response.size() << "字节]: ";
            for (auto b : response) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
            }
            std::cout << std::dec << std::endl;
        }
    }

    if (response.size() >= 5) {
        size_t len = response.size();
        
        // 按照倒数解析：
        // 倒数第4位 = 数据高位 (High)
        // 倒数第3位 = 数据低位 (Low)
        int high = response[len - 4];
        int low = response[len - 3];
        int val = (high << 8) | low;
        
        currentHeight = val;
        if (print) std::cout << "当前解析高度: " << currentHeight << " mm" << std::endl;
        return true;
    } 
    
    if (print && !response.empty()) std::cout << "读取高度失败 (数据长度不够)" << std::endl;
    return false;
}

// ---------------------------------------------------------
// 记忆点位功能
// ---------------------------------------------------------

bool ElevatorController::saveCurrentAsPreset() {
    std::lock_guard<std::mutex> lk(fileMutex);
    // 必须先确信 currentHeight 是最新的
    if (!queryHeight(true)) {
        std::cout << ">>> [错误] 读取当前高度失败，无法保存预设。" << std::endl;
        return false;
    }

    presetHeight = currentHeight;

    try {
        std::ofstream outfile(presetFile);
        if (!outfile.is_open()) {
            std::cout << ">>> [错误] 无法打开记忆文件（路径: " << presetFile << "）" << std::endl;
            return false;
        }
        outfile << presetHeight;
        outfile.close();
        std::cout << ">>> [成功] 已将当前位置 " << presetHeight << " 设为记忆点 (文件: " << presetFile << ")" << std::endl;
        return true;
    } catch (const std::exception &e) {
        std::cout << ">>> [异常] 保存记忆文件失败: " << e.what() << std::endl;
        return false;
    }
}

void ElevatorController::loadPreset() {
    std::lock_guard<std::mutex> lk(fileMutex);

    if (std::filesystem::exists(presetFile)) {
        std::ifstream infile(presetFile);
        if (infile.is_open()) {
            infile >> presetHeight;
            infile.close();
            std::cout << ">>> 系统初始化: 读取到记忆点位 " << presetHeight << " (文件: " << presetFile << ")" << std::endl;
            return;
        }
    }
    std::cout << ">>> 系统初始化: 无记忆点位 (文件: " << presetFile << ")" << std::endl;
    presetHeight = -1; // -1 表示无效
}

// ---------------------------------------------------------
// 自动归位逻辑
// ---------------------------------------------------------

void ElevatorController::autoRunToPreset() {
    if (presetHeight < 0) {
        std::cout << ">>>无有效记忆点，跳过自动运行。" << std::endl;
        return;
    }

    std::cout << ">>> 正在尝试自动运行到记忆点: " << presetHeight << " ..." << std::endl;
    
    // 1. 先读一次当前位置
    if (!queryHeight(false)) {
        std::cout << ">>> [错误] 无法读取当前位置，取消自动运行。" << std::endl;
        return;
    }

    const int ERROR_MARGIN = 2; // 最终允许的静态误差范围
    // 惯性补偿：提前 N mm 发送停止指令
    // 之前 10mm 导致提前停了 5mm，现在修正为 3mm
    const int STOP_OFFSET = 3; 

    int diff = presetHeight - currentHeight;

    if (std::abs(diff) <= ERROR_MARGIN) {
        std::cout << ">>> 当前已在记忆点附近，无需移动。" << std::endl;
        return;
    }

    // 2. 判断方向并启动
    // 记录一下我们是在往上走还是往下走
    bool movingUp = (diff > 0);

    if (movingUp) {
        moveUp(); 
    } else {
        moveDown();
    }

    // 3. 闭环轮询监测
    std::cout << ">>> 自动运行中... (按 Ctrl+C 紧急停止)" << std::endl;
    while (true) {
        // 提高检测频率到 20Hz (50ms)，提高控制精度
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
        if (!queryHeight(false)) continue;

        std::cout << "\r 目标: " << presetHeight << " | 当前: " << currentHeight << "   " << std::flush;

        bool shouldStop = false;
        
        if (movingUp) {
            // 向上走：如果 (目标 - 当前) < 提前量，就停
            if (currentHeight >= (presetHeight - STOP_OFFSET)) shouldStop = true;
        } else {
            // 向下走：如果 (当前 - 目标) < 提前量，就停
            if (currentHeight <= (presetHeight + STOP_OFFSET)) shouldStop = true;
        }

        if (shouldStop) {
            stop();
            std::cout << "\n>>> [刹车] 触发提前停止 (补偿=" << STOP_OFFSET << "mm)" << std::endl;
            
            // 停稳后再读一次
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            queryHeight(true);
            break;
        }
    }
}

// 供外部读取当前高度的线程安全访问器（简单封装，当前实现为非并发访问为主）
int ElevatorController::getCurrentHeight(){
    return currentHeight;
}

// 新增：线程安全读取保存的 presetHeight
int ElevatorController::getPresetHeight() {
    std::lock_guard<std::mutex> lk(fileMutex);
    return presetHeight;
}

//------------------------------------------------
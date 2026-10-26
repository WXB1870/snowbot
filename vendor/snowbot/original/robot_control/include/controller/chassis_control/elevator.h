#ifndef __elevator__H__
#define __elevator__H__

#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <filesystem>   // 新增
#include <mutex>        // 新增
#include <cstdlib>      // getenv

#include "controller/chassis_control/serial_port.h"

//升降台通讯代码

// CRC16 计算工具函数 (Modbus)
uint16_t calculateCRC(const std::vector<uint8_t>& data);

class ElevatorController {
    private:
        SerialPort& serial;
        int currentHeight;  // 当前实时高度
        int presetHeight;   // 记忆的目标点位
        std::string presetFile;
        std::mutex fileMutex;   // 保护文件读写
    
    public:
        ElevatorController(SerialPort& sp);

        // 辅助：发送带 CRC 的命令 (保持不变)
        void sendCommand(std::vector<uint8_t> cmd);

        // ---------------------------------------------------------
        // 动作指令
        // ---------------------------------------------------------
        //上升
        void moveUp();

        //下降
        void moveDown();

        //停止
        void stop();

        //重置/回原点
        void reset();

        // ---------------------------------------------------------
        // 读取和状态管理 (修改为带返回值的形式，方便内部调用)
        // ---------------------------------------------------------

        bool queryHeight(bool print = true);

        // ---------------------------------------------------------
        // 记忆点位功能
        // ---------------------------------------------------------

        //记录点位功能
        bool saveCurrentAsPreset();

        //读取记录点位
        void loadPreset();

        // ---------------------------------------------------------
        // 自动归位逻辑
        // ---------------------------------------------------------

        void autoRunToPreset();
    
        // 供外部读取当前高度的线程安全访问器（简单封装，当前实现为非并发访问为主）
        int getCurrentHeight();

        // 新增：线程安全读取保存的 presetHeight
        int getPresetHeight();

};

#endif
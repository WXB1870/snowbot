#include "controller/chassis_control/serial_port.h"


bool SerialPort::openPort(int baudRate) {
    // 使用 O_NDELAY 打开以防止 DCD 信号导致的阻塞，随后立即恢复为阻塞模式
    fd = open(deviceName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "无法打开串口: " << deviceName << " 错误: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 恢复为阻塞模式 (Blocking Mode)
    // 这样 read() 会等待数据到达或超时，而不是立即返回
    fcntl(fd, F_SETFL, 0);

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "无法获取串口属性" << std::endl;
        return false;
    }

    // 设置波特率
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    // --- 配置 8N1 (8数据位, 无校验, 1停止位) ---
    tty.c_cflag &= ~PARENB; // 无校验
    tty.c_cflag &= ~CSTOPB; // 1停止位
    tty.c_cflag &= ~CSIZE;  // 清除大小位
    tty.c_cflag |= CS8;     // 8数据位

    // --- 基础控制设置 ---
    tty.c_cflag &= ~CRTSCTS;         // 禁用硬件流控
    tty.c_cflag |= (CLOCAL | CREAD); // 忽略调制解调器控制线，启用接收器

    // --- 原始模式配置 (Raw Mode) ---
    // 禁用所有特殊的输入处理（如将回车转换行等）
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    // 禁用所有输出处理
    tty.c_oflag &= ~OPOST;
    // 禁用回显、规范模式、信号字符等
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // --- 读取超时设置 ---
    // VTIME: 读取超时时间 (单位: 0.1秒)。5 = 0.5秒
    // VMIN:  最小读取字符数。0 = 只要有数据就返回，没数据就等超时
    tty.c_cc[VTIME] = 5; 
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "无法设置串口属性" << std::endl;
        return false;
    }

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);
    std::cout << "串口 " << deviceName << " 打开成功。" << std::endl;
    return true;
}

void SerialPort::closePort(){
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

void SerialPort::sendData(const std::vector<uint8_t>& data) {
    if (fd == -1) return;
    
    int count = write(fd, data.data(), data.size());
    if (count < 0) {
        std::cerr << "发送失败: " << strerror(errno) << std::endl;
    } else {
        // Debug: 打印发送的内容
        // std::cout << "已发送: ";
        // for(auto b : data) printf("%02X ", b);
        // std::cout << std::endl;
    }
}

std::vector<uint8_t> SerialPort::readData(int bufferSize) {
    std::vector<uint8_t> buffer(bufferSize);
    if (fd == -1) return {};

    int n = read(fd, buffer.data(), bufferSize);
    if (n > 0) {
        buffer.resize(n);
        return buffer;
    }
    return {};
}
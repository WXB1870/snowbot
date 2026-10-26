#ifndef __serial_port__H__
#define __serial_port__H__

#include <string>
#include <vector>
#include <iostream>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <cstring>
#include <cerrno>

//升降台通讯基类代码

class SerialPort {
public:
    int fd;
    std::string deviceName;

    SerialPort(const std::string& device) : fd(-1), deviceName(device) {}

    ~SerialPort() {
        closePort();
    }

    bool openPort(int baudRate = B9600);
    void closePort();
    void sendData(const std::vector<uint8_t>& data);
    std::vector<uint8_t> readData(int bufferSize = 256);
};

#endif
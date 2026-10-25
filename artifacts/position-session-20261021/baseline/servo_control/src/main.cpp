#include <iostream>
#include <cstdio>
#include <csignal>
#include <atomic>
#include <pthread.h>
#include "rclcpp/rclcpp.hpp"

#include "ros_controller/rosLoop.h"

std::atomic<bool> sigInterrupt(false);

static void signalHandler(int sig){
    (void) sig;
    // 仅设置原子标志, 由主循环检查后进入关机回零流程.
    // 注意: 此函数运行在 SIGINT 信号处理上下文中, 不可调用非异步信号安全
    // 函数 (printf/malloc/ROS 日志等).
    sigInterrupt.store(true, std::memory_order_release);
}

int main(int argc, char ** argv){
    (void) argc;
    (void) argv;

    std::cout << "机器人控制程序开启" << std::endl;
    std::cout << "===================================" << std::endl;

    // === 信号处理: 用 sigaction 直接掌管 SIGINT/SIGTERM, 不让 rclcpp 的
    // sigwait 线程截获信号 ===
    //
    // 之前用 std::signal() 在 rclcpp::init 之前注册, 会被 rclcpp 的 sigwait
    // 机制覆盖: rclcpp::init 会在主线程屏蔽 SIGINT 并起一个独立线程用
    // sigwait 接收信号. 这样 SIGINT 来时被 sigwait 线程捕获, main.cpp 的
    // signalHandler 永远不会被调用, sigInterrupt 不被设置, 主循环无法
    // 进入关机回零流程 (表现为 Ctrl+C 偶尔无响应, 需要按多次才生效).
    //
    // 修复: rclcpp::init 之后用 sigaction 重新安装 SIGINT/SIGTERM 处理器,
    // 这样会覆盖 rclcpp 在主线程上设置的 sigwait 屏蔽, 让 SIGINT 直接调用
    // signalHandler 设置 sigInterrupt = true.
    //
    // 同时设置 SA_RESTART, 让被 SIGINT 中断的系统调用 (clock_nanosleep,
    // recv 等) 自动重启, 不影响 EtherCAT 实时线程的 PDO 周期稳定性.
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);

    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;  // 让被中断的系统调用自动重启
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    auto node = std::make_shared<ServoNode>();

    node->run();
    EtherCAT_Stop();
    rclcpp::shutdown();

    return 0;
}

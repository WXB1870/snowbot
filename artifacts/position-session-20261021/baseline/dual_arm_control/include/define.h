#ifndef __KERNEL_DEFINE__H__
#define __KERNEL_DEFINE__H__

#include <stdint.h>
#include <iostream>
#include <string>
//#include "interface/State.h"
#include <atomic>
#include <csignal>
#define real_T double 
//extern void throwFatalError(std::string err);
extern std::string __ModuleName__;
extern std::atomic<bool> sigInterrupt;
//extern void handleSigInterrupt(int32_t signal);


#define   BASE_CONFIG_PATH   ("/home/fun/singlejoint-test/src/slrjnttest/config/")
//#define SINGLE_LEG_TEST_MODE  true
#undef SINGLE_LEG_TEST_MODE
#define SINGLE_LEG_TEST_WITH_ROUTER true
#undef SINGLE_LEG_TEST_WITH_ROUTER

//#define TEST_WITH_NO_WAIST
#undef  TEST_WITH_NO_WAIST

#define DEBUGMODE true

#define ROBOT_LEG_VER  2



#define ROBOT_INFO(x)                                           \
    do                                                          \
    {                                                           \
        std::cout << "\033[37m\033[1m[" << __ModuleName__ << "]\033[0m" \
                << "\033[37m\033[1m[信息]\033[0m "         \
                << "\033[37m" << x << "\033[0m" << std::endl; \
    } while (false)

#define ROBOT_SUCCESS(x)                                        \
    do                                                          \
    {                                                           \
        std::cout << "\033[32m\033[1m[" << __ModuleName__ << "]\033[0m" \
                << "\033[32m\033[1m[成功]\033[0m "         \
                << "\033[32m" << x << "\033[0m" << std::endl; \
    } while (false)

#define ROBOT_WARN(x)                                           \
    do                                                          \
    {                                                           \
        std::cout << "\033[33m\033[1m[" << __ModuleName__ << "]\033[0m" \
                << "\033[33m\033[1m[警告]\033[0m "         \
                << "\033[33m" << x << "\033[0m" << std::endl; \
    } while (false)

#define ROBOT_ERROR(x)                                          \
    do                                                          \
    {                                                           \
        std::cout << "\033[31m\033[1m[" << __ModuleName__ << "]\033[0m" \
                << "\033[31m\033[1m[错误]\033[0m "         \
                << "\033[31m" << x << "\033[0m" << std::endl; \
    } while (false)

#define ROBOT_FATAL(x)                                          \
    do                                                          \
    {                                                           \
        std::cout << "\033[35m\033[1m[" << __ModuleName__ << "]\033[0m" \
                << "\033[35m\033[1m[致命]\033[0m "         \
                << "\033[35m" << x << "\033[0m" << std::endl; \
    } while (false)

#define ROBOT_DEBUG(x)                                              \
    do                                                              \
    {                                                               \
        if (DEBUGMODE == true)                                     \
        {                                                           \
            std::cout << "\033[36m\033[1m[" << __ModuleName__ << "]\033[0m" \
                    << "\033[36m\033[1m[调试]\033[0m "         \
                    << "\033[36m" << x << "\033[0m" << std::endl; \
        }                                                           \
    } while (false)

#endif // __KERNEL_DEFINE__H__

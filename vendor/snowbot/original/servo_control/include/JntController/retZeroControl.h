#ifndef __retZeroControl__H__
#define __retZeroControl__H__

#include <iostream>
#include <string>
#include <vector>

#include "ros_controller/rosLoopMsgDefine.h"
#include "servo_control/zero_pos_config.h"

/*插补轨迹结果参数*/
struct TrajResult {
    std::vector<float> positionIncrements;
    std::vector<float> VelocityTraj;
};
/*轨迹输出参数*/
struct TrajCtlOutp{
    std::vector<double> lArm_desPos;
    std::vector<double> lArm_desVel;
    std::vector<double> lArm_Kp;
    std::vector<double> lArm_Kd;

    std::vector<double> rArm_desPos;
    std::vector<double> rArm_desVel;
    std::vector<double> rArm_Kp;
    std::vector<double> rArm_Kd;

    bool ifOutpReachEnd;
};

//关节位置初始化
class JointPos_Init{
    private:

        //*************************
        //插补用变量与函数
        float retZeroTime = 2.0;
        float control_T = TEMPTIME;
        TrajResult dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase = 1.2);
        //*************************
        //插补轨迹储存
        std::vector<TrajResult> lArm_JntTraj;
        std::vector<TrajResult> rArm_JntTraj;
        int trajLen = 0;
        //*************************
        //循环控制变量
        int temp_times = 0;
    public:
        //插补函数
        void retZero_runin(std::vector<double> lArm_stPos, std::vector<double> rArm_stPos);

        //输出函数
        TrajCtlOutp retZero_trajOutp();
};

#endif
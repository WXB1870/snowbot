#ifndef __xrTelepercationCom__H__
#define __xrTelepercationCom__H__

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <unistd.h> // for read()
#include <sys/select.h> // for select()

#include "robot_control/msgType.h"
#include "controller/upper_Transmit.h"

/*
        by 霜云喵
==========================
遥操作通讯代码
==========================

==========================
*/
extern std::vector<double> dbg_reserve;

//------------------------------------------------------------
//辅助类：插补器
/*插补器*/
class stTraj_runiner {
    /*插补器*/
    /*注意总运动时间必须是control_T的整数倍*/
    private:
        float control_T = TempTime;//控制周期

        float lastTempPosOutp = 0.0f;
        float lastTempVelOutp = 0.0f;
    public:
        /**拟合代码初始化代码
         * deg -- 样条阶数
         * bar_len -- 窗口长度
        */

        //点位插补
        TrajResult dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase = 1.2);
        /*备注：最后一个参量的含义为加速时间占到达总位移一半时的总运动时间的比例，这个数值必须大于1*/

        //辅助函数 -- 输出最后一轮的速度
        //上述计算过程中的mposDes指的是相对于初始位置的位置变化量，因此这里引入初始位置将位置量还原
        std::pair<float, float> lastPosVelGet(float posSt = 0.0f);
};

/**/
//卡尔曼滤波器
class kalman_filter_v{
    private:
        double outvar[2];   //卡尔曼滤波器输出
        double Pmtrx[2][2]; //状态估计误差协方差矩阵
        double Qgain;   //噪声增益（用于调整过程噪声协方差矩阵Q的影响，该项越大滤波越强）
        double Rgain;   //测量噪声增益（用于调整测量噪声协方差矩阵的影响）
        
        //初始化状态估计协方差矩阵
        double P0mtrx[2][2]={{1000000.0f, 0.0f}, {0.0f, 1000000.0f}};
        //状态转移矩阵及其转置
        double FAImtrx[2][2] = {{1.0, 0.001}, {0.0, 1.0}};
        double FAImtrxT[2][2] = {{1.0,0.0}, {0.001, 1.0}};
        //过程噪声协方差矩阵
        double Qmtrx[2][2] = {{0.001*0.001*0.001/3.0f, 0.001*0.001*0.5f}, {0.001*0.001*0.5f, 0.001}};

        //中间变量
        double midx[2];  //预测的状态值
        double tempMtrx[2][2]; //临时矩阵
        double FAIPFAIT[2][2]; // 预测的状态估计误差协方差矩阵
        double Pk_[2][2]; // 更新后的状态估计误差协方差矩阵
        double Kgain[2]; // 卡尔曼增益

        //---------------成员函数---------------
        //自定义矩阵乘法
        void mMatrixMul(double a[],double b[],int m,int n,int k,double c[]);

    public:
        void FilterInit(double inputVar, double mQgain, double mRgain);     //初始化函数
        double FilteredResCal(double inputvar);      //滤波结果计算函数

};
/**/

//------------------------------------------------------------
//通讯核心代码
class xr_telepercation_ctl{
    private:

        //过程变量（最终的输出六维数组）
        std::vector<double> lArm_end_pos_rpy;
        std::vector<double> rArm_end_pos_rpy;

        //控制变量
        dataSendWrap mdataSend;
        ctlDesMsgWrap ctlStMsg;                     //初始位置包装
        ctlDesMsgWrap ctlDesMsg;                    //目标位置包装

        //--------------位姿初始化部分--------------
        std::vector<ctlDesMsgWrap> ctlStPos_traj;   //到达开始遥操作位置的轨迹包装
        //单关节轨迹规划函数，初始化位姿。
        std::vector<double> lArm_end_pos_rpy_st = {
            0.4818, 0.239, -0.1493, 0.0, 0.0, 0.0
        };
        std::vector<double> rArm_end_pos_rpy_st = {
            0.4818, -0.239, -0.1493, 0.0, 0.0, 0.0
        };
        //插补器及其轨迹接收器
        float trajTime = 2.0;       //轨迹执行时间统一为2秒
        float accRateBase = 1.2;
        std::vector<stTraj_runiner> l_trajRuniner;
        std::vector<stTraj_runiner> r_trajRuniner;
        std::vector<TrajResult> l_stDerPosTraj;
        std::vector<TrajResult> r_stDerPosTraj;
        int maxLen = 0;

        ctlDesMsgWrap stPos_ctlDesMsg;
        dataSendWrap stPosReach_dataSend;
        int outpTimes = 0;
        void stPosTrajOutp();
        bool ifReachXrStPos = false;



        //位置坐标系调平参数
        std::vector<double> pos_trans_l = {
            0.4818, 0.239, -0.1493
        };
        std::vector<double> pos_trans_r = {
            0.4818, -0.239, -0.1493
        };

        std::vector<double> gesture_trans_l = {
            0.0, 0.0, 0.0
        };
        std::vector<double> gesture_trans_r = {
            0.0, 0.0, 0.0
        };

        //刚度参数：
        int rigidity_mod = 3;   //(定义为中刚度模式)

        void trans_and_outp_core();

        //过程控制变量：

        //是否为初始化轨迹阶段
        bool ifUseStTrajOutp = true;

        //是否为第一次输出
        bool ifFirstOutp = true;

    public:

        //遥操作回调变量

        //左右臂姿态（欧拉四元数）
        std::vector<double> l_arm_end_pos;
        std::vector<double> r_arm_end_pos;

        //左右臂末端位置
        std::vector<double> l_arm_end_delta;
        std::vector<double> r_arm_end_delta;

        //变量初始化
        void init();

        //变量转化并输出
        dataSendWrap trans_and_outp();

        //输出过程控制参量：初始静止状态；
        bool ifUseStFtr = true;
        int ststabTimes = 20;
        int stabtimecount = 0;
};

#endif
#ifndef __upper_Transmit__H__
#define __upper_Transmit__H__

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

#include <unistd.h> // for read()
#include <sys/select.h> // for select()

#include <rclcpp/rclcpp.hpp>

#include "robot_control/msgType.h"

#include "messageType/upper_msgType.h"

#include <mutex>
#include <functional>                
#include <future>                  
#include <memory> 


/*
        by 霜云喵
==========================
上位机交互代码 -- 负责总控层与上位机的局域网通讯解析
==========================

==========================
*/



//--------------------------------------------------
//输入数据处理代码：

//三次B-样条插值
class BSplineFitter_vr{
    private:
        std::vector<std::vector<double>> dataPoints;

        //计算三次均匀B——样条基函数
        void calculateBasisFunctions(double u, 
                               double& N0, double& N1, 
                               double& N2, double& N3) const;

        // 计算B样条曲线点（单点）
        std::vector<double> evaluateCurvePoint(
            const std::vector<std::vector<double>>& controlPoints,
            double u);
        
        // 生成完整B样条曲线
        std::vector<std::vector<double>> generateBSplineCurve(
            const std::vector<std::vector<double>>& controlPoints,
            int samplesPerSegment = 100);

        //曲线长度参数：
        int m_samplePerSegment = 0;
        int controlPoint_num = 0;
    
    public:
        // 拟合接口：输入原始点，输出拟合曲线
        std::vector<std::vector<double>> fit(
            const std::vector<std::vector<double>>& dataPoints,
            int samplesPerSegment = 100);
        
        // 一维数据拟合（将y值作为数据，x均匀分布）
        std::vector<double> fit1D(
            const std::vector<double>& yValues,
            int samplesPerSegment = 100,
            int windowLen = 20
        );

        // 新增：一维点组的最后一段精细采样
        std::vector<double> getLastSegmentPoints1D(
            const std::vector<double>& yValues,
            int pointNum = 50,
            int windowLen = 20
        );

        //参数输出 -- 拟合点的个数
        int pointNumInCurve();
        
};

//一般的三次样条曲线拟合
class CubicSpline_vr{
    private:
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> a, b, c, d;

        // 计算样条系数
        void calculateSplineCoefficients();

        // 查找x_val所在的区间索引
        size_t findSegment(double x_val);

        //传入数据点，计算样条插值
        void SplineCal();

        //计算样条插值在指定点的值
        double interpolate(double x_val);

        bool ifFirstTemp = true;

        //假设每个区间长度为1
        double x_temp = 0;
        double der_x_temp = 1;
        int temp_num = 0;       //区间迭代计数器

    public:
        int windowLen = 300;

        //接口封装：计算x_i到x_{i+1}的拟合结果
        std::vector<double> cubicSplineCal(double y_lst, double y_des, int windowLen);
};

//卡尔曼滤波器
class kalman_filter_vr{
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

//--------------------------------------------------
//插补核心代码 -- 先进行三次样条曲线插补，再进行卡尔曼滤波
class Inp_DataProcess{
    private:

        //B样条曲线参数
        int B_Fit_sampRate = 10;
        int B_Fit_winLen = 30;

        //B样条曲线
        BSplineFitter_vr B_spFit;

        //三次样条曲线参数
        int Cubic_Fit_winLen = 30;

        //三次样条曲线
        CubicSpline_vr Cubic_spFit;

        /*  选择哪一个样条曲线平滑方法
        -----------------------------
            1 -- 三次样条曲线
            2 -- 三次B-样条曲线
        */
        int splineFitter_choose = 1;

        //卡尔曼滤波器参数
        double Qgain = 30000;
        double Rgain = 120;

        //卡尔曼滤波器
        kalman_filter_vr dKalmanFtr;

        //过程控制量
        bool ifFirstTemp = true;
        double lastInp = 0.0;

    public:
        double Spline_fit(double des_Inp);


};
//--------------------------------------------------
//辅助函数

//四元数乘法
std::vector<double> multiplyQuaternions(const std::vector<double>& q1, const std::vector<double>& q2);

// 欧拉角转四元数（ZYX顺序 - 先绕Z轴，再Y轴，最后X轴）
std::vector<double> eulerToQuaternionZYX(const std::vector<double>& euler);

// 四元数转欧拉角（ZYX顺序）
std::vector<double> quaternionToEulerZYX(const std::vector<double>& q);

//--------------------------------------------------
//数据处理包装1 -- 笛卡尔空间数据处理
class arm_traj_ctl_cartesian{
    private:
        //过程变量（最终的输出六维数组）
        std::vector<double> lArm_end_pos_rpy;
        std::vector<double> rArm_end_pos_rpy;

        //控制变量
        ctlDesMsgWrap ctlStMsg;      //初始位置包装
        ctlDesMsgWrap ctlDesMsg;     //目标位置包装

        //过程控制变量
        bool ifFirstOutp = true;    //判断量 -- 是否为第一次输出

        //------------------------------
        //滤波器

        //滤波平滑器 -- 笛卡尔空间
        Inp_DataProcess l_ftr[6];
        Inp_DataProcess r_ftr[6];

        //滤波器控制变量
        bool ifUseStFtr = true;

        //输出过程控制参量：初始静止状态；
        int ststabTimes = 5;
        int stabtimecount = 0;

        //------------------------------
        //笛卡尔空间控制参量
        //位置坐标系调平参数
        std::vector<double> pos_trans_l = {
            //0.23, 0.255, -0.309
            0.0, 0.0, 0.0
        };
        std::vector<double> pos_trans_r = {
            //0.23, -0.255, -0.309
            0.0, 0.0, 0.0
        };

        std::vector<double> gesture_trans_l = {
            //0.0, -1.5707 , 0.0
            0.0, 0.0, 0.0
        };
        std::vector<double> gesture_trans_r = {
            //0.0, -1.5707, 0.0
            0.0, 0.0, 0.0
        };
        //------------------------------

        void trans_and_outp_core();

    public:
    
        //左右臂姿态（欧拉四元数）
        std::vector<double> l_arm_end_pos;
        std::vector<double> r_arm_end_pos;

        //左右臂末端位置
        std::vector<double> l_arm_end_delta;
        std::vector<double> r_arm_end_delta;

        //变量初始化
        void init();

        /*变量转化与输出
        --------------------
        controlMod -- 控制模式： 0 —— 点位控制模式；1 —— 轨迹控制模式
        */
        ctlDesMsgWrap trans_and_outp(bool ifReachEnd, int controlMod);
};


//数据处理包装2 -- 关节空间数据处理
class arm_traj_cal_JntSpace{
    private:
        //控制变量
        ctlDesMsgWrap ctlStMsg;
        ctlDesMsgWrap ctlDesMsg;

        //过程控制变量
        bool ifFirstOutp = true;    //判断量 -- 是否为第一次输出

        //------------------------------
        //滤波器

        //滤波平滑器 -- 关节空间
        Inp_DataProcess l_ftr[7];
        Inp_DataProcess r_ftr[7];

        //滤波器控制变量
        bool ifUseStFtr = true;

        //输出过程控制参量：初始静止状态
        int ststabTimes = 20;
        int stabtimecount = 0;

        void trans_and_outp_core();

    public:

        //左右臂关节角度
        std::vector<double> l_arm_joint_angular;
        std::vector<double> r_arm_joint_angular;

        //变量初始化
        void init();

        /*变量转化与输出
        --------------------
        controlMod -- 控制模式： 0 —— 点位控制模式；1 —— 轨迹控制模式
        */
        ctlDesMsgWrap trans_and_outp(bool ifReachEnd, int controlMod);

};

//--------------------------------------------------
//数据处理核心代码1 -- 机械臂顶层下发轨迹处理 -- 包含针对两种轨迹的处理
class arm_traj_ctl{
    private:
        //发送变量包装
        dataSendWrap mdataSend;   

        //---------------------------------------
        // 默认控制参量参数定义

        //默认刚度参数
        std::vector<int> rigidity_mod_l = { 2, 2, 2, 2, 2, 2, 2};
        std::vector<int> rigidity_mod_r = { 2, 2, 2, 2, 2, 2, 2};

        //默认PD参数传输
        std::vector<double> Kp_l_default = {20, 20, 20, 20, 20, 20, 20};
        std::vector<double> Kd_l_default = {15, 15, 15, 15, 15, 15, 15};

        std::vector<double> Kp_r_default = {20, 20, 20, 20, 20, 20, 20};
        std::vector<double> Kd_r_default = {15, 15, 15, 15, 15, 15, 15};

        //---------------------------------------
        // 轨迹数据处理包装汇总
        arm_traj_ctl_cartesian cartesian_traj_;
        arm_traj_cal_JntSpace JntSpace_traj_;

        //---------------------------------------

    public:
        void init();

        //关节静止情况修改代码
        void JntStabMod(std::vector<int> l_JntStab, std::vector<int> r_JntStab); 

        /*  数据处理与传输代码
        --------------------
        传输协议
        输入内容：
            1. 重力补偿控制量 -- gravCompMod
                0 -- 无重力补偿；1 -- 有重力补偿，目标值作为输入量；2 -- 有重力补偿，反馈值作为输入量
            2. 关节限位控制量 -- JntStabMod
            3. 运动控制量 -- ctlTransMod
                0 -- 归零控制；1 -- 关节模式；2 -- 点位控制； 3 -- 关节轨迹控制； 4 -- 点位轨迹控制； 5 -- 零力拖动模式
            4. 目标力位 -- ctlDesMsg
            5. 力前馈开关量 -- ifUse_TorFeedForward
                默认值：0
                0 -- 不使用；1 -- 使用
            6. 笛卡尔空间轨迹传输模式 -- DescTrajMod
                默认值：1
                1 -- 姿态描述采用欧拉角(z,y,x)；2 -- 姿态描述采用欧拉四元数(w,x,y,z)

            **备注 -- 底层力控尚未开发，所以力前馈控制量恒定为0
        */
        dataSendWrap trans_and_outp(
            int gravCompMod, 
            JntLimMsg_trans JntStabMod_l, 
            JntLimMsg_trans JntStabMod_r, 
            int ctlTransMod, 
            ctlDesMsg_trans ctlDesMsg, 
            bool ifReachEnd, 
            int ifUseTorFeedForward = 0,
            int DescTrajMod = 1
        );
};

//--------------------------------------------------

#endif
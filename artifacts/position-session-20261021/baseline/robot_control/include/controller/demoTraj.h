#ifndef __demoTraj__H__
#define __dempTraj__H__

#include<iostream>
#include<vector>
#include<string>
#include<math.h>

#include "robot_control/config.h"

#include "robot_control/msgType.h"
#include "messageType/upper_msgType.h"
#include "messageType/chassis_msgType.h"

/*
        by 霜云喵
==========================
测试轨迹代码包
==========================
本代码包包含所有预设的测试轨迹
未来其他控制代码书写规范与方法可以参照本代码包裹

注意增添输出过程中修改输出参数的代码段

==========================
*/

//===============================================================================================

/*插补器*/
class runiner {
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

        //一次插补
        TrajResult dPos_queue_stabTime_single(float mposDes, float mTrajTime);
        //二次插补
        TrajResult dPos_queue_stabTime_double(float mposDes, float velSt, float mtrajTime);
        //点位插补
        TrajResult dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase = 1.2);
        /*备注：最后一个参量的含义为加速时间占到达总位移一半时的总运动时间的比例，这个数值必须大于1*/

        //辅助函数 -- 输出最后一轮的速度
        //上述计算过程中的mposDes指的是相对于初始位置的位置变化量，因此这里引入初始位置将位置量还原
        std::pair<float, float> lastPosVelGet(float posSt = 0.0f);
};

/*  数据传输类
----------------
*/
struct DotOutpWrap{
    //需要持续输出的控制量
    armTrajMsg_Trans armTrajMsg;        //机械臂轨迹控制信息传输包裹
    base_control_msg baseCtlMsg;         //底盘控制信息传输包裹

    //-----其他控制量-----

    //关节静止控制量
    std::vector<int> l_ifJntStab;
    std::vector<int> r_ifJntStab;

};

//核心类 -- 用于储存所有的演示轨迹
//注意针对每个初始化、输出函数刷一遍，看看有没有轨迹编号与变量标号不匹配的情况
//注意这些轨迹的参数暂时还没有精调，需要进一步精调以保障轨迹质量
class allDemoTraj{
    public:
        int m_rigidityCtlMod = 1;   //底层控制模式，注意，纯位置模式和电流模式不要管这个，直接设就行

        //----------轨迹1----------
        //纯点位控制轨迹（可以兼顾用作负载测试轨迹）
        /**/
        void init_v1();
        DotOutpWrap TrajOutp_v1();
        /**/

        //----------轨迹2----------
        //定位精度测试轨迹 -- 先抬起到目标位置，再作往复运动
        void init_v2();
        DotOutpWrap TrajOutp_v2();

        //过程控制参数
        bool ifFirstSend_v2 = false;
        bool ifSecondSend_v2 = false;

        //----------轨迹3----------
        //极限速度测试轨迹 -- 先抬起至起始位置，再开始运动
        void init_v3();
        DotOutpWrap TrajOutp_v3();

        //过程控制参数
        bool ifFirstSend_v3 = false;
        bool ifSecondSend_v3 = false;


        //----------轨迹4----------
        //轨迹控制轨迹
        void init_v4();
        DotOutpWrap TrajOutp_v4();

        //----------轨迹5----------
        //零空间控制轨迹 -- 先用点位控制运动到目标点位，再执行零空间轨迹

        void init_v5();
        DotOutpWrap TrajOutp_v5();
        bool ifReachStartPos_v5 = false;    //控制变量 -- 是否到达初始位置
        bool initCtl_v5 = false;

        //----------轨迹6---------
        //三点点位控制代码
        void init_v6();
        DotOutpWrap TrajOutp_v6();

        //----------轨迹7---------
        //零力拖动demo
        void init_v7();
        DotOutpWrap TrajOutp_v7();

        //----------demo操作界面-----------

    private:
        DotOutpWrap mdataSend;     //输出数据包装

        //----------轨迹1参数----------
        /**/
        ctlDesMsgWrap ctlStMsg_v1;
        ctlDesMsgWrap ctlDesMsg_v1;
        /**/

        //----------轨迹2参数----------

        ctlDesMsgWrap ctlStMsg_v2;

        ctlDesMsgWrap ctlDesMsg_v2_p1;
        std::vector<int> lArm_ifJntStab_v2_1, rArm_ifJntStab_v2_1;

        ctlDesMsgWrap ctlDesMsg_v2_p2;
        std::vector<int> lArm_ifJntStab_v2_2, rArm_ifJntStab_v2_2;

        bool ifSecondPoint_v2;      //是否为第2个点对应的运动过程

        //----------轨迹3参数----------
        ctlDesMsgWrap ctlStMsg_v3;

        ctlDesMsgWrap ctlDesMsg_v3_p1;
        std::vector<int> lArm_ifJntStab_v3_1, rArm_ifJntStab_v3_1;

        ctlDesMsgWrap ctlDesMsg_v3_p2;
        std::vector<int> lArm_ifJntStab_v3_2, rArm_ifJntStab_v3_2;

        bool ifSecondPoint_v3;      //是否为第2个点对应的运动过程

        bool ifRedefineStabJntAng = true;

        double traj_time_v3;

        //----------轨迹4参数----------
        ctlDesMsgWrap ctlStMsg_v4;      //初始位置包装
        ctlDesMsgWrap ctlDesMsg_v4;     //目标位置包装
        std::vector<ctlDesMsgWrap> ctlDesTraj_v4;  //关节限制轨迹

        int trajLen;        //单条运动轨迹长度
        int all_len;        //轨迹总长度
        int temp_times;     //迭代次数

        std::vector<runiner> lArm_DescRuniner_v4;
        std::vector<runiner> rArm_DescRuniner_v4;

        std::vector<runiner> lArm_JntRuniner_v4;
        std::vector<runiner> rArm_JntRuniner_v4;

        //轨迹4新增控制参数 -- 让机械臂高速动三次后强制到目标点进入中等阻抗
        int counter_1 = 0;      //计数器1 -- 用于衡量是否到达目标位置的变量
        int counter_2 = 0;      //计数器2 -- 每当计temp_times大小等于轨迹长度，该数值加1,并请零counter_1
        int all_count = 3;      //计数器3 -- 默认为3,用于控制轨迹总长度
        bool ifReachDesPos = false;     //判断变量 -- 是否到达目标位置

        //----------轨迹5参数----------
        
        ctlDesMsgWrap ctlStMsg_v5;
        ctlDesMsgWrap ctlDesMsg_v5;

        JntLimMsgWrap stJntLimMsg_l;                //轨迹初始端受限关节信息
        JntLimMsgWrap endJntLimMsg_l;               //轨迹终点端受限关节信息
        JntLimMsgWrap stJntLimMsg_r;                //轨迹初始端受限关节信息
        JntLimMsgWrap endJntLimMsg_r;               //轨迹终点端受限关节信息
        std::vector<JntLimMsgWrap> lArmJntLimMsg;
        std::vector<JntLimMsgWrap> rArmJntLimMsg;

        int trajLen_v5;     //单条运动轨迹长度
        int all_len_v5;     //运动轨迹总长度
        int temp_times_v5;  //迭代次数

        //----------轨迹6参数----------
        ctlDesMsgWrap ctlStMsg_v6;      //初始位置包装
        std::vector<ctlDesMsgWrap> ctlDesPoints_v6;  //关节限制轨迹
        std::vector<std::vector<ctlDesMsgWrap>> ctlDesTraj_v6;    //完整的轨迹序列（需要添加）

        std::vector<int> all_len_v6;                    //轨迹总长度（需要添加）
        int all_singleJntTempTimes_v6 = 0;
        int singleJntTempTimes_v6 = 0;      //单个点到达的次数
        int temp_times_v6 = 0;
        int dotID_v6 = 0;
        bool ifFw = true;
        int tol = 2;

        int point_choose_v6 = 0;

        std::vector<runiner> lArm_DescRuniner_v6;
        std::vector<runiner> rArm_DescRuniner_v6;

        std::vector<runiner> lArm_JntRuniner_v6;
        std::vector<runiner> rArm_JntRuniner_v6;

        //----------轨迹7参数----------

        ctlDesMsgWrap ctlStMsg_v7;
        ctlDesMsgWrap ctlDesMsg_v7;
};

//===============================================================================================


#endif
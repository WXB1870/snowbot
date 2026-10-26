#ifndef __msgType__H__
#define __msgType__H__

#include<iostream>
#include<vector>
#include<string>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

/*
        by 霜云喵
================================
机械臂通讯用的数据类型包

本头文件中，定义了数据交换所需的数据类型以及交换过程所需的常数
============修改日志============

==============协议==============
注意笛卡尔空间位置参数向量旋转矩阵应当采用欧拉角(roll,pitch,yaw)形式进行描述

================================
*/

//---------------关键常数定义----------

//单臂自由度
#define SingleArmDof (7)
#define LARMDOF (7)
#define RARMDOF (7)

//一轮迭代循环花费时间
#define TempTime (0.02)

/*  数据类型：数据发送包裹 -- 上层上位机向着中层节点发送的数据
---------------------------------------------------------
*/
/*1. 辅助类型 -- 重力补偿控制参数
-------------------------------
    所有开关量：0 -- 否； 1 -- 是
    GravCompInpMod：控制量——控制重力采用何种输入 1--采用控制器目标位置输入，2--采用反馈输入，3--仅位置量采用反馈值作为输入的重力补偿
*/
typedef struct{
    int ifUseGravComp;          //开关量——是否打开重力补偿
    int mIfUseFricComp;         //开关量——是否打开摩擦补偿
    int mIfUseInpFtr;           //开关量——是否使用输入滤波器
    int mIfUseOutpFtr;          //开关量——是否使用输出滤波器
    int GravCompInpMod;         //控制量——控制重力采用何种输入
}gravComCtlWrap;

/*  辅助类 -- 插补过程中有关静止阶段信息的容器*/
typedef struct{
    bool ifUseStab;         //判断量 -- 是否在初始阶段静止
    int start_stabTimes;    //初始阶段的静止次数
    int des_stabTimes;      //运动到目标阶段的静止次数
}runinStabMsgWrap;

struct TrajResult {
    std::vector<float> positionIncrements;
    std::vector<float> VelocityTraj;
};

/*  辅助类 -- 目标位置、目标前馈力矩的包装
---------------------------------------------
    注意目标位置、目标前馈力矩包含角度模式以及笛卡尔模式两种，当处于一种传输模式下，对于另一种传
    输模式对应的数据应当传送合适的占位值占位。
---------------------------------------------
    注意：对于笛卡尔空间的目标信息需要用六维向量描述，这里采用vector容器装入
    对于关节空间目标信息用vector容器即可
*/
typedef struct{
    std::vector<double> desPos_l_Descartes;     //笛卡尔目标位置（左臂）
    std::vector<double> desTor_l_Descartes;     //笛卡尔目标前馈末端力（左臂）

    std::vector<double> desPos_r_Descartes;     //笛卡尔目标位置（右臂）
    std::vector<double> desTor_r_Descartes;     //笛卡尔目标前馈末端力（右臂）

    std::vector<double> desPos_l_JointSpace;    //关节空间目标位置（左臂）
    std::vector<double> desTor_l_JointSpace;    //关节空间目标前馈力（左臂）

    std::vector<double> desPos_r_JointSpace;    //关节空间目标位置（右臂）
    std::vector<double> desTor_r_JointSpace;    //关节空间目标前馈力（右臂）
}ctlDesMsgWrap;

/*  辅助类 -- 关节限制类
--------------------------
    本类别中给出关节限制信息
*/
typedef struct{
    bool ifUseJntLim;               //判断量——是否开启关节限制
    int limJntNum;                  //限制关节数量
    std::vector<std::string> limJntLabel;      //受限关节的标签
    std::vector<double> limJntAng;   //受限关节的角度
}JntLimMsgWrap;

/*  类型整合 -- 数据传输包裹
-----------------------------
数据含义：
    数据传输模式选择：
        0 -- 归零控制
        1 -- 关节模式（直接传输关节指令）
        2 -- 点位控制（传输点位指令）
        3 -- 关节轨迹控制（连续的关节位置）
        4 -- 点位轨迹控制（连续的点位轨迹）
        5 -- 零力拖动模式
        
    插补模式选择
        -1 -- 不插补 -- 直接输出逆解结果
        1 -- 线性插补 -- 连续两点间进行，速度近似连续
        2 -- 二次插补 -- 连续两点间进行，速度近似连续
        3 -- 点位插补 -- 包含完整的加减速过程，适配点位控制
        **插补在关节空间进行**
        **使用时一定要看好插补模式是否匹配**

    底层控制模式选择
        -1 -- 自定义刚度系数模式
        1 -- 低刚度
        2 -- 中刚度
        3 -- 高刚度
        7 -- 超高刚度

所有开关量：0--否；1--是
*/
typedef struct{
    /*  中层节点控制数据：  */

    //----------数据传输与插补模式控制---------
    int ctl_transMod;       //数据传输模式
    int runinMod;           //插补模式

    //----------求解模式信息----------
    int ifUseRet;           //开关量：求解模式 -- 是否使用返回值
    int ifUseTemp;          //开关量：点位插补模式下，是否循环输出（一般为否）
    bool ifUseRetToStartRunin = false; //下层插补从当前反馈开始
    bool ifStreamingJointPosition = false; //连续关节位置流
    
    int rigidityCtl_mod_l[7];       //控制量：左臂所有关节底层控制模式选择
    int rigidityCtl_mod_r[7];       //控制量：右臂所有关节底层控制模式选择

    //左右臂下发的PD参数
    std::vector<double> KpTrans_l;
    std::vector<double> KdTrans_l;
    std::vector<double> KpTrans_r;
    std::vector<double> KdTrans_r;

    //----------重力补偿控制----------
    gravComCtlWrap GravCompCtlMsg;
    int stJntID_lArm;       //左臂初始关节ID
    int stJntID_rArm;       //右臂初始关节ID
    int urdf_choose;        //urdf路径文件

    //----------插补信息包装----------
    std::vector<std::vector<double>> TrajTime;       //各个关节的运动时间
    std::vector<std::vector<double>> accRateBase;    //各个关节加速阶段时间占比
    std::vector<std::vector<runinStabMsgWrap>> m_stabMsg;   //静止阶段信息

    //-----静止模式控制量：当关节处于静止模式时，其输出是否为恒定位置-----
    std::vector<std::vector<int>> ifJntStab;    // 注意在笛卡尔空间轨迹模式下这个值必须全部刷成0

    bool ifRedefineJntTrajStab;     //判断量 -- 是否重新定义角度控制中静止关节的角度

    //----------关节限位信息----------
    JntLimMsgWrap lArm_JntLim;      //左臂关节限制信息
    JntLimMsgWrap rArm_JntLim;      //右臂关节限制信息

    bool ifUpdateLim;      //判断量 -- 在计算过程中关节受限信息是否更新

    //----------关节初始化信息----------

    //判断是否初始化，或者重新初始化
    bool ifReInit;

    //初始化关节位置
    std::vector<double> lArm_stPos;
    std::vector<double> rArm_stPos;

    //信息打印参数
    bool ifDispJntInfo;
    bool ifUse_visualization;
    bool ifDisp_msgs;

    //优化器参数
    double optiWeight[5];       //优化器权重
    int template_times;         //优化器迭代次数
    double errTol;              //优化器容差

    int ifPrintOptiLevel;       //信息打印参数：是否打印优化日志
    bool ifPrintOptiTime;       //信息打印参数：是否打印计算时间

    //----------目标数据发送----------
    /*  目标控制数据    */
    ctlDesMsgWrap ctlDesMsg;

}dataSendWrap;





/*  类型整合 -- 返回数据传输包裹
--------------------------------
*/
typedef struct{
    /*  中层节点反馈数据    */

    //----------关节电机反馈值----------

    std::vector<double> lArm_pos_fdBack;        //左臂关节位置反馈
    std::vector<double> lArm_spd_fdback;        //左臂关节速度反馈
    std::vector<double> lArm_iq_fdback;         //左臂关节iq反馈值

    std::vector<double> rArm_pos_fdback;
    std::vector<double> rArm_vel_fdback;
    std::vector<double> rArm_iq_fdback;

    //----------关节电机是否运动到目标位置---------
    std::vector<bool> lArm_if_Jnt_reach_desPos;
    std::vector<bool> rArm_if_Jnt_reach_desPos;

    //---------底层计算状态反馈---------
    std::vector<double> lArm_JntTor_gravComp;
    std::vector<double> lArm_JntTor_InteriaComp;
    std::vector<double> lArm_JntTor_FeedForward;

    std::vector<double> rArm_JntTor_gravComp;
    std::vector<double> rArm_JntTor_InteriaComp;
    std::vector<double> rArm_JntTor_FeedForward;

    //----------控制反馈量----------
    bool ifFirstTemp_inBase;        //底层插补是否为首轮
    bool ifFirstTemp_inMiddle;      //中层插补是否为首轮

}dataGetWrap;


//====================================================================================================

//辅助函数：KDL::Frame、KDL::Jntarray与std::vector<double> 类型的数据相互转化的代码
KDL::JntArray vec_2_JntArray(const std::vector<double>& mVecInp);
std::vector<double> JntArray_2_vec(const KDL::JntArray& mArrInp);
KDL::Frame vec_2_Frame(const std::vector<double>& mVecInp);
std::vector<double> Frame_2_vec(const KDL::Frame& mFrameInp);

#endif
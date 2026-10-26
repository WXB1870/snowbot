#ifndef __ArmControllerNew__H__
#define __ArmControllerNew__H__
#include <vector>
#include <stdexcept>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp> // 用于逆动力学（含重力项）
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Dense>

#include <fstream>
#include <string>
#include <random>

#include "robot_config/msgDefine.h"
#include "robot_config/config.h"
#include "controller/IKFKsolver.h"
#include "controller/TopicJointState.h"

/*
        by 霜云喵
================================
整体控制代码 -- 改进重构版
本代码包为正逆解、机械臂力补偿以及基础点位规划的封装，后续在此进一步封装阻抗控制等
============修改日志============

==============协议==============
臂杆顺序：
    0--左臂
    1--右臂
关节标号：
    0~6 -- 左臂7个关节
    7~13 -- 右臂7个关节
关节角度默认采用弧度制，角度制需要转化成弧度制再运用
注意传入的关节角度数量需要时刻保证与关节数量相等

这里电流常数的定义为：电流乘以这个常数，可以得到电机输出力矩

正解中末端姿态量采用旋转矩阵描述，既R=R_z(Yaw)*R_y(Pitch)*R_x(Roll)

==============封装内容==============

一. 一级控制器：
    1. 重力补偿、惯性力补偿、科氏力补偿
    2. 力前馈（预留接口）
    3. 插补 -- 3种插补模式

二. 综合控制器：
    1. 输入：
        控制模式
            对应控制模式，有不同的插补模式
            对应控制模式，有不同的输入代码
        控制参数
        运动参数

    2. 输出：
        5个控制量
        控制时间

================================
*/

//数据传输类型：

extern std::vector<double> dbg_reserve_extern;
extern std::vector<double> dbg_reserve_extern_2;
extern std::vector<double> dbg_reserve_extern_3;

//--------------------------------------------------------------------------------

/*
注意 -- 以下数据不是全都需要传输的，不同控制模式必须传输的数据是不同的
--------------------------------
对于不同的数据传输类型，对于不必须传输的数据，可以直接空着或者传输一个占位的数据
*/

/*  辅助类 -- 插补过程中有关静止阶段信息的容器*/
typedef struct{
    bool ifUseStab;         //判断量 -- 是否在初始阶段静止
    int start_stabTimes;    //初始阶段的静止次数
    int des_stabTimes;      //运动到目标阶段的静止次数
}runinStabMsg;

/*  辅助类 -- 重力补偿控制参数封装
---------------------------------
（速度反馈值常见存在较大噪声，由此差分而来的加速度效果不好）
重力输入模式分为三种：
    1 -- 采用控制器目标位置输入
    2 -- 采用反馈值作为输入（不推荐）
    3 -- 部分采用反馈值做输入（位置量采用反馈值，速度、加速度采用输入值）
*/
typedef struct{
    int ifUseGravComp;      //开关量——是否打开重力补偿
    int mIfUseFricComp;     //开关量——是否打开摩擦补偿
    int mIfUseInpFtr;       //开关量——是否使用输入滤波器
    int mIfUseOutpFtr;      //开关量——是否使用输出滤波器
    int GravCompInpMod;     //控制量——控制重力采用何种输入：1--采用控制器目标位置输入；2--采用反馈值作为输入

    int stJntID_lArm;       //左臂初始关节ID
    int stJntID_rArm;       //右臂初始关节ID
    int urdf_choose;        //预设urdf路径选择

}gravCompCtlMsg;

/*  数据传输控制：
-------------------------
数据含义：
    数据传输模式选择：
        0 -- 归零控制
        1 -- 关节模式（直接传输关节指令）
        2 -- 点位控制（传输点位指令）
        3 -- 关节轨迹控制（连续的关节位置）
        4 -- 点位轨迹控制（连续的点位轨迹）
    插补模式选择
        -1 -- 不插补 -- 直接输出逆解结果
        1 -- 线性插补 -- 连续两点间进行，速度近似连续
        2 -- 二次插补 -- 连续两点间进行，速度近似连续
        3 -- 点位插补 -- 包含完整的加减速过程，适配点位控制
        **插补在关节空间进行**
        **使用时一定要看好插补模式是否匹配**
    底层关节控制模式选择
        0 -- 保护待机模式
        1 -- 低刚度
        2 -- 中刚度
        3 -- 高刚度
        4 -- 纯位置模式
        5 -- 纯电流模式
-------------------------
    **注意：插补模式必须与点位控制模式适配**
    **注意：该类中采用vector嵌套格式传输数据，
      嵌套外层 -- 对应各个机械臂
      嵌套内层 -- 对应机械臂中的各个关节
    **
    **注意：在使用归零控制后，下一轮不管使用什么控制，ifUseRet必须设定为true（1）**
*/
typedef struct{
    int transMod;       //数据传输模式
    int runinMod;       //插补模式  
    //------求解模式信息------
    int ifUseRet;       //求解模式 -- 是否使用返回值 0--否；1--是
    bool ifUseTemp;     //求解模式 -- 是否使用循环输出
    bool ifUseRetToStartRunin = false; //下层插补从当前反馈开始
    bool ifStreamingJointPosition = false; //连续关节位置流
    int base_rigiditiCtl_mod_l[SINGLEARMDOF];   //求解模式 -- 左臂底层关节控制模式
    int base_rigiditiCtl_mod_r[SINGLEARMDOF];   //求解模式 -- 右臂底层关节控制模式
    gravCompCtlMsg gravComp_ctl;    //重力补偿求解模式控制
    //------插补信息包装------
    float upperMonitor_time;                            //上位机轨迹迭代时间（适配于连续规划）
    std::vector<std::vector<float>> TrajTime;           //各个关节的运动时间（适配于点位规划）
    std::vector<std::vector<float>> accRateBase;        //各个关节加速阶段时间占比
    std::vector<std::vector<runinStabMsg>> m_stabMsg;    //辅助类：静止阶段信息

    /*只有机器人处于关节控制模式下时，这个输入量有意义，该输入量含义为是否令关节处于静止输出模式
    -------------------------------------------------------------------------------
    当处于静止输出模式时，关节位置输出恒定为初始位置
    该判断值： 0--否 ； 1--是
    */
    std::vector<std::vector<int>> ifJntStab;
}modCtl;

//--------------------------------------------------------------------------------

//插补信息类内传递
/*  一般插补模式信息：
-------------------------
    适配于插补模式1、2
*/
typedef struct{
    float mPosDes;          //目标位置
    float mPosSt;           //初始位置
    float mVelSt;           //初始速度
    float mTrajTime;        //目标运动时间
    int runinMod;           //插补模式（填写1或者2,对应插补模式1、2）
}runinMsg;
/*  点位控制插补信息：
-----------------------
    适配插补模式3
*/
typedef struct{
    float mPosDes;                  //目标位置
    float mPosSt;                   //初始位置
    float mTrajTime;                //总运动时间
    float accTimeRateBase;          //加速度占比
}runinMsgDotMod;

/*插补运动轨迹参数*/
typedef struct{
    float a;
    float c;
    float C;
    float K;
    float t;
    float posEnd;
}MotionParams;

/*插补轨迹结果参数*/
struct TrajResult {
    std::vector<float> positionIncrements;
    std::vector<float> VelocityTraj;
};

//类定义结构体——逆解控制输出量
struct ctlOutp {
    double posDes;      //目标位置
    double velDes;      //目标速度

    double grav_TorDes;         //目标重力补偿力矩
    double interia_TorDes;      //目标惯性力补偿力矩
    double feedforward_TorDes;  //目标前馈力矩

    bool ifReachEnd;    //是否达到目标位置
};

//======================================================================
/*一级控制算法*/


//插补类：
class runiner {
    /*插补器*/
    /*注意总运动时间必须是control_T的整数倍*/
    private:
        float control_T = TEMPTIME;//控制周期

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

//力补偿类：

//机器人整体力补偿环节：
//摩擦补偿参数
//力补偿系数
extern double mGravCompInte[ALLJNTNUM];
extern double mCoriousCompInte[ALLJNTNUM];
extern double mInertiaCompInte[ALLJNTNUM];

extern float lArm_fs[SINGLEARMDOF];
extern float lArm_fc[SINGLEARMDOF];
extern float lArm_fstic[SINGLEARMDOF];
extern float lArm_vs[SINGLEARMDOF];
extern float lArm_gama[SINGLEARMDOF];
extern float lArm_JntT0[SINGLEARMDOF];
extern int ifUseLeftFricComp[SINGLEARMDOF];

extern float rArm_fs[SINGLEARMDOF];
extern float rArm_fc[SINGLEARMDOF];
extern float rArm_fstic[SINGLEARMDOF];
extern float rArm_vs[SINGLEARMDOF];
extern float rArm_gama[SINGLEARMDOF];
extern int ifUseRightFricComp[SINGLEARMDOF];

//电流-力矩常数，用于将反馈电流转化成力矩量
extern float Kt_lArm[SINGLEARMDOF];     //左臂的电流常数
extern float Kt_rArm[SINGLEARMDOF];     //左臂的电流常数

//低通滤波器参数
//暂时采用一阶低通滤波器处理传感器返回的数据
extern double all_sampRate_posInp_lArm[SINGLEARMDOF];      //位置输入低通滤波采样率
extern double all_cutoffFreq_posInp_lArm[SINGLEARMDOF];    //位置输入低通滤波时间常数
extern double all_sampRate_velInp_lArm[SINGLEARMDOF];      //速度输入低通滤波采样率
extern double all_cutoffFreq_velInp_lArm[SINGLEARMDOF];    //速度输入低通滤波时间常数
extern double all_sampRate_TorInp_lArm[SINGLEARMDOF];      //力输入低通滤波采样率
extern double all_cutoffFreq_TorInp_lArm[SINGLEARMDOF];    //力输入低通滤波时间常数
extern double all_sampRate_TorOutp_lArm[SINGLEARMDOF];      //力输出低通滤波采样率
extern double all_cutoffFreq_TorOutp_lArm[SINGLEARMDOF];    //力输出低通滤波时间常数

extern double all_sampRate_posInp_rArm[SINGLEARMDOF];      //位置输入低通滤波采样率
extern double all_cutoffFreq_posInp_rArm[SINGLEARMDOF];    //位置输入低通滤波时间常数
extern double all_sampRate_velInp_rArm[SINGLEARMDOF];      //速度输入低通滤波采样率
extern double all_cutoffFreq_velInp_rArm[SINGLEARMDOF];    //速度输入低通滤波时间常数
extern double all_sampRate_TorInp_rArm[SINGLEARMDOF];      //力输入低通滤波采样率
extern double all_cutoffFreq_TorInp_rArm[SINGLEARMDOF];    //力输入低通滤波时间常数
extern double all_sampRate_TorOutp_rArm[SINGLEARMDOF];      //力输出低通滤波采样率
extern double all_cutoffFreq_TorOutp_rArm[SINGLEARMDOF];    //力输出低通滤波时间常数

//辅助函数：力与电流的转换
std::vector<double> Tor2Iq(float Kt[], std::vector<double> Tor, int Len);
std::vector<double> Iq2Tor(float Kt[], std::vector<double> Tor, int Len);
//力补偿类
class ForceComp
{
    private:

        //Pinocchio模型与数据
        pinocchio::Model model_;
        pinocchio::Data data_;

        //机器人状态矩阵
        Eigen::VectorXd q_, v_, a_;             //所有关节(包含虚拟关节)位置、速度、加速度
        Eigen::VectorXd v_lst_;                 //所有关节上一轮速度

        float temp_T = TEMPTIME;   //控制时间
        int tempLen_1 = 2 * SINGLEARMDOF;

        //综合重力补偿输出系数
        //由于.urdf文件中质量项不一定准（这是很常见的现象），因此可以把输出结果除一个补偿系数来调平
        //但这只是一个治标不治本的方法，要想彻底改善还得修改.urdf文件
        int JntNum = ALLJNTNUM;
        std::vector<double> grav_compInte;      //重力补偿除系数，注意这个力矩补偿系数顺序对应关节排序的协议
        std::vector<double> corious_compInte;   //科氏力补偿除系数
        std::vector<double> interia_compInte;   //惯性力补偿除系数


        std::vector<Kernel::StribeckFriction> allFricComp;  //stribeck关节摩擦补偿

        //滤波器项：
        std::vector<Kernel::lowpassFilter> accFtr;      //摩擦对应的滤波器项
        std::vector<double> sampRate = {
            10000, 10000, 10000, 10000, 10000, 10000, 10000,
            10000, 10000, 10000, 10000, 10000, 10000, 10000
        };
        std::vector<double> cutoffFreq = {
            10, 10, 10, 10, 10, 10, 10,
            10, 10, 10, 10, 10, 10, 10
        };

    public:

        int inte_inp[14] = {
            1, 1, 1, 1, 1, 1, 1,     //左臂
            1, 1, 1, 1, 1, 1, 1     //右臂
        };
        int inte_outp[14] = {
            1, 1, 1, 1, 1, 1, 1,      //左臂
            1, 1, 1, 1, 1, 1, 1     //右臂
        };

        bool init(KDL::JntArray stPos_lArm, int stJntID_lArm, 
                       KDL::JntArray stPos_rArm, int stJntID_rArm, 
                       int urdf_choose, int ArmNum = 2);

        /*
            注意：必须保证补偿函数调用顺序，先调用重力综合补偿函数，再调用惯性综合补偿函数
        */

        //重力综合补偿（重力，科氏力）
        std::vector<double> gravComp(std::vector<double> lArm_pos, std::vector<double> lArm_vel,
                      std::vector<double> rArm_pos, std::vector<double> rArm_vel
                      );

        //惯性综合补偿(预留空函数作为接口)(惯性力+科氏力)
        std::vector<double> InteriaMulComp(
            std::vector<double> lArm_pos, std::vector<double> lArm_vel,
            std::vector<double> rArm_pos, std::vector<double> rArm_vel
        );

        //摩擦补偿
        void FricCompInit();
        std::vector<double> FricComp(
            std::vector<double> lArm_pos, std::vector<double> lArm_vel, std::vector<double> lArm_Tor,
            std::vector<double> rArm_pos, std::vector<double> rArm_vel, std::vector<double> rArm_Tor
        );

        //打印信息函数
        void printModelInfo();
};

//----------------------------------------------------------------------------
//滤波器类
class WeightedMovingAverage {
    private:
        std::deque<double> data_buffer_;  // 数据缓冲区
        std::vector<double> weights_;     // 权重系数
        size_t window_size_;              // 窗口大小
        double sum_weights_;              // 权重总和（用于归一化）

        //计算加权平均值
        double calculateWeightedAverage();

    public:

        //构造函数
        WeightedMovingAverage(size_t window_size, const std::vector<double>& weights);

        //添加数据并返回滤波值
        double filter(double new_value);

        //-----------------------------------
        //辅助函数
        // 获取当前滤波器的状态（是否已填满窗口）
        bool isReady() const {
            return data_buffer_.size() == window_size_;
        }

        // 重置滤波器
        void reset() {
            data_buffer_.clear();
        }

        // 获取当前缓冲区大小
        size_t getCurrentSize() const {
            return data_buffer_.size();
        }

        // 获取窗口大小
        size_t getWindowSize() const {
            return window_size_;
        }
};

//======================================================================
/*单关节点位控制代码
-------------------------------------------
  单关节点位控制代码仅包含定时轨迹控制代码
*/

//静止关节代码
class jnt_stabOutp{
    //机器人关节静息输出
    private:
        float JntDeg;   

    public:
        void init(float mJntDeg);
        ctlOutp StabOutp();
};

//单关节代码控制代码封装
class moveJntCtl_stabTime{
    private:

        //插补器
        runiner mlocater;
        //控制参数：
        float temp_T = TEMPTIME;        //控制时间
        int stabRanges;                 //在初始位置静止时长
        int stabTimes;                  //在目标位置停留的时间
        bool ifUseStab = true;          //是否静息
        int TempTimes = 0;              //迭代次数

        //----------------------------------------
        //过程参数：
        float startPos;     //初始位置
        float endPos;       //末端位置

        //轨迹参数：
        //用于搭配定时轨迹控制
        std::vector<float> posTraj_stabTime;
        std::vector<float> velTraj_stabTime;
        std::vector<float> TorTraj_stabTime;        //力矩输出占位，设定为0
        std::vector<float> posTraj_stabTime_half;
        std::vector<float> velTraj_stabTime_half;
        std::vector<float> TorTraj_stabTime_half;   //力矩输出占位，设定为0

        //----------------------------------------
        //搭配连续轨迹规划的变量
        float posTraj_last;

    public:
        int ID;

        //初始化函数 -- 对应两种控制模式，有两种初始化函数：
        //模式1 -- 适配非连续运动，点到点模式
        /*  求解函数
        --------------------
        输入量含义：
            mCtlMsg -- 求解信息包装
            mstabMsg -- 静止信息包装
        
        */
        void init_stabTime(runinMsgDotMod mCtlMsg, runinStabMsg mstabMsg);
        //输出函数——每调用一次就输出一个相应的输出值
        ctlOutp desPos_get_stabTime();          //做一个从位置A到位置B的往复运动
        ctlOutp desPos_get_stabTimeStop();      //做一个从位置A到位置B的运动并停止在B点
        int TrajLen;    //停留阶段轨迹长度

        //模式2 -- 适配连续运动
        void init_stabTime_continue(runinMsg mCtlMsg);
        ctlOutp desPos_get_stab_Continue();
        int TrajLen_Continue;

        //模式3 -- 静息输出
        jnt_stabOutp mStabOutp;

        /*  辅助函数 -- 输出最后一轮运动的速度
        ------------------------------------
        输出量依据顺序依次为位置、速度
        */
        std::pair<float, float> lastTempPosVel();

        //刷新函数：
        void clear(){
            posTraj_stabTime.clear();
            velTraj_stabTime.clear();
            TorTraj_stabTime.clear();
            posTraj_stabTime_half.clear();
            velTraj_stabTime_half.clear();
            TorTraj_stabTime_half.clear();

            this->TempTimes = 0;
            this->ifUseStab = true;
        }
};

//======================================================================
/*综合控制算法*/
float double2Float(double inp);                 //快捷转换函数 -- double变为float      

/*  针对机器人整体直接传输关节参数的求解代码封装
---------------------------------------------
    注意：力轨迹与点位控制不适配，其对应的力输出默认为0
    仅当关节处于轨迹模式时，才会涉及到力前馈的控制
*/
class JntTrackSol{
    private:
        bool topic_stream_ = false;
        std::vector<TopicJointState> topic_left_, topic_right_;
        //机器人关节求解器：
        std::vector<moveJntCtl_stabTime> lArmJntTrajSol;
        std::vector<moveJntCtl_stabTime> rArmJntTrajSol;
        std::vector<jnt_stabOutp> lArmStab;
        std::vector<jnt_stabOutp> rArmStab;

        //机器人力矩求解器 -- 力矩轨迹插补
        std::vector<moveJntCtl_stabTime> lArmJntTorTrajSol;
        std::vector<moveJntCtl_stabTime> rArmJntTorTrajSol;

        //过程量：
        KDL::JntArray posLst_lArm;      //上一轮关节角度 -- 左臂
        KDL::JntArray posLst_rArm;      //上一轮关节角度 -- 右臂

        KDL::JntArray velLst_lArm;      //上一轮角速度 -- 左臂
        KDL::JntArray velLst_rArm;      //上一轮角速度 -- 右臂

        std::vector<double> torLst_lArm;    //上一轮力矩输出 -- 左臂
        std::vector<double> torLst_rArm;    //上一轮力矩输出 -- 右臂

        int outp_mod;       //计算控制量--用于输出模式的控制
        bool ifUseTemp = false; //控制量：是否使用循环输出
        bool ifFirstTemp = true;    //控制量：是否为第一轮输出

        std::vector<std::vector<int>> ifJntStab;    //关节输出模式 -- 关节是否静止

    public:

        //初始化：
        void init(KDL::JntArray posSt_lArm, KDL::JntArray posSt_rArm);

        void JntTrajCal(
            //目标量
            KDL::JntArray posDes_lArm, KDL::JntArray posDes_rArm,
            std::vector<double> JntTorDes_l, std::vector<double> JntTorDes_r,

            //反馈量
            KDL::JntArray posRet_lArm, KDL::JntArray posRet_rArm,
            KDL::JntArray velRet_lArm, KDL::JntArray velRet_rArm,
            std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r,

            modCtl mod_ctl_msg,
            bool ifRedefineStab = false         //是否刷新关节静止值
        );
        //这里放一个轨迹长度参数，放在类外便于交互
        std::vector<float> lArmTrajLen;
        std::vector<float> rArmTrajLen;

        std::vector<std::vector<ctlOutp>> solveResOutp(std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r);

        
};

/*逆解轨迹与正解算法封装*/

class FrameTrackSol{
    private:
        //逆解正解求解器：
        IkFk_solverPino mSol_Pino;

        //插补器
        std::vector<moveJntCtl_stabTime> lArm_JntTrajSol;
        std::vector<moveJntCtl_stabTime> rArm_JntTrajSol;

        //过程参量：
        KDL::JntArray lArmLstPos;
        KDL::JntArray rArmLstPos;
        KDL::JntArray lArmLstVel;
        KDL::JntArray rArmLstVel;

        //适配恒定输出的过程参量：
        KDL::JntArray lArmDesPos;
        KDL::JntArray rArmDesPos;

        //修正参量
        std::vector<double> lArm_posLst_velCor;
        std::vector<double> rArm_posLst_velCor;

        //过程控制参量   
        float tempT = TEMPTIME;
        bool if_not_firstTemp = false;  //过程控制量--用于限制第一轮研究中不能重新优化
        bool ifUseTemp;     //过程控制量--用于点到点模式，用于表明是不是循环输出模式
        int outp_mod;       //计算控制量--用于输出模式的控制

        bool ifUseSolve;     //判断量：是否求解过
        bool ifUseFtr = true;   //是否使用滤波器
        std::vector<double> posFtr_weight = {
            1.0, 2.0, 2.0, 2.0
        };
        std::vector<double> velFtr_weight = {
            1.2, 1.5, 2.0, 2.2, 2.0
        };
        WeightedMovingAverage posOutpWtr = WeightedMovingAverage(
            posFtr_weight.size(), posFtr_weight
        );
        WeightedMovingAverage velOutpWtr = WeightedMovingAverage(
            velFtr_weight.size(), velFtr_weight
        );
        std::vector<WeightedMovingAverage> allJntPosFilter_l;
        std::vector<WeightedMovingAverage> allJntVelFilter_l;
        std::vector<WeightedMovingAverage> allJntPosFilter_r;
        std::vector<WeightedMovingAverage> allJntVelFilter_r;

        //关节静止模式：
        bool ifFirstTemp = true;
        std::vector<std::vector<int>> ifJntStab;    //关节输出模式 -- 关节是否静止
        //关节静止控制器
        std::vector<jnt_stabOutp> lArmStab;         
        std::vector<jnt_stabOutp> rArmStab;

        //---------------
        //权重控制代码
        //点到点权重控制
        std::vector<double> p2p_optiWeight = {
            50.0, 1.0, 0.0001, 0.0, 0.0
        };
        //上一轮控制模式
        int controlMod_lastTemp = 0;

    public:

        /*初始化求解器*/
        void init_solver(
            JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,       //关节限制配置
            std::vector<double> JntAngConf_l, std::vector<double> JntAngConf_r,     //关节初始角度配置
            optiPrintMsgCtl mPrintMsgPkg,           //程序打印控制参数
            optimizerCtl m_opti_ctl_msg        //优化器参数
        );

        /*  求解代码
        ---------------
        这里添加力反馈接口，用于集成力控
        */
        void FrameTrack(
            bool ifUpdateLim,               //判断量 -- 是否更新限制信息
            JntLimMsg allArm_LimJnt,        //求解关节限制信息，将所有的关节信息都放在这里面

            KDL::Frame lArm_desPos, KDL::Frame rArm_desPos,         //求解目标位置
            std::vector<double> lArm_desTor, std::vector<double> rArm_desTor,   //目标前馈力矩

            KDL::JntArray lArmPosRet, KDL::JntArray rArmPosRet,     //位置返回值
            KDL::JntArray lArmVelRet, KDL::JntArray rArmVelRet,     //速度返回值
            std::vector<double> lArmTorRet, std::vector<double> rArmTorRet,     //关节力矩反馈值

            modCtl mod_ctl_msg          //数据传输控制过程
        );
        //这里放一个轨迹长度参数，放在类外便于交互
        std::vector<float> lArm_TrajLen;
        std::vector<float> rArm_TrajLen;

        /*  逆解轨迹输出
        -----------------
        同时输出双臂的轨迹，先左臂，后右臂
        */
        std::vector<std::vector<ctlOutp>> IKsolveResOutp(std::vector<double> JntTorRet_l, std::vector<double> JntTorRet_r);

        //正解结果输出接口函数
        std::vector<KDL::Frame> FkSolver(KDL::JntArray lArm_JntArray, KDL::JntArray rArm_JntArray){
            return mSol_Pino.Fk_pinoSolver(lArm_JntArray, rArm_JntArray);
        }

        double reserve[14] = {
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0
        };

        double dbg_a = 0;
};




#endif

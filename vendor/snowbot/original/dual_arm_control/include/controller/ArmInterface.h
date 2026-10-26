#ifndef __ArmInterfaceNew__H__
#define __ArmInterfaceNew__H__

#include "controller/ArmController.h"
#include "robot_config/config.h"

/*
        by 霜云喵
================================
操作代码封装与操作界面封装
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

这里电流常数的定义为：电流乘以这个常数，可以得到电机输出力矩

末端姿态量采用旋转矩阵描述，既R=R_z(Yaw)*R_y(Pitch)*R_x(Roll)

控制时间单位为秒(s)

注意要在合适的时候调用修正函数：KDL::Frame FrameCor(KDL::Frame FrameInp)；
时刻注意输入逆解求解器的量必须圆整到(-pi~pi)之间，否则求解会出错

================================
*/

//默认目标PD参量
extern std::vector<double> Kp_l_Inte;
extern std::vector<double> Kd_l_Inte;
extern std::vector<double> Kp_r_Inte;
extern std::vector<double> Kd_r_Inte;

/*基础操作封装*/
typedef struct{
    //重力补偿
    std::vector<float> lArm_Comp_grav;
    std::vector<float> rArm_comp_grav;

    //惯性力综合补偿
    std::vector<float> lArm_Comp_InteriaComb;
    std::vector<float> rArm_Comp_InteriaComb;

    //力前馈综合补偿
    std::vector<float> lArm_feedForward;
    std::vector<float> rArm_feedForward;

}TorCompWrap;

//重力补偿封装
class DualArmGravWrap{
    private:

    //--------------------------------------------------
    //重力补偿封装

        //滤波器：
        Kernel::lowpassFilter posInp_lowpassFilter_lArm[SINGLEARMDOF];          //位置输入低通滤波
        Kernel::lowpassFilter velInp_lowpassFilter_lArm[SINGLEARMDOF];          //速度输入低通滤波
        Kernel::lowpassFilter TorInp_lowpassFilter_lArm[SINGLEARMDOF];          //力输入低通滤波

        Kernel::lowpassFilter grav_TorOutp_lowpassFilter_lArm[SINGLEARMDOF];    //重力输出低通滤波
        Kernel::lowpassFilter Inte_TorOutp_lowpassFilter_lArm[SINGLEARMDOF];    //惯性综合补偿输出低通滤波
        Kernel::lowpassFilter feedforward_TorComp_lowpassFilter_lArm[SINGLEARMDOF]; //前馈（摩擦）低通滤波
        
        Kernel::lowpassFilter posInp_lowpassFilter_rArm[SINGLEARMDOF];          //位置输入低通滤波
        Kernel::lowpassFilter velInp_lowpassFilter_rArm[SINGLEARMDOF];          //速度输入低通滤波
        Kernel::lowpassFilter TorInp_lowpassFilter_rArm[SINGLEARMDOF];          //力输入低通滤波

        Kernel::lowpassFilter grav_TorOutp_lowpassFilter_rArm[SINGLEARMDOF];    //重力输出低通滤波
        Kernel::lowpassFilter Inte_TorOutp_lowpassFilter_rArm[SINGLEARMDOF];    //惯性综合补偿输出低通滤波
        Kernel::lowpassFilter feedforward_TorComp_lowpassFilter_rArm[SINGLEARMDOF]; //前馈（摩擦）低通滤波

        //控制参量，默认 0--否 ； 1--是，若有特殊情况另行说明
        int ifUseInpFtr;        //开关量——是否使用输入滤波器
        int ifUseOutpFtr;       //开关量——是否使用输出滤波器
        int ifUseFricComp;      //开关量——是否使用摩擦补偿

        //力补偿器
        ForceComp mForceComp;
        

    public:
    //--------------------------------------------------
    //重力补偿求解封装

        /*  初始化函数
        ----------------
        输入量：
            mIfUseInpFtr, mIfUseOutpFtr -- 是否使用输入输出滤波器(int)
            lArm_stPos, rArm_stPos -- 机器人初始位置(KDL::JntArray)
            lArm_stJnt, rArm_stJnt, urdf_choose -- 机器人左右臂初始关节ID，预设urdf选择标号
            mIfUseFricComp -- 是否使用求解器内集成的stribeck摩擦补偿
        */
        void init_GravComp(
            //补偿控制参量：
            int mIfUseInpFtr, int mIfUseOutpFtr,

            //重力补偿参量
            KDL::JntArray lArm_stPos, KDL::JntArray rArm_stPos, 
            //是否使用集成stribeck摩擦补偿参量
            int mIfUseFricComp = 0,

            int lArm_stJnt = 0, int rArm_stJnt = 7, int urdf_choose = 2
        );

        //重力综合补偿函数——注意这个函数中仅包含对机器人本体的重力、科氏力、惯性力补偿，摩擦补偿仅包含重力模型自带的摩擦补偿
        //依据函数分工，具体的摩擦补偿以及其他的前馈不在这里定义，这里对力前馈输出仅包含重力补偿算法自带stribeck补偿。
        TorCompWrap GravCompOutp(
            //输入量
            std::vector<double> lArm_pos, std::vector<double> lArm_vel, std::vector<double> lArm_Tor,
            std::vector<double> rArm_pos, std::vector<double> rArm_vel, std::vector<double> rArm_Tor
        );

        //纯粹的重力补偿函数——输出中仅包含重力补偿
        TorCompWrap onlyGravComp(
            //输入量：
            std::vector<double> lArm_pos, std::vector<double> lArm_vel,
            std::vector<double> rArm_pos, std::vector<double> rArm_vel
        );

        //保留值，debug用
        double reserve[10];
};

//Kp，Kd输出封装
typedef struct{
    std::vector<double> lArmKp;
    std::vector<double> lArmKd;
    std::vector<double> rArmKp;
    std::vector<double> rArmKd;
}ArmPD;
class PDInte{
    private:
        //模式1：预设的PD参数

        std::vector<float> Kp_lArm_low;
        std::vector<float> Kd_lArm_low;
        std::vector<float> Kp_rArm_low;
        std::vector<float> Kd_rArm_low;

        std::vector<float> Kp_lArm_mid;
        std::vector<float> Kd_lArm_mid;
        std::vector<float> Kp_rArm_mid;
        std::vector<float> Kd_rArm_mid;

        std::vector<float> Kp_lArm_high;
        std::vector<float> Kd_lArm_high;
        std::vector<float> Kp_rArm_high;
        std::vector<float> Kd_rArm_high;

        std::vector<float> Kp_lArm_superhigh;
        std::vector<float> Kd_lArm_superhigh;
        std::vector<float> Kp_rArm_superhigh;
        std::vector<float> Kd_rArm_superhigh;

        std::vector<float> Kp_lArm_tag;
        std::vector<float> Kd_lArm_tag;
        std::vector<float> Kp_rArm_tag;
        std::vector<float> Kd_rArm_tag;

    public:
        void PDInteInit();

        //输出函数 -- 输出预设的PD参数
        //输入量 -- modChoose:输出模式选择，不同标号对应不同组的PD参数;     rigidChoose:刚度等级选择，不同标号对应不同刚度等级
        ArmPD PDInteOutp(int modChoose, int rigidChoose);

};

//================================================================================
/*辅助函数*/

/*  类型转换函数：
------------------
注意：Frame类型转化对应的vector容器应当是六维度向量，其数据含义依次为：
[x, y, z, row, pitch, yaw] -- 三个平移量+三个欧拉角
*/
KDL::JntArray vec_to_JntArray(const std::vector<double>& mVecInp);
std::vector<double> JntArray_to_vec(const KDL::JntArray& mArrInp);
KDL::Frame vec_to_Frame(const std::vector<double>& mVecInp);
std::vector<double> Frame_to_vec(const KDL::Frame& mFrameInp);

//================================================================================
/*整体封装*/

//函数输出包装
typedef struct{
    std::vector<double> posDes;
    std::vector<double> velDes;

    std::vector<double> gravComp_TorDes;        //重力补偿目标力矩
    std::vector<double> InteriaComp_TorDes;     //惯性补偿目标力矩
    std::vector<double> FeedForward_TorDes;     //前馈补偿目标力矩

    std::vector<double> KpDes;
    std::vector<double> KdDes;
    std::vector<bool> ifComToEnd;   //关节轨迹是否输出完成的判断量
}outpWrap;

/*修正函数 -- 将输入的Frame类型函数修正，防止角度超程。
求解逆解时，各个角度值应当在(-pi~pi)之间，不能圆整后超出此范围，否则会报错
*/
KDL::Frame FrameCor(KDL::Frame FrameInp); 

//操作界面封装
class ArmCtlOptInterface{
    
    private:
        //重力补偿综合类：
        DualArmGravWrap GravComp;

        //关节空间综合控制代码 -- 直接传输关节值大小
        JntTrackSol JntSpace_track;

        //笛卡尔空间综合控制代码 -- 传输笛卡尔空间轨迹
        FrameTrackSol FrameSpace_track;

        //刚度参数：
        int rigidityMod_l[SINGLEARMDOF];
        int rigidityMod_r[SINGLEARMDOF];
        //PD参数输出
        PDInte mArmPDInte;

        //过程参数
        outpWrap lArmLst_outpWrap;      //左臂上一轮输出
        outpWrap rArmLst_outpWrap;      //右臂上一轮输出
        KDL::JntArray stPos_l;
        KDL::JntArray stPos_r;

        //过程控制参数
        int outpMod;    //输出模式
        int GravCompInpChoose;      //重力补偿输入选择：1--采用计算输出值作为重力补偿输入；2--采用反馈值作为补偿输入
    
        
        bool ifOutpUseGravComp = false;     //开关量 -- 输出过程中是否使用重力补偿

        //机器人关节参数--电流常数：
        std::vector<double> currentConstant_l = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        };
        std::vector<double> currentConstant_r = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        };

        //关节位置修正系数 -- 描述站立情况下torso相对于universe的位姿
        std::vector<double> torso_pos_l = {
            -0.0039635, 0.0, 0.844
        };
         std::vector<double> torso_pos_r = {
            -0.0039635, 0.0, 0.844
        };

    public:
        //初始化函数
        void init(
            KDL::JntArray stPos_lArm, KDL::JntArray stPos_rArm,
            JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,
            modCtl mod_ctl_msg,
            optiPrintMsgCtl mPrintMsgPkg,   //程序打印控制参数
            optimizerCtl m_opti_ctl_msg     //优化器参数
        );

        /*  计算函数 -- 这里整合两种接口，分别适配于输入量为笛卡尔空间角度与关节空间角度的情况
        ---------------------------------------------------------------------------
        注意这里提供了两种目标位置接口：
            KDL::Frame类型的接口 -- 适配于求解笛卡尔空间中关节轨迹
            KDL：：JntArray类型的接口 -- 适配于求解关节空间轨迹
        注意：对于不同求解模式，只需要往对应的接口里面传入数据就行，另外的不适配的接口随便传点什么占位即可
        另外，对于重力补偿，应当在输出过程中实时进行，而非在整个轨迹计算过程中进行
        */
        void Traj_cal(
            bool ifUpdateLim,
            JntLimMsg allArmLimJnt,

            //笛卡尔空间的目标值
            KDL::Frame lArm_desPos_Frame, KDL::Frame rArm_desPos_Frame,
            std::vector<double> lArm_desTor_Frame, std::vector<double> rArm_desTor_Frame,

            //关节空间的目标值
            KDL::JntArray lArm_desPos_Jnt, KDL::JntArray rArm_desPos_Jnt,
            std::vector<double> lArm_desTor_Jnt, std::vector<double> rArm_desTor_Jnt,

            //反馈值
            KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
            KDL::JntArray lArm_velRet, KDL::JntArray rArm_velRet,
            std::vector<double> lArm_iq_ret, std::vector<double> rArm_iq_ret,

            modCtl mod_ctl_msg,
            bool ifRedefineJntTrajStab = false
        );

        /*  输出函数 -- 依据计算模式调用相应的API函数并输出*/
        std::vector<outpWrap> Traj_temp_outp(
            KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
            KDL::JntArray lArm_velRet, KDL::JntArray rArm_velRet,
            std::vector<double> lArm_iq_Ret, std::vector<double> rArm_iq_Ret
        );

        /*正解接口函数*/
        std::vector<KDL::Frame> FKsolver(KDL::JntArray lArm_JntArray, KDL::JntArray rArm_JntArray){
            return FrameSpace_track.FkSolver(lArm_JntArray, rArm_JntArray);
        }

        double reserve[14] = {
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0
        };
};


#endif

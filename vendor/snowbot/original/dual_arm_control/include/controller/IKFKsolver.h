#ifndef __IKFKsolverNew__H__
#define __IKFKsolverNew__H__

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>


#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp> // 用于逆动力学（含重力项）
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>

#include "robot_config/msgDefine.h"

#include "controller/JntController.h"
#include "controller/ik_solver_wrapper.h"
#include "robot_config/config.h"

/*
        by 霜云喵
================================
正逆解求解核心代码
============修改日志============

==============协议==============
臂杆顺序：
    0--左臂
    1--右臂
关节标号：
    0 -- universe
    1~7 -- 左臂
    8~14 -- 右臂

正解中末端姿态量采用旋转矩阵描述，既R=R_z(Yaw)*R_y(Pitch)*R_x(Roll)

算法输出角度为相对于初始值的增量值
滤波器输入输出也是这个增量值

================================
*/

//===============================================================================================
//求解器

typedef struct{
    int ID;
    float ang;
}fixedJnt;      //固定的关节

//--------------------求解器关键变量--------------------

//力位混合控制模式应用的数据转化代码
extern float degInpRange[2];    //单位：弧度度
#define URDFCHOOSE (2)
#define TRACsolveTimeOut (0.01)      //Trac-ik求解器超时时间(s)
#define TRACsolveErrTol (1e-4)      //Trac-ik求解器容许误差

//加载机械臂urdf文件路径

extern std::string path_options[3];
extern std::string base_link[3];
extern std::string tip_link[3];

//--------------------求解器变量类封装--------------------

//定义路径选择类——选择urdf路径
typedef struct{
    int urdf_choose;
    int base_choose;
    int tip_choose;
}PathChoose;

//定义初始化关节限制类：
typedef struct{
    bool ifUseJntLim;               //判断量——是否开启关节限制
    int limJntNum;                  //限制关节数量
    std::vector<std::string> limJntLabel;      //受限关节的标签
    std::vector<double> limJntAng;   //受限关节的角度
}JntLimMsg;

//信息打印参数
typedef struct{
    bool dispJntInfo;               //开关量——是否打印
    bool use_visualization;
    bool disp_messages;
}optiPrintMsgCtl;

//优化器控制类：
typedef struct{
    double OptiWeight[5];           //控制器优化权重——四个权重分别对应位置误差权重、旋转误差权重、正则化成本权重(避免角度过大)、以及平滑项权重
    int template_times;             //控制器优化迭代次数
    double errTol;                  //误差容差
    int ifPrintLevel;               //是否打印优化日志
    bool ifPrintOptiTime;           //是否打印计算时间
}optimizerCtl;

//--------------------信息打印函数封装--------------------

void printJointInfo(const pinocchio::Model& model);     //打印pinocchio模型信息的函数

//字典函数——依据关节ID获取关节名称与索引


//--------------------求解器接口封装类--------------------

class IkFk_solverPino{
    private:
        IKSolverWrapper pyDualArmSolverWrapper;   //基于python的求解器封装，

        //过程控制变量
        std::vector<double> lastTempSolve_Jnt_q;    //上一轮求解中关节求解结果

        //辅助控制变量：
        bool ifDispJntMsg;
    public:
        //----------界面控制变量----------

        //--------------------封装辅助函数--------------------

        //--------------------接口函数--------------------

        //初始化函数
        void Init(PathChoose mChoose_lArm, PathChoose mChoose_rArm,                         //路径选择配置
                  JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,                             //关节信息限制配置
                  std::vector<double> JntAngConf_l, std::vector<double> JntAngConf_r,       //初始角度配置
                  optiPrintMsgCtl mPrintMsgPkg,     //程序打印控制参数
                  optimizerCtl mOptiCtl         //优化器控制参数
            );

        //权重更新
        /*  输入量：
            update_mode -- 更新模式：1——采用初始化的默认权重；2——采用自定义权重
            optiWeight_selfDefine -- 自定义权重
        */
        void PinoSolver_optiWeightUpdate(int update_mode, std::vector<double> optiWeight_selfDefine, bool ifUseFtr);

        //正解求解器
        std::vector<KDL::Frame> Fk_pinoSolver(KDL::JntArray lArm_JntArray, KDL::JntArray rArm_JntArray);

        //逆解求解器
        std::vector<KDL::JntArray> IK_pinoSolver(
            KDL::Frame lArm_DesPos, KDL::Frame rArm_DesPos, 
            KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
            int solveMod        //求解模式 -- 是否使用返回值：0--否；1--是
        );

        //求解器更新函数
        /*注意：要将所有关节限制信息都装在这一个函数里面*/
        void solverUpdate(JntLimMsg mLimMsg);
};
//===============================================================================================

//其他辅助函数：

//打印KDL::Frame类型数据的函数
void printKDLFrameAsMatrix(const KDL::Frame& frame);

#endif

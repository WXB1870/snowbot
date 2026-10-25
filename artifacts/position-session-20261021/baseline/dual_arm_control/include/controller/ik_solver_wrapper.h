#ifndef __ik_solver_wrapper__H__
#define __ik_solver_wrapper__H__

#include <Python.h>
#include <vector>
#include <string>
#include <numpy/arrayobject.h>

#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

#include "robot_config/msgDefine.h"

/*
        by 霜云喵
================================
python求解器包装
============修改日志============

==============协议==============
1. 自由度协调协议：机器人必须保证模型中左右自由度数目与全局变量规定的左右自由度数目相同，二者要做好协调

================================
*/

class IKSolverWrapper{
    //求解器包装类：
    public:
        //类内变量

        //可交互成员函数：
        IKSolverWrapper();
        ~IKSolverWrapper();

        // 初始化函数
        bool initialize(
            //模型限制参数
            const std::string& urdf_path,
            const std::vector<double>& initial_q,
            const std::vector<std::string>& arm_end_labels,
            bool use_joint_limits,
            const std::vector<std::string>& limited_joints,
            const std::vector<double>& limited_joint_angles,

            //模型信息打印参数
            bool display_joint_info,
            bool use_visualization,
            bool display_messages,

            //控制器控制参数
            const std::vector<double>& optiWeight,
            int template_times,
            float errTol,
            int ifPrintLevel,
            bool ifPrintOptiTime
        );

        // 修改权重接口
        void optiWeight_def(
            int optiWeight_mod,
            const std::vector<double>& optiWeight_selfDefine,
            bool ifUseFtr
        );
    
        // 逆解求解函数
        std::pair<KDL::JntArray, KDL::JntArray> solveIK(
            KDL::Frame left_wrist_pose,
            KDL::Frame  right_wrist_pose,
            const std::vector<double>& current_q = {}
        );
    
        // 正解求解函数
        std::pair<KDL::Frame, KDL::Frame> solveFK(
            KDL::JntArray lArm_pos, KDL::JntArray rArm_pos
        );
    
        // 修改限位接口
        bool updateJointLimits(
            bool use_limits,
            const std::vector<std::string>& limited_joints,
            const std::vector<double>& limited_joint_angles,
            bool display_messages
        );

    private:
        
        PyObject* pModule;
        PyObject* pIKSolverInstance;
        bool initialized;

        // 辅助函数：将std::vector转换为Python列表
        PyObject* vectorToPythonList(const std::vector<double>& vec);
        PyObject* stringVectorToPythonList(const std::vector<std::string>& vec);

        // 辅助函数：Python列表转换成vector类型的函数
        std::vector<double> pythonListToVector(PyObject* pyList);
        std::vector<std::string> pythonListToStringVector(PyObject* pyList);

        // 辅助函数：将4x4位姿矩阵转换为Python可接受的格式
        PyObject* poseToPythonObject(KDL::Frame pose);
        PyObject* JntArrayToPyObject(KDL::JntArray lArm_pos, KDL::JntArray rArm_pos);

        // 辅助函数：将Python列表转化为JntArray
        /*
            注意：
            pyListToDualArmJntArray -- 输入为一维python列表
            pyListToFrame -- 输入为二维python列表
        */
        std::pair<KDL::JntArray, KDL::JntArray> pyListToDualArmJntArray(PyObject* pyList);      //python列表转化为KDL::JntArray对
        KDL::Frame pyListToFrame(PyObject* pyList);     //python列表转化为KDL::Frame

        // 辅助函数：检查Python调用是否成功
        bool checkPythonCall(PyObject* pResult, const std::string& funcName);

        // 初始化numpy数组支持
        int initNumpy();

};

std::string get_packages_path(const std::string packages, const std::string folder);

#endif

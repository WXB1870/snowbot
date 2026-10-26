#include "controller/ik_solver_wrapper.h"
#include <iostream>
#include <sstream>
#include <filesystem>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::string get_packages_path(const std::string packages, const std::string folder) {
    try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory(packages);
        std::filesystem::path p = pkg_share;
        p /= folder;
        if (std::filesystem::exists(p)) return p.string();
    } catch (...) {
        // ignore and fallback
    }
}

IKSolverWrapper::IKSolverWrapper() 
    : pModule(nullptr), pIKSolverInstance(nullptr), initialized(false) {
    // 初始化Python解释器
    Py_Initialize();
    PyRun_SimpleString("import sys");
    // 配置 Python 包路径，使 ik_solver.py 能 import pinocchio（带 casadi
    // 支持）、casadi、meshcat 等。
    //
    // 背景：ROS 自带的 pinocchio 编译时未启用 casadi 支持，导致
    // `from pinocchio import casadi` 失败。/opt/openrobots 下有从源码
    // 编译的 pinocchio 4.1.0（带 casadi 支持和 Python 绑定），
    // 以及配套的 casadi 3.7.2 Python 模块。
    //
    // 路径优先级（从前到后）：
    //   1. 环境变量 PINOCCHIO_CASADI_PATH（用户自定义，兼容旧配置）
    //   2. /opt/openrobots/lib/python3.12/site-packages（默认）
    //   3. ROS2 自带 Python 包（/opt/ros/jazzy/lib/python3.12/site-packages）
    //
    // 注意：必须用 sys.path.insert(0, ...) 让 /opt/openrobots 的 pinocchio
    // 优先于 ROS 的版本被加载，否则 ROS 的 pinocchio（无 casadi）会被先
    // 导入并缓存到 sys.modules，导致后续无法切换。ROS 的 coal Python
    // 模块仍需通过 sys.path 保留来提供碰撞检测支持。
    {
        std::string py_path;
        const char* env_path = std::getenv("PINOCCHIO_CASADI_PATH");
        if (env_path && std::filesystem::exists(env_path)) {
            py_path = env_path;
        } else {
            std::filesystem::path candidate =
                "/opt/openrobots/lib/python3.12/site-packages";
            if (std::filesystem::exists(candidate)) {
                py_path = candidate.string();
            }
        }
        // 先插入 openrobots 路径（优先级最高）
        if (!py_path.empty()) {
            std::string cmd = "sys.path.insert(0, \"" + py_path + "\")";
            PyRun_SimpleString(cmd.c_str());
        }
        // 再追加 ROS2 Python 路径（rclpy、coal 等 ROS 相关包仍需要）
        {
            std::filesystem::path ros_py_path =
                "/opt/ros/jazzy/lib/python3.12/site-packages";
            if (std::filesystem::exists(ros_py_path)) {
                std::string cmd =
                    "sys.path.append(\"" + ros_py_path.string() + "\")";
                PyRun_SimpleString(cmd.c_str());
            }
        }
    }
    PyGILState_STATE gstate = PyGILState_Ensure();
    
    // 初始化numpy
    initNumpy();
}

IKSolverWrapper::~IKSolverWrapper() {
    // 清理Python对象
    Py_XDECREF(pIKSolverInstance);
    Py_XDECREF(pModule);
    
    // 结束Python解释器
    Py_Finalize();
}

int IKSolverWrapper::initNumpy() {
    //手动实现初始化numpy数组
    if (_import_array() < 0) {
        PyErr_Print();
        PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
        return -1;
    }
    return 0;
}

bool IKSolverWrapper::initialize(

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

) {
    try {
        // 导入Python模块

        // 必须在任何 Python API 调用之前初始化解释器
        if (!Py_IsInitialized()) {
            Py_Initialize();
            if (!Py_IsInitialized()) {
                fprintf(stderr, "Failed to initialize Python interpreter!\n");
                return false;
            }
        }

        // 1. 获取 Python 的 sys 模块
        PyObject* pSysModule = PyImport_ImportModule("sys");
        if (!pSysModule) {
            PyErr_Print();
            throw std::runtime_error("Failed to import sys module");
        }

        // 2. 获取 sys.path 对象
        PyObject* pSysPath = PyObject_GetAttrString(pSysModule, "path");
        Py_DECREF(pSysModule); // 释放 sys 模块
        if (!pSysPath) {
            PyErr_Print();
            throw std::runtime_error("Failed to get sys.path");
        }

        // 3. 将 ik_solver.py 所在目录添加到 sys.path
        std::string ikSolverPath = get_packages_path("snowbot_dual_arm_control", "scripts");
        PyObject* pPath = PyUnicode_FromString(ikSolverPath.c_str());
        if (!pPath) {
            Py_DECREF(pSysPath);
            PyErr_Print();
            throw std::runtime_error("Failed to create path string");
        }

        if (PyList_Append(pSysPath, pPath) == -1) {
            Py_DECREF(pSysPath);
            Py_DECREF(pPath);
            PyErr_Print();
            throw std::runtime_error("Failed to append path to sys.path");
        }

        PyObject* pName = PyUnicode_FromString("ik_solver");
        pModule = PyImport_Import(pName);

        if (!pModule) {
            PyErr_Print();
            throw std::runtime_error("Failed to load ik_solver module");
        }

        //1. 获取RobotModelDataWrap类
        PyObject* pRobotModelDataWrap = PyObject_GetAttrString(pModule, "RobotModelDataWrap");
        if (!pRobotModelDataWrap || !PyCallable_Check(pRobotModelDataWrap)) {
            throw std::runtime_error("Cannot find RobotModelDataWrap class");
        }

        // 创建RobotModelDataWrap实例
        PyObject* pModelArgs = PyTuple_New(2);
        
        // 第一个参数：model_path
        PyTuple_SetItem(pModelArgs, 0, PyUnicode_FromString(urdf_path.c_str()));
        
        // 第二个参数：ArmEndLabel
        PyObject* pArmEndLabel = stringVectorToPythonList(arm_end_labels);
        PyTuple_SetItem(pModelArgs, 1, pArmEndLabel);

        PyObject* pModelWrap = PyObject_CallObject(pRobotModelDataWrap, pModelArgs);
        Py_DECREF(pModelArgs);
        Py_DECREF(pRobotModelDataWrap);

        if (!pModelWrap) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotModelDataWrap instance");
        }

        //2. 获取RobotModelLim类
        PyObject* pRobotModelLim = PyObject_GetAttrString(pModule, "RobotModelLim");
        if (!pRobotModelLim || !PyCallable_Check(pRobotModelLim)) {
            throw std::runtime_error("Cannot find RobotModelLim class");
        }

        // 创建RobotModelLim实例
        PyObject* pLimArgs = PyTuple_New(4);
        PyTuple_SetItem(pLimArgs, 0, PyBool_FromLong(use_joint_limits));
        
        // 受限关节列表
        PyObject* pLimJointList = stringVectorToPythonList(limited_joints);
        PyTuple_SetItem(pLimArgs, 1, pLimJointList);
        
        PyTuple_SetItem(pLimArgs, 2, PyBool_FromLong(display_joint_info));
        
        // 受限关节角度列表
        PyObject* pLimJointAngles = vectorToPythonList(limited_joint_angles);
        PyTuple_SetItem(pLimArgs, 3, pLimJointAngles);

        PyObject* pLimMsgWrap = PyObject_CallObject(pRobotModelLim, pLimArgs);
        Py_DECREF(pLimArgs);
        Py_DECREF(pRobotModelLim);

        if (!pLimMsgWrap) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotModelLim instance");
        }

        //3. 获取OptiInte类
        PyObject* pOptiInte = PyObject_GetAttrString(pModule, "OptiInte");
        if (!pOptiInte || !PyCallable_Check(pOptiInte)) {
            throw std::runtime_error("Cannot find OptiInte class");
        }

        // 创建OptiInte实例
        PyObject* pOptiArgs = PyTuple_New(10);

        // 获取控制器参数 -- 显示传递5个参数，其他的使用默认值
        // 优化过程中的基坐标系与末端坐标系——采用默认值
        PyTuple_SetItem(pOptiArgs, 0, PyUnicode_FromString("body_link_2"));
        PyTuple_SetItem(pOptiArgs, 1, PyLong_FromLong(21));
        PyTuple_SetItem(pOptiArgs, 2, PyLong_FromLong(39));

        // 优化器权重设置——需要传入
        PyObject* pOptiWeight = PyList_New(optiWeight.size());  //创建python list对象
        for(size_t i = 0; i < optiWeight.size(); i++){
            PyList_SetItem(pOptiWeight, i, PyFloat_FromDouble(optiWeight[i]));
        }
        PyTuple_SetItem(pOptiArgs, 3, pOptiWeight);

        // 滤波器设置——使用默认值
        PyObject* pFtrWeight = PyList_New(0);
        PyTuple_SetItem(pOptiArgs, 4, pFtrWeight);
        PyTuple_SetItem(pOptiArgs, 5, PyLong_FromLong(14));

        // 优化器控制量设置
        PyTuple_SetItem(pOptiArgs, 6, PyLong_FromLong(template_times));
        PyTuple_SetItem(pOptiArgs, 7, PyFloat_FromDouble(errTol));
        PyTuple_SetItem(pOptiArgs, 8, PyLong_FromLong(ifPrintLevel));
        PyTuple_SetItem(pOptiArgs, 9, PyBool_FromLong(ifPrintOptiTime));

        PyObject* pOptiInteInstance = PyObject_CallObject(pOptiInte, pOptiArgs);
        
        Py_DECREF(pOptiArgs);
        Py_DECREF(pOptiInte);

        if (!pOptiInteInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create OptiInte instance");
        }

        //4. 获取ViewMsg类
        PyObject* pViewMsg = PyObject_GetAttrString(pModule, "ViewMsg");
        if (!pViewMsg || !PyCallable_Check(pViewMsg)) {
            throw std::runtime_error("Cannot find ViewMsg class");
        }

        // 创建ViewMsg实例
        PyObject* pViewArgs = PyTuple_New(2);
        PyTuple_SetItem(pViewArgs, 0, PyBool_FromLong(use_visualization));
        PyTuple_SetItem(pViewArgs, 1, PyBool_FromLong(display_messages));
        PyObject* pViewMsgInstance = PyObject_CallObject(pViewMsg, pViewArgs);
        Py_DECREF(pViewArgs);
        Py_DECREF(pViewMsg);

        if (!pViewMsgInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create ViewMsg instance");
        }

        //5. 初始关节角度
        PyObject* pInitialQ = vectorToPythonList(initial_q);

        // 获取DualArmIK类
        PyObject* pDualArmIKClass = PyObject_GetAttrString(pModule, "DualArmIK");
        if (!pDualArmIKClass || !PyCallable_Check(pDualArmIKClass)) {
            throw std::runtime_error("Cannot find DualArmIK class");
        }

        // 创建DualArmIK实例
        PyObject* pInitArgs = PyTuple_New(5);
        PyTuple_SetItem(pInitArgs, 0, pModelWrap);
        PyTuple_SetItem(pInitArgs, 1, pInitialQ);    
        PyTuple_SetItem(pInitArgs, 2, pLimMsgWrap);
        PyTuple_SetItem(pInitArgs, 3, pOptiInteInstance);
        PyTuple_SetItem(pInitArgs, 4, pViewMsgInstance);

        pIKSolverInstance = PyObject_CallObject(pDualArmIKClass, pInitArgs);
        Py_DECREF(pInitArgs);
        Py_DECREF(pDualArmIKClass);

        if (!pIKSolverInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create DualArmIK instance");
        }

        //----------所有销毁统一放到最后执行----------

        //释放模型安装辅助参数
        Py_DECREF(pPath);   //释放路径字符串
        Py_DECREF(pName);   //释放名称字符串

        //1. 销毁pModelWrap -- 模型包装
        Py_DECREF(pModelWrap);

        //2. 销毁pLimMsgWrap -- 模型限制包装
        Py_DECREF(pLimMsgWrap);

        //3. 销毁机器人控制参数包装变量
        Py_DECREF(pOptiInteInstance);

        //4. 销毁机器人信息展示参数包装变量
        Py_DECREF(pViewMsgInstance);

        initialized = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Initialization error: " << e.what() << std::endl;
        return false;
    }
}

/*
求解逆解的封装代码

*/
std::pair<KDL::JntArray, KDL::JntArray> IKSolverWrapper::solveIK(
    KDL::Frame left_wrist_pose,
    KDL::Frame  right_wrist_pose,
    const std::vector<double>& current_q
) {
    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    std::pair<KDL::JntArray, KDL::JntArray> dualArmJnt;

    try {
        // 获取solve_ik方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "solveIk_ListRet");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find solveIk_ListRet method");
        }

        // 准备参数
        PyObject* pArgs = PyTuple_New(current_q.empty() ? 2 : 3);
        
        // 左腕位姿 (4x4矩阵)
        PyObject* pLeftPose = poseToPythonObject(left_wrist_pose);
        PyTuple_SetItem(pArgs, 0, pLeftPose);

        // 右腕位姿 (4x4矩阵)
        PyObject* pRightPose = poseToPythonObject(right_wrist_pose);
        PyTuple_SetItem(pArgs, 1, pRightPose);

        // 当前关节角度 (可选)
        if (!current_q.empty()) {
            PyObject* pCurrentQ = vectorToPythonList(current_q);
            PyTuple_SetItem(pArgs, 2, pCurrentQ);
        }

        // 调用方法
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("solve_ik call failed");
        }

        // 转换结果
        std::vector<double> result = pythonListToVector(pResult);
        dualArmJnt = pyListToDualArmJntArray(pResult);

        Py_DECREF(pResult);

        return dualArmJnt;
    } catch (const std::exception& e) {
        std::cerr << "IK solving error: " << e.what() << std::endl;
        return dualArmJnt;
    }
}

std::pair<KDL::Frame, KDL::Frame> IKSolverWrapper::solveFK(
    KDL::JntArray lArm_pos, KDL::JntArray rArm_pos
) {
    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    //输出结果包装：
    KDL::Frame lArmFrame;
    KDL::Frame rArmFrame;

    try {
        // 获取solveFk方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "solveFk_ListRet");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find solveFk_ListRet method");
        }

        // 准备参数
        PyObject* pQ = JntArrayToPyObject(lArm_pos, rArm_pos);
        PyObject* pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 0, pQ);

        // 调用方法
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("solveFk call failed");
        }

        // 解析结果 (应该是包含两个4x4矩阵的元组)
        if (!PyTuple_Check(pResult) || PyTuple_Size(pResult) != 2) {
            throw std::runtime_error("Invalid solveFk result format");
        }

        // 解析左臂位姿
        PyObject* pLeftPose = PyTuple_GetItem(pResult, 0);
        lArmFrame = pyListToFrame(pLeftPose);
        // 解析右臂位姿
        PyObject* pRightPose = PyTuple_GetItem(pResult, 1);
        rArmFrame = pyListToFrame(pRightPose);

        Py_DECREF(pResult);
        return {lArmFrame, rArmFrame};
        
    } catch (const std::exception& e) {
        std::cerr << "FK solving error: " << e.what() << std::endl;
        return {lArmFrame, rArmFrame};
    }
}

bool IKSolverWrapper::updateJointLimits(
    bool use_limits,
    const std::vector<std::string>& limited_joints,
    const std::vector<double>& limited_joint_angles,
    bool display_messages
) {
    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    try {
        // 获取RobotModelLim类
        PyObject* pRobotModelLim = PyObject_GetAttrString(pModule, "RobotModelLim");
        if (!pRobotModelLim || !PyCallable_Check(pRobotModelLim)) {
            throw std::runtime_error("Cannot find RobotModelLim class");
        }

        // 创建RobotModelLim实例
        PyObject* pLimArgs = PyTuple_New(4);
        PyTuple_SetItem(pLimArgs, 0, PyBool_FromLong(use_limits));
        
        // 受限关节列表
        PyObject* pLimJointList = stringVectorToPythonList(limited_joints);
        PyTuple_SetItem(pLimArgs, 1, pLimJointList);
        
        PyTuple_SetItem(pLimArgs, 2, PyBool_FromLong(display_messages));
        
        // 受限关节角度列表
        PyObject* pLimJointAngles = vectorToPythonList(limited_joint_angles);
        PyTuple_SetItem(pLimArgs, 3, pLimJointAngles);

        PyObject* pLimMsgWrap = PyObject_CallObject(pRobotModelLim, pLimArgs);
        Py_DECREF(pLimArgs);
        Py_DECREF(pRobotModelLim);

        if (!pLimMsgWrap) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotModelLim instance");
        }

        // 获取JntLimReDef方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "JntLimReDef");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            Py_DECREF(pLimMsgWrap);
            throw std::runtime_error("Cannot find JntLimReDef method");
        }

        // 调用方法
        PyObject* pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 0, pLimMsgWrap);
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("JntLimReDef call failed");
        }

        bool success = PyObject_IsTrue(pResult);
        Py_DECREF(pResult);
        return success;
    } catch (const std::exception& e) {
        std::cerr << "Update joint limits error: " << e.what() << std::endl;
        return false;
    }
}

// 辅助函数实现
PyObject* IKSolverWrapper::vectorToPythonList(const std::vector<double>& vec) {
    PyObject* list = PyList_New(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        PyList_SetItem(list, i, PyFloat_FromDouble(vec[i]));
    }
    return list;
}

PyObject* IKSolverWrapper::stringVectorToPythonList(const std::vector<std::string>& vec) {
    PyObject* list = PyList_New(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        PyList_SetItem(list, i, PyUnicode_FromString(vec[i].c_str()));
    }
    return list;
}

std::vector<double> IKSolverWrapper::pythonListToVector(PyObject* pyList) {
    std::vector<double> vec;
    if (PyList_Check(pyList)) {
        Py_ssize_t size = PyList_Size(pyList);
        vec.reserve(size);
        for (Py_ssize_t i = 0; i < size; ++i) {
            PyObject* item = PyList_GetItem(pyList, i);
            vec.push_back(PyFloat_AsDouble(item));
        }
    } else if (PyArray_Check(pyList)) {
        // 处理numpy数组
        double* data = (double*)PyArray_DATA((PyArrayObject*)pyList);
        npy_intp size = PyArray_SIZE((PyArrayObject*)pyList);
        vec.assign(data, data + size);
    }
    return vec;
}

std::vector<std::string> IKSolverWrapper::pythonListToStringVector(PyObject* pyList) {
    std::vector<std::string> vec;
    if (PyList_Check(pyList)) {
        Py_ssize_t size = PyList_Size(pyList);
        vec.reserve(size);
        for (Py_ssize_t i = 0; i < size; ++i) {
            PyObject* item = PyList_GetItem(pyList, i);
            const char* str = PyUnicode_AsUTF8(item);
            if (str) {
                vec.emplace_back(str);
            }
        }
    }
    return vec;
}

std::pair<KDL::JntArray, KDL::JntArray> IKSolverWrapper::pyListToDualArmJntArray(PyObject* pyList){
    KDL::JntArray lArmJntArray;
    KDL::JntArray rArmJntArray;
    lArmJntArray.resize(LarmDOF);
    rArmJntArray.resize(RarmDOF);

    if(PyList_Check(pyList)){
        //针对一般的列表进行处理
        Py_ssize_t size = PyList_Size(pyList);
        if(size != LarmDOF + RarmDOF){
            std::cerr << "错误：机器人求解器返回结果自由度数目与模型关节数目不匹配" << 
                "机器人求解器返回自由度数目：" << size <<
                "模型关节数目：" << LarmDOF + RarmDOF << std::endl;
                return {lArmJntArray, rArmJntArray};
        }

        //将数据依次装入两个JntArray
        // 填充左臂和右臂的关节数组
        for (int i = 0; i < LarmDOF; ++i) {
            PyObject* item = PyList_GetItem(pyList, i);
            lArmJntArray(i) = PyFloat_AsDouble(item);
        }
        for (int i = 0; i < RarmDOF; ++i) {
            PyObject* item = PyList_GetItem(pyList, LarmDOF + i);
            rArmJntArray(i) = PyFloat_AsDouble(item);
        }

        return {lArmJntArray, rArmJntArray};
    }else{
        std::cerr << "错误：关节角度值应当返回类型为列表" << std::endl;
        return {lArmJntArray, rArmJntArray};
    }
}

KDL::Frame IKSolverWrapper::pyListToFrame(PyObject* pyList){
    // 检查输入是否为列表或元组
    if (!PyList_Check(pyList) && !PyTuple_Check(pyList)) {
        throw std::runtime_error("Input is not a list or tuple");
    }

    // 检查长度是否为 16（4x4 矩阵展开）
    Py_ssize_t size = PySequence_Size(pyList);
    if (size != 16) {
        throw std::runtime_error("Input list must have 16 elements (4x4 matrix)");
    }

    // 提取数据到 C++ 数组
    std::vector<double> matrixData;
    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* item = PySequence_GetItem(pyList, i);
        if (!PyFloat_Check(item) && !PyLong_Check(item)) {
            Py_DECREF(item);
            throw std::runtime_error("List elements must be numbers");
        }
        matrixData.push_back(PyFloat_AsDouble(item));
        Py_DECREF(item);
    }

    // 构造 KDL::Rotation（3x3 部分）和 KDL::Vector（平移部分）
    KDL::Rotation rotation;
    KDL::Vector translation;

    // 按行优先填充旋转矩阵（前 3x4 部分）
    for (int i = 0; i < 3; ++i) {        // 行
        for (int j = 0; j < 3; ++j) {    // 列
            rotation(i, j) = matrixData[i * 4 + j];
        }
        translation[i] = matrixData[i * 4 + 3]; // 第 4 列是平移
    }

    // 构造 KDL::Frame 并返回
    return KDL::Frame(rotation, translation);
}

PyObject* IKSolverWrapper::poseToPythonObject(KDL::Frame pose) {
    npy_intp dims[2] = {4, 4};
    PyObject* obj = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
    double* data = (double*)PyArray_DATA((PyArrayObject*)obj);
    
    //填充 4x4 位姿矩阵
    //旋转部分
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            data[i*4 + j] = pose.M(i, j);
        }
    }
    //平移部分：
    data[0*4 + 3] = pose.p.x();
    data[1*4 + 3] = pose.p.y();
    data[2*4 + 3] = pose.p.z();
    //剩余部分填充
    data[3*4 + 0] = 0;
    data[3*4 + 1] = 0;
    data[3*4 + 2] = 0;
    data[3*4 + 3] = 1;

    return obj;
}

void IKSolverWrapper::optiWeight_def(
    int optiWeight_mod,
    const std::vector<double>& optiWeight_selfDefine,
    bool ifUseFtr
){
    try{
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "optiWeight_update");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find optiWeight_update method");
        }

        PyObject* pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, PyLong_FromLong(optiWeight_mod));
        PyTuple_SetItem(pArgs, 1, vectorToPythonList(optiWeight_selfDefine));
        PyTuple_SetItem(pArgs, 2, PyBool_FromLong(ifUseFtr));

        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("optiWeight_update call failed");
        }

        Py_DECREF(pResult);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

    }catch(const std::exception& e) {
        std::cerr << "optimize Weight update error: " << e.what() << std::endl;
    }
}

PyObject* IKSolverWrapper::JntArrayToPyObject(KDL::JntArray lArm_pos, KDL::JntArray rArm_pos){
    PyObject* list = PyList_New(lArm_pos.rows() + rArm_pos.rows());

    size_t temp = 0;
    for(size_t i = 0; i < lArm_pos.rows(); i++){
        PyList_SetItem(list, temp, PyFloat_FromDouble(lArm_pos(i)));
        temp += 1;
    }
    for(size_t j = 0; j < rArm_pos.rows(); j++){
        PyList_SetItem(list, temp, PyFloat_FromDouble(rArm_pos(j)));
        temp += 1;
    }
    return list;
}

bool IKSolverWrapper::checkPythonCall(PyObject* pResult, const std::string& funcName) {
    if (!pResult) {
        PyErr_Print();
        std::cerr << "Error calling Python function: " << funcName << std::endl;
        return false;
    }
    return true;
}
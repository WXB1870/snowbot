import numpy as np
import sys
from dataclasses import dataclass, field
from typing import List
from scipy.spatial.transform import Rotation

import os
import csv

sys.path.insert(0, "/opt/openrobots/lib/python3.12/site-packages")

import meshcat.geometry as mg
import pinocchio as pin                           
from pinocchio.visualize import MeshcatVisualizer
from weighted_moving_filter import WeightedMovingFilter
import casadi
from pinocchio import casadi as cpin

'''
        by 霜云喵
-------------------------
逆解求解封装模块
-------------------------
协议：
1. 目标位姿输入值是相对于body_link_2的位姿

2. 所有角度均采用弧度制表示

3. 初始化过程中需要输入机器人当前的角度，如果开启角度限制，则限制关节定死的角度就是初始化的角度

4. 机器人关节顺序不能乱，必须遵从从左臂到右臂，关节ID从小到大的顺序，否则会出bug

-------------------------
修改：
1. 将位置限制的过程挪到DualArmIK类的初始化中进行，并将初始模型以及位置限制后的模型分别进行包装
2. 修改初始角度
3. 修改优化机制，添加速度连续性指标并添加相应初始化代码

'''

#利用casadi做的逆解求解器
@dataclass
class RobotModelDataWrap:
    #python数据包装类，用于模型数据信息包装与传输
    model_path: str      #原始机器人模型路径包装
    ArmEndLabel: List[str]              #机器人末端标签

@dataclass
class RobotModelLim:
    #python数据包装类，用于限制机器人特定关节的角度
    ifUseLim: bool      #判断量：是否使用位置限制
    LimJntLabelList: List[str]      #受限关节标签的列表
    ifDispJntMsg: bool  #开关量：是否打印关节信息

    LimJntAngList: List[float] = field(default_factory=list)      #受限关节角度的列表

    def __post_init__(self):
        #针对列表在初始化之后填充默认值：
        if not self.LimJntAngList:  #若用户未输入，则设定初始值
            self.LimJntAngList = []     #默认为空列表

@dataclass
class OptiInte:
    #python数据包装类，用于设定优化目标函数信息

    #优化过程基坐标系与末端坐标系
    baseFrameName: str = "body_link_2"
    lArmEndID: int = 21   # deprecated, use ArmEndLabel instead
    rArmEndID: int = 39   # deprecated, use ArmEndLabel instead

    #优化器权重设置
    optiWeight: List[float] = field(default_factory=list)       #优化权重设置

    #滤波器参数设置
    FtrWeight: List[float] = field(default_factory=list)        #滤波器权重
    FtrDataLen: int = 14                                        #滤波器数据队列长度

    #优化器控制量：
    max_iter: int = 500         #最大迭代次数
    tol: float = 1e-5           #收敛容差度
    print_level: int = 0        #开关量——是否打印优化日志（默认不打印）
    print_time: bool = False    #开关量——是否打印计算时间（默认不打印）

    #优化过程的迭代时间(单位：s)
    tempTime: float = 0.02

    def __post_init__(self):
        #在初始化之后填充默认值
        if not self.optiWeight:     #若用户未输入，则设定为初始值
            self.optiWeight = [50.0, 1.0, 0.002, 0.01, 0.0002]
        if not self.FtrWeight:
            self.FtrWeight = [0.4, 0.3, 0.2, 0.1]

@dataclass
class ViewMsg:
    #python数据包装类，用于可视化信息传输
    ifVisu: bool = False                #是否使用机器人可视化
    ifUseMsgDisp: bool = False          #是否打印机器人信息

#numpy矩阵打印简易代码
def print_matrix(matrix):
    """
    numpy矩阵打印
    """
    print("矩阵信息:")
    for row in matrix:
        print("[", end=" ")
        for val in row:
            print(f"{val:.4f}", end="\t")  # 保留 4 位小数
        print("]")

#机器人模型关节信息打印简易代码
def printJointInfo(model):
    """打印模型关节信息"""
    print("====================")
    print("DOF num of robot model is = {model.nq}")
    print("====================")
    for name, joint in zip(model.names, model.joints):
        print(f"Joint: {name}, ID: {joint.id}")

#遍历模型，依据模型受限关节名称获取其ID
def getIDviaName(model, nameList: list[str]):
    #遍历模型，依据模型受限关节名称获取其ID
    JntID = []
    for JntName in nameList:
        for name, joint in zip(model.names, model.joints):
            if JntName == name:
                JntID.append(joint.id)

    return JntID

#辅助函数：
def getRobotModel(urdf_path: str):
    robot_model = pin.RobotWrapper.BuildFromURDF(urdf_path)
    return robot_model

#辅助函数：SE3对象化作numpy数组
def pinoSE3ToNp(M):
    R = np.array(M.rotation)
    t = np.array(M.translation).reshape(3, 1)
    homogeneous_matrix = np.vstack([
        np.hstack([R, t]),
        np.array([0, 0, 0, 1])
    ])
    return homogeneous_matrix

#辅助函数：将位置矩阵转化为[x,y,z,r,p,y]六维旋量的代码
def pose_matrix_to_euler(pose_matrix):
    """
    从 4×4 位姿矩阵中提取 (x, y, z, roll, pitch, yaw)
    
    参数:
        pose_matrix: 4×4 齐次变换矩阵（numpy.ndarray）
    
    返回:
        x, y, z, roll, pitch, yaw: 位置和欧拉角（弧度）
    """
    # 提取平移部分
    x = pose_matrix[0, 3]
    y = pose_matrix[1, 3]
    z = pose_matrix[2, 3]
    
    # 提取旋转矩阵
    R = pose_matrix[:3, :3]
    
    # 计算 pitch
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    # 处理万向节锁（pitch 接近 ±90 度）
    if np.isclose(np.abs(pitch), np.pi / 2):
        roll = 0
        yaw = np.arctan2(R[0, 1], R[1, 1])
    else:
        # 计算 roll
        roll = np.arctan2(R[2, 1], R[2, 2])
        # 计算 yaw
        yaw = np.arctan2(R[1, 0], R[0, 0])
    
    return x, y, z, roll, pitch, yaw

class DualArmIK:

    #初始化函数
    def __init__(self, 
                 modelWrap: RobotModelDataWrap,       #机器人模型包裹
                 model_q_st,        #机器人关节初始角度(numpy数组)
                 m_limMsgWrap: RobotModelLim,
                 m_optiInte: OptiInte,          #机器人控制参数
                 m_viewInte: ViewMsg            #机器人信息展示的参数
                 ):
        #装入原始的机器人模型 (跳过mesh加载避免STL文件问题)
        try:
            self.origin_robot = pin.RobotWrapper.BuildFromURDF(modelWrap.model_path)
        except ValueError:
            model = pin.buildModelFromUrdf(modelWrap.model_path)
            self.origin_robot = pin.RobotWrapper(model, pin.GeometryModel(), pin.GeometryModel())
        self.originRobot_DOF = self.origin_robot.model.nq     #原始模型自由度数量
        self.lArm_EndLabel = modelWrap.ArmEndLabel[0]
        self.rArm_EndLabel = modelWrap.ArmEndLabel[1]
        if m_limMsgWrap.ifDispJntMsg:
            print("限位前关节：")
            printJointInfo(self.origin_robot.model)

        #针对机器人角度进行限制
        self.ifUseLim = m_limMsgWrap.ifUseLim                   #判断量：是否开启关节限制
        self.limJntLabel = m_limMsgWrap.LimJntLabelList         #受限关节标签
        self.limJntID = getIDviaName(self.origin_robot.model, self.limJntLabel)     #受限关节ID
        self.LimJntAng = list(m_limMsgWrap.LimJntAngList) if m_limMsgWrap.LimJntAngList else []  #受限关节角度

        # 构建完整参考构型 (全模型DOF)，锁关节用指定角度，其余用传入的手臂角度
        full_nq = self.origin_robot.model.nq
        self.jntRef = np.zeros(full_nq)
        arm_q_idx = 0
        for jnt in self.origin_robot.model.joints:
            if jnt.nq > 0:  # movable joint
                idx = jnt.idx_q
                jnt_id = jnt.id
                if jnt_id in self.limJntID:
                    lim_idx = self.limJntID.index(jnt_id)
                    self.jntRef[idx] = self.LimJntAng[lim_idx] if lim_idx < len(self.LimJntAng) else 0.0
                else:
                    if arm_q_idx < len(model_q_st):
                        self.jntRef[idx] = model_q_st[arm_q_idx]
                    arm_q_idx += 1

        self.ifLstUseLim = m_limMsgWrap.ifUseLim             #辅助变量 —— 判断上一轮初始化中是否加了限制，用于后续更新过程
        self.LastLimJntLabel = m_limMsgWrap.LimJntLabelList     #辅助变量 —— 用于存储上一轮限制中限制了哪几个关节
        self.LastLimJntID = getIDviaName(self.origin_robot.model, self.LastLimJntLabel)     #辅助变量 —— 用于储存上一轮的限制中限制关节的ID
        self.LastLimJntAng = list(self.LimJntAng)             #辅助变量 —— 用于储存上一轮限制中限制的关节的角度

        #创建受限的机器人模型 -- 实际求解中用受限机器人模型求解
        if self.ifUseLim:

            #创建受限机器人模型
            self.reduced_robot = self.origin_robot.buildReducedRobot(
                self.limJntLabel,
                self.jntRef,
            )
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            #获取当前机器人模型所有关节角度，作为初始角度
            self.q_st = np.array(self.reduced_robot.q0)

            #打印信息
            if m_limMsgWrap.ifDispJntMsg:
                print("限位后机器人关节信息：")
                printJointInfo(self.reduced_robot.model)
                print("机器人关节初始角度：")
                print(self.q_st)
        else:
            self.reduced_robot = self.origin_robot
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            #获取当前机器人模型所有关节角度，作为初始角度
            self.q_st = np.array(self.reduced_robot.q0)

            if m_limMsgWrap.ifDispJntMsg:
                print("未使用关节限位")
                print("机器人关节初始角度：")
                print(self.q_st)

        #开关量：
        self.Visualization = m_viewInte.ifVisu      #开关量——是否使用可视化
        self.ifUseMsgDisp = m_viewInte.ifUseMsgDisp        #开关量——是否打印机器人信息

        #打印机器人信息 -- 关节名称、关节ID、关节类型、关节限位（若有）
        if self.ifUseMsgDisp:
            print("\n===== Robot Model Information =====")
            print(f"Model Name: {self.reduced_robot.model.name}")
            print(f"Degrees of Freedom (DOF): {self.reduced_robot.model.nq}")
            
            # 打印关节信息
            print("\nJoints Information:")
            for i in range(self.reduced_robot.model.njoints):
                joint_name = self.reduced_robot.model.names[i]
                joint_type = "Revolute" if self.reduced_robot.model.joints[i].nq == 1 else "Fixed"
                print(f"  ID: {i}, Name: {joint_name}, Type: {joint_type}")
            
            # 打印末端执行器信息（仅使用输入参数 ArmEndLabel，避免访问未初始化的属性）
            print("\nEnd Effectors (Input Labels):")
            print(f"  Left Arm: {self.lArm_EndLabel}")
            print(f"  Right Arm: {self.rArm_EndLabel}")
            
            # 打印关节限位（如果有）
            print("\nJoint Position Limits:")
            lower_limits = self.reduced_robot.model.lowerPositionLimit
            upper_limits = self.reduced_robot.model.upperPositionLimit
            for i in range(self.reduced_robot.model.nq):
                print(f"  Joint {i}: Lower={lower_limits[i]:.4f}, Upper={upper_limits[i]:.4f}")

             # 添加打印帧信息的代码
            print("\nFrame Information:")
            print(f"Total number of frames: {len(self.reduced_robot.model.frames)}")
            print("\nJOINT and BODY frames:")
            for frame_id, frame in enumerate(self.reduced_robot.model.frames):
                if frame.type == pin.FrameType.JOINT or frame.type == pin.FrameType.BODY:
                    frame_type = "JOINT" if frame.type == pin.FrameType.JOINT else "BODY"
                    parent_joint_name = self.reduced_robot.model.names[frame.parent]
                    print(f"  ID: {frame_id}, Type: {frame_type}, Name: {frame.name}, Parent Joint: {parent_joint_name}")

            print("===================================\n")

        #建立casadi逆解求解器模型
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        #casadi逆解求解参数确定
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        #获取末端ID并定义误差函数
        self.base_frame_id = self.reduced_robot.model.getFrameId(m_optiInte.baseFrameName)
        self.L_hand_id = self.reduced_robot.model.getFrameId(self.lArm_EndLabel)
        self.R_hand_id = self.reduced_robot.model.getFrameId(self.rArm_EndLabel)

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,3]
                )
            ]
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,:3].T)
                )
            ]
        )

        #优化问题：
        self.tempTime = m_optiInte.tempTime

        self.opti = casadi.Opti()       #初始化优化问题
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)    #定义优化变量：机器人关节配置

        #定义参数(固定输入)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)      #平滑项辅助参数 -- 上一次关节位置
        self.var_v_last = self.opti.parameter(self.reduced_robot.model.nq)      #平滑项辅助参数 -- 上一次关节速度
        self.param_tf_l = self.opti.parameter(4, 4)     #左臂目标位置(齐次变换矩阵)
        self.param_tf_r = self.opti.parameter(4, 4)     #右臂目标位置(齐次变换矩阵)
        #定义目标函数(成本函数)：
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #位移误差成本
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #旋转误差成本
        self.regularization_cost = casadi.sumsqr(self.var_q)            #正则化成本(避免关节角度过大)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)      #平滑成本(最小化当前关节配置与上一时刻配置差值，使运动平滑)
        self.vel_smooth_cost = casadi.sumsqr((self.var_q - self.var_q_last)/self.tempTime - self.var_v_last)    #平滑成本（最小化当前关节速度与上一刻差值）


        #优化问题约束(关节位置限制)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit
            )
        )

        #优化目标函数(成本函数的加权组合)
        self.Weight_param = self.opti.parameter(5)
        self.opti.minimize(
            self.Weight_param[0] * self.translational_cost 
            + self.Weight_param[1] * self.rotation_cost 
            + self.Weight_param[2] * self.regularization_cost 
            + self.Weight_param[3] * self.smooth_cost
            + self.Weight_param[4] * self.vel_smooth_cost
        )
        # 储存初始化权重
        self.optiWeight_origin = m_optiInte.optiWeight
        # 权重注入
        self.opti.set_value(self.Weight_param , m_optiInte.optiWeight)

        #求解器配置
        opts = {
            'ipopt': {
                'print_level': m_optiInte.print_level,       # 不打印优化日志
                'max_iter': m_optiInte.max_iter,         # 最大迭代次数
                'tol': m_optiInte.tol             # 收敛容忍度
            },
            'print_time': m_optiInte.print_time,        # 不打印计算时间
            'calc_lam_p': False         # 避免 NaN 问题（CasADi FAQ）
        }
        self.opts = opts
        self.opti.solver("ipopt", opts)

        #初始化数据
        q_st_np = np.array(self.q_st)
        if len(q_st_np) == self.reduced_robot.model.nq:
            #self.init_data = np.zeros(self.reduced_robot.model.nq)  # 初始关节配置（全零）
            self.init_data = q_st_np    #初始化关节配置（按照初始位置初始化）

            self.init_vel = np.zeros(len(self.init_data))
            self.last_init_data = self.init_data.copy()    # 保存上一次位置用于速度计算
        else:
            print(f"Error(IK-01): incorrect init input -- joint ang array len incorrect")

        #滤波器初始化
        self.FtrWeight = m_optiInte.FtrWeight
        self.smooth_filter = WeightedMovingFilter(np.array(self.FtrWeight), self.reduced_robot_DOF)  # 加权移动平均滤波器
        self.ifUseSmoothFtr = True

        #设置可视化代码
        if self.Visualization and len(self.origin_robot.visual_model.geometryObjects) > 0:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(self.origin_robot.model, self.origin_robot.collision_model, self.origin_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            
            # 如果开关量为 True，则显示所有关节的坐标系
            self.vis.displayFrames(True, frame_ids=[self.base_frame_id], axis_length=0.2, axis_width=5)
            self.vis.displayFrames(True, frame_ids=[self.L_hand_id, self.R_hand_id], axis_length = 0.15, axis_width = 5)
            
            self.vis.display(pin.neutral(self.origin_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = [self.lArm_EndLabel, self.rArm_EndLabel]
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 20
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )

#===========================================================================================================================
#辅助函数

    #辅助函数：将输入目标位姿基坐标系转换的函数
    def desPos_baseFrameTrans(self, desPos_origin):
        #原始基坐标系 -- arm_base_link
        #转换后基坐标系 -- universe

        # 获取符号旋转矩阵
        R_sym = self.cdata.oMf[self.base_frame_id].rotation  # 3x3 SX
        t_sym = self.cdata.oMf[self.base_frame_id].translation  # 3x1 SX

        # 转换为 NumPy（需要先求值）
        R_np = np.array([[float(R_sym[i, j]) for j in range(3)] for i in range(3)])
        t_np = np.array([float(t_sym[i]) for i in range(3)])

        # 构造 4x4 位姿矩阵
        left_wrist_trans_np = np.eye(4)
        left_wrist_trans_np[:3, :3] = R_np
        left_wrist_trans_np[:3, 3] = t_np

        desPos_Transed = left_wrist_trans_np @ desPos_origin
        return desPos_Transed
    
    #辅助函数：适用于numpy数组的转化方法，将目标位姿由arm_base_link为基坐标系转化到universe为基坐标系
    def desPos_baseFrameTrans_np(self, desPos_origin):
        #适用于numpy数组的转化方法
        # 获取符号旋转矩阵
        R_sym = self.cdata.oMf[self.base_frame_id].rotation  # 3x3 SX
        t_sym = self.cdata.oMf[self.base_frame_id].translation  # 3x1 SX

        # 转换为 NumPy（需要先求值）
        R_np = np.array([[float(R_sym[i, j]) for j in range(3)] for i in range(3)])
        t_np = np.array([float(t_sym[i]) for i in range(3)])

        # 构造齐次变换矩阵（4x4）
        base_to_universe = np.eye(4)
        base_to_universe[:3, :3] = R_np
        base_to_universe[:3, 3] = t_np.flatten()  # 确保是 1D 数组

        # 处理输入位姿（确保是齐次矩阵）
        if desPos_origin.shape == (3, 4):
            desPos_origin_h = np.vstack([desPos_origin, [0, 0, 0, 1]])
        elif desPos_origin.shape == (4, 4):
            desPos_origin_h = desPos_origin
        else:
            raise ValueError("输入位姿必须是 3x4 或 4x4 的 NumPy 数组")

        # 坐标变换：T_universe = T_base_to_universe * T_origin
        desPos_Transed = base_to_universe @ desPos_origin_h
        return desPos_Transed
    
    #辅助函数：适用于numpy数组的转化方法，将输入位姿由universe转化到arm_base_link
    def desPos_baseFrameTrans_inverse(self, desPos_universe):
        """
        将输入位姿从universe坐标系转换到arm_base_link坐标系
        
        参数:
            desPos_universe: 在universe坐标系中的位姿 (3x4或4x4 numpy数组)
            
        返回:
            desPos_base: 在arm_base_link坐标系中的位姿 (4x4 numpy数组)
        """
        # 转换后基坐标系 -- arm_base_link
        # 原始基坐标系 -- universe

        # 获取符号旋转矩阵和平移向量
        R_sym = self.cdata.oMf[self.base_frame_id].rotation  # 3x3 SX
        t_sym = self.cdata.oMf[self.base_frame_id].translation  # 3x1 SX

        # 转换为 NumPy（需要先求值）
        R_np = np.array([[float(R_sym[i, j]) for j in range(3)] for i in range(3)])
        t_np = np.array([float(t_sym[i]) for i in range(3)])

        # 构造从universe到arm_base_link的齐次变换矩阵
        # T_universe_to_base = T_base_to_universe^(-1)
        universe_to_base = np.eye(4)
        universe_to_base[:3, :3] = R_np.T  # 旋转矩阵的逆等于转置
        universe_to_base[:3, 3] = -R_np.T @ t_np  # 平移部分：-R^T * t

        # 处理输入位姿（确保是齐次矩阵）
        if desPos_universe.shape == (3, 4):
            desPos_universe_h = np.vstack([desPos_universe, [0, 0, 0, 1]])
        elif desPos_universe.shape == (4, 4):
            desPos_universe_h = desPos_universe
        else:
            raise ValueError("输入位姿必须是 3x4 或 4x4 的 NumPy 数组")

        # 坐标变换：T_base = T_universe_to_base * T_universe
        desPos_base = universe_to_base @ desPos_universe_h
        
        return desPos_base

    #辅助函数：类型转换 -- 将SE3类型数据转化成六维旋量
    def transSE3_to_vec(self, SE3_matrix):
        pos = SE3_matrix.translation
        rot = Rotation.from_matrix(SE3_matrix.rotation).as_euler('xyz', degrees=False)

        vecRes = np.hstack([pos, rot])
        return vecRes
    
    #辅助函数：类型转换 -- 将numpy思维数组转化成六维旋量
    def transNpMat_to_vec(self, np_Mat):
        pos = np_Mat[:3, 3]
        rot = Rotation.from_matrix(np_Mat[:3, :3]).as_euler('xyz', degrees=False)

        vecRes = np.hstack([pos, rot])
        return vecRes

#===========================================================================================================================
#功能函数

    #辅助函数：可视化信息发送与更新函数
    def qResCompleteFunc(self, q_res):
        #输入量——角度求解结果
        #将削减自由度的模型结果补全为全模型的角度数组
        q_res_send = np.array(q_res)
        if self.ifUseLim:
            full = np.zeros(self.originRobot_DOF)
            q_idx = 0
            for jnt in self.origin_robot.model.joints:
                if jnt.nq > 0:
                    if jnt.id in self.limJntID:
                        lim_idx = self.limJntID.index(jnt.id)
                        full[jnt.idx_q] = self.LimJntAng[lim_idx] if lim_idx < len(self.LimJntAng) else 0.0
                    else:
                        if q_idx < len(q_res):
                            full[jnt.idx_q] = q_res[q_idx]
                        q_idx += 1
            return full
        return q_res_send
        
#===========================================================================================================================
    #权重模式选择函数
    def optiWeight_update(self, optiWeight_mode, optiWeight_selfDefine, ifUseFtr):
        """
        权重模式选择函数
        ----------------------
        输入量：
            optiWeight_mode -- 权重输入模式：1 -- 经初始化的默认权重；2 -- 自定义权重
            optiWeight_selfDefine -- 自定义权重系数
        """

        # 将机器人参数刷新到零位
        self.init_data = self.q_st.copy()
        self.init_vel = np.zeros(len(self.init_data))
        self.last_init_data = self.init_data.copy()
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.var_q_last, self.init_data)
        self.opti.set_value(self.var_v_last, self.init_vel)

        self.ifUseSmoothFtr = ifUseFtr
        

        if optiWeight_mode == 1:
            self.opti.set_value(self.Weight_param, self.optiWeight_origin)
        else:
            self.opti.set_value(self.Weight_param, optiWeight_selfDefine)

    #逆解求解函数
    def solve_ik(self, left_wrist, right_wrist,             #目标位姿(4X4齐次变换矩阵)
                 current_lr_arm_motor_q = None,             #当前关节角度
                 ):
        #初始化优化器，current_lr_arm_motor_q 已是简化模型维度(C++侧已做reduced)，直接使用
        if current_lr_arm_motor_q is not None:
            current_q_full = np.array(current_lr_arm_motor_q)

            if hasattr(self, 'last_init_data'):
                if len(current_q_full) == len(self.last_init_data):
                    self.init_vel = (current_q_full - self.last_init_data) / self.tempTime
                    self.last_init_data = self.init_data.copy()
                    self.init_data = current_q_full.copy()
            else:
                self.init_data = current_q_full.copy()
                self.init_vel = np.zeros(len(self.init_data))
                self.last_init_data = self.init_data.copy()

        if self.Visualization:
            print(self.init_data)
        self.opti.set_initial(self.var_q, self.init_data)

        #设置优化问题参数
        self.opti.set_value(self.param_tf_l, left_wrist)  # 设置左腕目标位姿
        self.opti.set_value(self.param_tf_r, right_wrist) # 设置右腕目标位姿
        self.opti.set_value(self.var_q_last, self.init_data)  # 用于平滑过渡（避免突变）
        self.opti.set_value(self.var_v_last, self.init_vel)     # 设置上一次关节速度

        #设置可视化参数
        if self.Visualization:
            left_wrist_trans = self.desPos_baseFrameTrans(left_wrist)
            right_wrist_trans = self.desPos_baseFrameTrans(right_wrist)
            self.vis.viewer[self.lArm_EndLabel].set_transform(left_wrist_trans)   # 显示左腕目标位姿
            self.vis.viewer[self.rArm_EndLabel].set_transform(right_wrist_trans)  # 显示右腕目标位姿

        # 如果当前解已满足目标(误差<0.1mm)，跳过优化直接返回
        import casadi
        t_err_init = float(casadi.sum1(self.translational_error(self.init_data, left_wrist, right_wrist)))
        r_err_init = float(casadi.sum1(self.rotational_error(self.init_data, left_wrist, right_wrist)))
        if abs(t_err_init) < 1e-4 and abs(r_err_init) < 1e-4:
            return self.init_data.copy()

        #求解优化问题
        try:
            sol = self.opti.solve()  # 尝试求解
            sol_q = self.opti.value(self.var_q)  # 提取解（关节角度）
            if self.ifUseSmoothFtr:
                self.smooth_filter.add_data(sol_q)  # 平滑滤波
                sol_q = self.smooth_filter.filtered_data  # 获取滤波后的值

            # 更新速度和位置用于下一次迭代
            self.init_vel = (sol_q - self.init_data) / self.tempTime  # 计算新速度
            self.last_init_data = self.init_data.copy()  # 保存当前数据作为下一次的"上一次数据"
            self.init_data = sol_q.copy()  # 更新为新的解

        except:
            print(f"Error(IK-02): ERROR in IK solving -- No result found")
            sol_q = self.init_data
            self.init_data = sol_q

        #返回简化模型求解结果(手臂J1~J7，14维)
        return sol_q
    
    #逆解求解函数封装 -- 返回lists类型的数据
    def solveIk_ListRet(self, left_wrist, right_wrist,      #目标位姿(4X4齐次变换)
                        current_lr_arm_motor_q = None,
                        ):
        sol_q = self.solve_ik(left_wrist, right_wrist, current_lr_arm_motor_q)

        #sol_q_np = sol_q.full()
        sol_q_list = sol_q.tolist()
        return sol_q_list

    #正解求解函数：
    def solveFk(self, arm_motor_q):
        """
        计算机器人双臂在给定关节角度下的末端执行器位姿（正运动学）
        
        参数:
            arm_motor_q: 机器人关节角度配置 (numpy数组，形状为 [nq,] 或 [1, nq])
            
        返回:
            left_wrist_pose: 左臂末端执行器位姿 (4x4齐次变换矩阵，pinocchio.SE3格式)
            right_wrist_pose: 右臂末端执行器位姿 (4x4齐次变换矩阵，pinocchio.SE3格式)
            注意这两个返回值对应的基坐标系为universe坐标系
        """
        # 确保输入是numpy数组 (简化模型维度，直接用于FK)
        q = np.array(arm_motor_q).flatten()
        q = np.pad(q, (0, max(0, self.reduced_robot.model.nq - len(q))))

        # 创建数据对象并更新关节配置
        data = self.reduced_robot.model.createData()
        pin.forwardKinematics(self.reduced_robot.model, data, q)
        pin.updateFramePlacements(self.reduced_robot.model, data)

        left_ee_id = self.reduced_robot.model.getFrameId(self.lArm_EndLabel)
        right_ee_id = self.reduced_robot.model.getFrameId(self.rArm_EndLabel)
        left_ee_pos = data.oMf[left_ee_id]
        right_ee_pos = data.oMf[right_ee_id]

        return left_ee_pos, right_ee_pos
    
    #正解求解函数封装 -- 返回Lists类型的数据
    def solveFk_ListRet(self, arm_motor_q):
        #输入量——关节角度的列表(lists)

        #求解pinocchio::SE3类型的返回值
        left_ee_pos, right_ee_pos = self.solveFk(arm_motor_q)

        #返回值转换为列表(lists)数据类型
        left_ee_pos_np_uncor = pinoSE3ToNp(left_ee_pos)
        right_ee_pos_np_uncor = pinoSE3ToNp(right_ee_pos)

        left_ee_pos_np = self.desPos_baseFrameTrans_inverse(left_ee_pos_np_uncor)
        right_ee_pos_np = self.desPos_baseFrameTrans_inverse(right_ee_pos_np_uncor)

        left_ee_pos_list = []
        right_ee_pos_list = []

        # 左臂末端位姿转换（4x4齐次变换矩阵展开为16个元素的列表，按行优先）
        for i in range(4):  # 行
            for j in range(4):  # 列
                left_ee_pos_list.append(left_ee_pos_np[i, j])

        # 右臂末端位姿转换（4x4齐次变换矩阵展开为16个元素的列表，按行优先）
        for i in range(4):  # 行
            for j in range(4):  # 列
                right_ee_pos_list.append(right_ee_pos_np[i, j])

        # 返回左右臂位姿的列表（每个列表包含16个元素，表示4x4矩阵）
        return left_ee_pos_list, right_ee_pos_list

    #修改限位接口
    def JntLimReDef(self, LimMsg: RobotModelLim):
        #重新设定关节位置限制与初始化方法

        #获取受限信息：
        self.jntRef = np.array(self.reduced_robot.q0)
        self.ifUseLim = LimMsg.ifUseLim
        self.limJntLabel = LimMsg.LimJntLabelList
        self.limJntID = getIDviaName(self.origin_robot.model, self.limJntLabel)
        self.LimJntAng = LimMsg.LimJntAngList

        #################################################debug
        #print("redef success")
        #print(f"ref joint unprocessed = {self.jntRef}")

        # 重建全模型参考构型
        self.jntRef = np.zeros(self.originRobot_DOF)
        for jnt in self.origin_robot.model.joints:
            if jnt.nq > 0:
                if jnt.id in self.limJntID:
                    lim_idx = self.limJntID.index(jnt.id)
                    self.jntRef[jnt.idx_q] = self.LimJntAng[lim_idx] if lim_idx < len(self.LimJntAng) else 0.0
        if LimMsg.ifDispJntMsg:
            print("初始位置为：")
            print(self.jntRef)

        # init_data 保持简化模型维度不变，锁关节切换时不需要补/删

        #创建模型
        if self.ifUseLim:
            #创建简化的模型
            self.reduced_robot = self.origin_robot.buildReducedRobot(
                self.limJntLabel,
                self.jntRef
            )
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            if LimMsg.ifDispJntMsg:
                print("限位后机器人关节信息：")
                printJointInfo(self.reduced_robot.model)
        else:
            #使用原始的模型
            self.reduced_robot = self.origin_robot
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            if LimMsg.ifDispJntMsg:
                print("未使用关节限位")

        #依据新的模型更新优化问题
        #刷新casadi逆解求解器模型
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        #获取末端ID并定义误差函数
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,3]
                )
            ]
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,:3].T)
                )
            ]
        )

        #优化问题更新：
        self.opti = casadi.Opti()       #更新优化问题
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)    #定义优化变量：机器人关节配置

        #定义参数(固定输入)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)      #平滑项辅助参数 -- 上一轮关节的位置
        self.var_v_last = self.opti.parameter(self.reduced_robot.model.nq)      # 上一次的关节速度

        self.param_tf_l = self.opti.parameter(4, 4)     #左臂目标位置(齐次变换矩阵)
        self.param_tf_r = self.opti.parameter(4, 4)     #右臂目标位置(齐次变换矩阵)

        #定义目标函数(成本函数)：
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #位移误差成本
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #旋转误差成本
        self.regularization_cost = casadi.sumsqr(self.var_q)            #正则化成本(避免关节角度过大)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)      #平滑成本(最小化当前关节配置与上一时刻配置差值，使运动平滑)
        # 添加速度平滑成本
        self.vel_smooth_cost = casadi.sumsqr(
            (self.var_q - self.var_q_last) / self.tempTime - self.var_v_last
        )

        #优化问题约束(关节位置限制)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit
            )
        )

        #优化目标函数(成本函数的加权组合)
        self.opti.minimize(
            self.optiWeight[0] * self.translational_cost 
            + self.optiWeight[1] * self.rotation_cost 
            + self.optiWeight[2] * self.regularization_cost 
            + self.optiWeight[3] * self.smooth_cost
            + self.optiWeight[4] * self.vel_smooth_cost
        )

        #求解器配置
        self.opti.solver("ipopt", self.opts)   

        # 重新初始化速度和上一次位置数据
        self.init_vel = np.zeros(len(self.init_data))  # 重新初始化速度
        self.last_init_data = self.init_data.copy()    # 重新初始化上一次位置

        #更新辅助变量
        self.ifLstUseLim = LimMsg.ifUseLim
        self.LastLimJntLabel = self.limJntLabel
        self.LastLimJntID = self.limJntID
        self.LastLimJntAng = LimMsg.LimJntAngList

        #更新滤波器配置
        self.smooth_filter = WeightedMovingFilter(np.array(self.FtrWeight), self.reduced_robot_DOF)  # 加权移动平均滤波器

#===========================================================================================================================
#数据分析处理函数

    """
        正解目标位姿与结果输出函数
        其作用为对比求解得到的关节空间位姿在笛卡尔空间映射，以及目标笛卡尔空间位姿
    ------------------------------
    输入量：
        FramePosDes -- 目标位姿（笛卡尔空间）
        JntArrayRes -- 关节求解结果

    """
    def IKresExport(self, FramePosDes_l, FramePosDes_r, JntArrayRes, csv_file):

        JntArrayRes_mapped_l, JntArrayRes_mapped_r = self.solveFk(JntArrayRes)

        x_des_l, y_des_l, z_des_l, row_des_l, pitch_des_l, yaw_des_l = pose_matrix_to_euler(FramePosDes_l)
        x_des_r, y_des_r, z_des_r, row_des_r, pitch_des_r, yaw_des_r = pose_matrix_to_euler(FramePosDes_r)
        
        x_get_l = JntArrayRes_mapped_l.translation[0]
        y_get_l = JntArrayRes_mapped_l.translation[1]
        z_get_l = JntArrayRes_mapped_l.translation[2]
        euler_angles_l = pin.rpy.matrixToRpy(JntArrayRes_mapped_l.rotation)
        row_get_l, pitch_get_l, yaw_get_l = euler_angles_l[0], euler_angles_l[1], euler_angles_l[2]

        x_get_r = JntArrayRes_mapped_r.translation[0]
        y_get_r = JntArrayRes_mapped_r.translation[1]
        z_get_r = JntArrayRes_mapped_r.translation[2]
        euler_angles_r = pin.rpy.matrixToRpy(JntArrayRes_mapped_r.rotation)
        row_get_r, pitch_get_r, yaw_get_r = euler_angles_r[0], euler_angles_r[1], euler_angles_r[2]

        x_err_l = x_des_l - x_get_l
        y_err_l = y_des_l - y_get_l
        z_err_l = z_des_l - z_get_l
        row_err_l = row_des_l - row_get_l
        pitch_err_l = pitch_des_l - pitch_get_l
        yaw_err_l = yaw_des_l - yaw_get_l

        x_err_r = x_des_r - x_get_r
        y_err_r = y_des_r - y_get_r
        z_err_r = z_des_r - z_get_r
        row_err_r = row_des_r - row_get_r
        pitch_err_r = pitch_des_r - pitch_get_r
        yaw_err_r = yaw_des_r - yaw_get_r

        # 准备数据行
        data_row = [
            x_des_l, y_des_l, z_des_l, row_des_l, pitch_des_l, yaw_des_l,  # 期望值
            x_get_l, y_get_l, z_get_l, row_get_l, pitch_get_l, yaw_get_l,  # 实际值
            x_err_l, y_err_l, z_err_l, row_err_l, pitch_err_l, yaw_err_l,  # 误差
            x_des_r, y_des_r, z_des_r, row_des_r, pitch_des_r, yaw_des_r,  # 期望值
            x_get_r, y_get_r, z_get_r, row_get_r, pitch_get_r, yaw_get_r,  # 实际值
            x_err_r, y_err_r, z_err_r, row_err_r, pitch_err_r, yaw_err_r   # 误差
        ]
        
        # 检查文件是否存在
        file_exists = os.path.isfile(csv_file)
    
        # 打开文件并写入数据
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            # 如果文件不存在，先写入表头
            if not file_exists:
                headers = [
                    'x_des_l', 'y_des_l', 'z_des_l', 'row_des_l', 'pitch_des_l', 'yaw_des_l',
                    'x_get_l', 'y_get_l', 'z_get_l', 'row_get_l', 'pitch_get_l', 'yaw_get_l',
                    'x_err_l', 'y_err_l', 'z_err_l', 'row_err_l', 'pitch_err_l', 'yaw_err_l',
                    'x_des_r', 'y_des_r', 'z_des_r', 'row_des_r', 'pitch_des_r', 'yaw_des_r',
                    'x_get_r', 'y_get_r', 'z_get_r', 'row_get_r', 'pitch_get_r', 'yaw_get_r',
                    'x_err_r', 'y_err_r', 'z_err_r', 'row_err_r', 'pitch_err_r', 'yaw_err_r'
                ]
                writer.writerow(headers)
            
            # 写入数据行
            writer.writerow(data_row)

    

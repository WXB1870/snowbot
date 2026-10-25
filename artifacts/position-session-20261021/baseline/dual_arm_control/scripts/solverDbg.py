from ik_solver import DualArmIK as IK
from ik_solver import RobotModelDataWrap as robotWrap
from ik_solver import RobotModelLim as LimMsg
from ik_solver import OptiInte as optiInte
from ik_solver import ViewMsg as viewMsg

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, "/opt/openrobots/lib/python3.12/site-packages")
import pinocchio as pin

def euler_to_pose_matrix(x, y, z, roll, pitch, yaw):
    """
    根据 (x, y, z, roll, pitch, yaw) 生成 4×4 位姿矩阵（齐次变换矩阵）
    
    参数:
        x, y, z: 位置坐标（单位：米或任意长度单位）
        roll (r): 绕 X 轴的旋转（弧度）
        pitch (p): 绕 Y 轴的旋转（弧度）
        yaw (y): 绕 Z 轴的旋转（弧度）
    
    返回:
        pose_matrix: 4×4 齐次变换矩阵（numpy.ndarray）
    """
    # 计算旋转矩阵（R = Rz(yaw) * Ry(pitch) * Rx(roll)）
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # 组合旋转矩阵（Z-Y-X 顺序）
    R = R_z @ R_y @ R_x  # 矩阵乘法
    
    # 构造齐次变换矩阵
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = R  # 旋转部分
    pose_matrix[:3, 3] = [x, y, z]  # 平移部分
    
    return pose_matrix

class IK_solver_debug:

    def printJointInfo(self, model):
        """打印模型关节信息"""
        for name, joint in zip(model.names, model.joints):
            print(f"Joint: {name}, ID: {joint.id}")

    def Init(self):
        self.ifUseDispMsg = True
        self.ifUseSimp = False

        #初始值设置
        urdf_path = "/home/gt/lxf/snowbot_ws/src/dual_arm/urdf/snowbot.urdf"
        end_label = ["arm_l_link_8", "arm_r_link_8"]
        #end_label = ["l_joint_7", "r_joint_7"]
        limJntID = ["arm_l_joint_3", "arm_r_joint_3"]
        
        
        JntStPos = [0, 0, -0.1, 0, 0, 0, 0,
                    0, 0, -0.1, 0, 0, 0, 0]
        self.jntStPos = JntStPos

        m_optiInte = optiInte()
        m_viewMsg = viewMsg(True, True)

        m_robotModel = robotWrap(urdf_path, end_label)
        m_limitMsg = LimMsg(self.ifUseSimp, limJntID, True)

        self.IKsolver = IK(m_robotModel, JntStPos, m_limitMsg, m_optiInte, m_viewMsg)
        self.singleArmDof = 7

        print(f"初始化完毕")

    def singleDot_IKDbg(self):
        #debug函数1——求解单个目标点逆解
        x_l = 0.3
        y_l = 0.204998
        z_l = -0.16
        r_l = 0.0
        p_l = -0.8
        yal_l = 1.570796

        x_r = 0.3
        y_r = -0.204998
        z_r = -0.16
        r_r = 0.0
        p_r = -0.8
        yal_r = -1.570796

        lArm_endPos = euler_to_pose_matrix(x_l, y_l, z_l, r_l, p_l, yal_l)
        rArm_endPos = euler_to_pose_matrix(x_r, y_r, z_r, r_r, p_r, yal_r)

        np.set_printoptions(
            precision=5,       # 保留5位小数
            suppress=True,     # 禁用科学计数法
            floatmode='fixed'  # 固定小数格式
        )
        print(f"左右臂的目标位姿打印")
        print("左臂位姿矩阵:")
        print(lArm_endPos)
        print("\n右臂位姿矩阵:")
        print(rArm_endPos)

        q_arm = self.IKsolver.solve_ik(lArm_endPos, rArm_endPos)

        lArm_endPosRet, rArm_endPosRet = self.IKsolver.solveFk(q_arm)
        lArm_endVec = self.IKsolver.transSE3_to_vec(lArm_endPosRet)
        rArm_endVec = self.IKsolver.transSE3_to_vec(rArm_endPosRet)
        print(lArm_endVec)
        print(rArm_endVec)

        lArm_endPosTrans = self.IKsolver.desPos_baseFrameTrans_np(lArm_endPos)
        rArm_endPosTrans = self.IKsolver.desPos_baseFrameTrans_np(rArm_endPos)
        lArm_endDesVecTrans = self.IKsolver.transNpMat_to_vec(lArm_endPosTrans)
        rArm_endDesVecTrans = self.IKsolver.transNpMat_to_vec(rArm_endPosTrans)
        print(lArm_endDesVecTrans)
        print(rArm_endDesVecTrans)

        temp = 0
        if self.ifUseDispMsg:
            for ele in q_arm:
                print(f"关节ID = {temp + 1}， 关节角度：{q_arm[temp]}")
                temp = temp + 1

    def dotTraj_IKDbg(self):
        #debug函数2——求解点迹逆解

        #过程控制参量
        trajLen = 1000      #点迹长度

        #始末位置定义
        x_l_begin, y_l_begin, z_l_begin = 0, 0.184996, -0.539000
        r_l_begin, p_l_begin, yaw_l_begin = 3.141593, 0.000015, 1.570793

        x_r_begin, y_r_begin, z_r_begin = 0.000009, -0.184996, -0.539000
        r_r_begin, p_r_begin, yaw_r_begin = -3.141569, 0.000015, -1.570823

        x_l_end, y_l_end, z_l_end = 0.3, 0.2, -0.16
        r_l_end, p_l_end, yaw_l_end = 0, -0.8, 1.57

        x_r_end, y_r_end, z_r_end = 0.3, -0.2, -0.16
        r_r_end, p_r_end, yaw_r_end = 0, -0.8, -1.57

        #点迹增量定义
        x_l_der, y_l_der, z_l_der = (x_l_end-x_l_begin)/trajLen, (y_l_end-y_l_begin)/trajLen, (z_l_end-z_l_begin)/trajLen
        r_l_der, p_l_der, yaw_l_end = (r_l_end-r_l_begin)/trajLen, (p_l_end-p_l_begin)/trajLen, (yaw_l_end-yaw_l_begin)/trajLen

        x_r_der, y_r_der, z_r_der = (x_r_end-x_r_begin)/trajLen, (y_r_end-y_r_begin)/trajLen, (z_r_end-z_r_begin)/trajLen
        r_r_der, p_r_der, yaw_r_end = (r_r_end-r_r_begin)/trajLen, (p_r_end-p_r_begin)/trajLen, (yaw_r_end-yaw_r_begin)/trajLen

        #定义空的返回值数组，用于接受所有关节的返回值
        all_arm_dof = 2 * self.singleArmDof
        
        #定义迭代量以及结果储存数组
        temp_lastJntArr = self.jntStPos
        allJntArrRes = np.empty([all_arm_dof, trajLen])

        for i in range(trajLen):
            #获取本轮目标位置
            x_l_temp, y_l_temp, z_l_temp = x_l_begin + (i+1)*x_l_der, y_l_begin + (i+1)*y_l_der, z_l_begin + (i+1)*z_l_der
            r_l_temp, p_l_temp, yaw_l_temp = r_l_begin + (i+1)*r_l_der, p_l_begin + (i+1)*p_l_der, yaw_l_begin + (i+1)*y_l_der

            x_r_temp, y_r_temp, z_r_temp = x_r_begin + (i+1)*x_r_der, y_r_begin + (i+1)*y_r_der, z_r_begin + (i+1)*z_r_der
            r_r_temp, p_r_temp, yaw_r_temp = r_r_begin + (i+1)*r_r_der, p_r_begin + (i+1)*p_r_der, yaw_r_begin + (i+1)*y_r_der

            lArm_endPos_temp = euler_to_pose_matrix(x_l_temp, y_l_temp, z_l_temp, r_l_temp, p_l_temp, yaw_l_temp)
            rArm_endPos_temp = euler_to_pose_matrix(x_r_temp, y_r_temp, z_r_temp, r_r_temp, p_r_temp, yaw_r_temp)

            temp_JntArr = self.IKsolver.solve_ik(lArm_endPos_temp, rArm_endPos_temp, temp_lastJntArr)

            allJntArrRes[:, i] = temp_JntArr    # 将 temp_JntArr 赋值给 allJntArrRes 的第 i 列
            temp_lastJntArr = temp_JntArr

        # ===== 新增交互式绘图代码 =====
        while True:
            # 打印关节ID信息，方便用户选择
            print("\n当前关节ID范围：0 ~ {}".format(all_arm_dof - 1))
            print("左臂关节ID：0 ~ {}".format(self.singleArmDof - 1))
            print("右臂关节ID：{} ~ {}".format(self.singleArmDof, all_arm_dof - 1))
    
            # 用户输入要绘制的关节ID
            joint_id = input("请输入要绘制的关节ID（输入 q 退出）：")
            if joint_id.lower() == 'q':
                break
    
            try:
                joint_id = int(joint_id)
                if joint_id < 0 or joint_id >= all_arm_dof:
                    print("错误：关节ID超出范围！")
                    continue
            except ValueError:
                print("错误：请输入有效的整数！")
                continue
    
            # 绘制关节轨迹
            plt.figure()
            plt.plot(allJntArrRes[joint_id, :], label=f'Joint {joint_id}')
            plt.xlabel('Trajectory Step')
            plt.ylabel('Joint Angle (rad)')
            plt.title(f'Joint Trajectory (ID: {joint_id})')
            plt.legend()
            plt.grid()
            plt.show()
    
            # 询问是否继续
            choice = input("是否继续绘制其他关节？(y/n): ").lower()
            if choice != 'y':
                break

    def NullSpace_ctlDbg(self):
        #debug函数3——零空间控制debug函数

        #定义受限关节角度变化范围
        limJntAng_st_l = -0.1
        limJntAng_end_l = 0.8
        limJntAng_st_r = 0.1
        limJntAng_end_r = -0.8
        
        #受限关节信息定义：
        limJntList = ["l_joint_3", "r_joint_3"]
        limJntAng = [limJntAng_st_l, limJntAng_st_r]
        if_disp = False
        limMsgWrap = LimMsg(True, limJntList, if_disp, limJntAng)

        #进行一轮逆解：
        x_l = 0.3
        y_l = 0.204998
        z_l = -0.16
        r_l = 0.0
        p_l = -0.8
        yal_l = 1.570796

        x_r = 0.3
        y_r = -0.204998
        z_r = -0.16
        r_r = 0.0
        p_r = -0.8
        yal_r = -1.570796

        lArm_endPos = euler_to_pose_matrix(x_l, y_l, z_l, r_l, p_l, yal_l)
        rArm_endPos = euler_to_pose_matrix(x_r, y_r, z_r, r_r, p_r, yal_r)

        q_arm = self.IKsolver.solve_ik(lArm_endPos, rArm_endPos)

        total_tempTimes: int = 20
        ifforward: bool = True

        trajLen:int = 0
        trajTolLen:int = 80

        #while True:

        ifUseDataGet = True
        
        while trajLen <= trajTolLen:
            trajLen = trajLen + 1
            for i in range(total_tempTimes):
                if ifforward:
                    limJntAng_temp_l = (limJntAng_end_l - limJntAng_st_l)/total_tempTimes*(i+1) + limJntAng_st_l
                    limJntAng_temp_r = (limJntAng_end_r - limJntAng_st_r)/total_tempTimes*(i+1) + limJntAng_st_r
                    m_limJntAng = [limJntAng_temp_l, limJntAng_temp_r]
                    limMsgWrap = LimMsg(True, limJntList, if_disp, m_limJntAng)
                    self.IKsolver.JntLimReDef(limMsgWrap)
                    q_arm = self.IKsolver.solve_ik(lArm_endPos, rArm_endPos)
                    
                    # 定义CSV文件路径（可以根据需要修改路径）
                    if ifUseDataGet:
                        csv_file_l = '/home/gt/lxf/catkin_ws/src/dual_arm_control/scripts/dataAnaRes/ik_results_ana.csv'
                        self.IKsolver.IKresExport(lArm_endPos, rArm_endPos, q_arm, csv_file_l)

                    if i+1 == total_tempTimes:
                        ifforward = False
                else:
                    limJntAng_temp_l = (limJntAng_st_l - limJntAng_end_l)/total_tempTimes*(i+1) + limJntAng_end_l
                    limJntAng_temp_r = (limJntAng_st_r - limJntAng_end_r)/total_tempTimes*(i+1) + limJntAng_end_r
                    m_limJntAng = [limJntAng_temp_l, limJntAng_temp_r]
                    limMsgWrap = LimMsg(True, limJntList, if_disp, m_limJntAng)
                    self.IKsolver.JntLimReDef(limMsgWrap)
                    q_arm = self.IKsolver.solve_ik(lArm_endPos, rArm_endPos)

                    # 定义CSV文件路径（可以根据需要修改路径）
                    if ifUseDataGet:
                        csv_file_l = '/home/gt/lxf/catkin_ws/src/dual_arm_control/scripts/dataAnaRes/ik_results_ana.csv'
                        self.IKsolver.IKresExport(lArm_endPos, rArm_endPos, q_arm, csv_file_l)

                    if i+1 == total_tempTimes:
                        ifforward = True   

    def Fk_debug(self):

        #定义初始值
        JntPosInp = np.array([
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ])
        
        #调试1：正解并打印：solveFk
        left_ee_pos, right_ee_pos = self.IKsolver.solveFk(JntPosInp)

        # 提取左臂位姿信息
        left_translation = left_ee_pos.translation  # 位置 [x, y, z]
        left_rotation = left_ee_pos.rotation       # 旋转矩阵 (3x3)
        left_rpy = pin.rpy.matrixToRpy(left_rotation)  # 转换为 roll, pitch, yaw

        # 提取右臂位姿信息
        right_translation = right_ee_pos.translation  # 位置 [x, y, z]
        right_rotation = right_ee_pos.rotation       # 旋转矩阵 (3x3)
        right_rpy = pin.rpy.matrixToRpy(right_rotation)  # 转换为 roll, pitch, yaw

        # 打印结果
        print("Left Arm End-Effector Pose:")
        print(f"Position (x, y, z): [{left_translation[0]:.8f}, {left_translation[1]:.8f}, {left_translation[2]:.8f}]")
        #print(f"Orientation (roll, pitch, yaw): [{np.degrees(left_rpy[0]):.4f}°, {np.degrees(left_rpy[1]):.4f}°, {np.degrees(left_rpy[2]):.4f}°]")
        print(f"Orientation (roll, pitch, yaw): [{left_rpy[0]:.8f}, {left_rpy[1]:.8f}, {left_rpy[2]:.8f}]")


        print("\nRight Arm End-Effector Pose:")
        print(f"Position (x, y, z): [{right_translation[0]:.8f}, {right_translation[1]:.8f}, {right_translation[2]:.8f}]")
        #print(f"Orientation (roll, pitch, yaw): [{np.degrees(right_rpy[0]):.4f}°, {np.degrees(right_rpy[1]):.4f}°, {np.degrees(right_rpy[2]):.4f}°]")
        print(f"Orientation (roll, pitch, yaw): [{right_rpy[0]:.8f}, {right_rpy[1]:.8f}, {right_rpy[2]:.8f}]")

        print(f"origin lArm pinocchio Frame:")
        print(left_ee_pos)
        print(f"origin rArm pinocchio Frame:")
        print(right_ee_pos)

        #调试2：正解并打印：solveFk_ListRet
        l_ee_np, r_ee_np = self.IKsolver.solveFk_ListRet(JntPosInp)
        print(f"lArm pinocchio Frame:")
        print(l_ee_np)
        print(f"rArm pinocchio Frame:")
        print(r_ee_np)
        # 打印 l_ee_np 和 r_ee_np 的尺寸（假设它们是 list）
        print("\nDebug solveFk_ListRet:")
        print(f"lArm Frame (type: {type(l_ee_np)}), size: {len(l_ee_np)}")
        print(f"rArm Frame (type: {type(r_ee_np)}), size: {len(r_ee_np)}")
        
        




if __name__ == "__main__":
    print(f"开始debug")
    solver_dbg = IK_solver_debug()
    solver_dbg.Init()
    #solver_dbg.singleDot_IKDbg()
    #solver_dbg.dotTraj_IKDbg()
    #solver_dbg.NullSpace_ctlDbg()
    solver_dbg.Fk_debug()
        
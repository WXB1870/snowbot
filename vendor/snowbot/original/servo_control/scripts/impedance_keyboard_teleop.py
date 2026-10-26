#!/usr/bin/env python3
"""键盘 teleop: 实时调整阻抗参考位姿, 测试末端跟随性能.

启动后自动订阅阻抗控制器 status 话题, 以机械臂当前末端位姿作为
初始参考 (避免使能瞬间参考跳变导致猛冲). 之后每次按键在当前参考
基础上增量调整.

按键 (在终端窗口中按下回车后生效):
  W/S:    z+ / z-           (上下, 0.01m/次)
  A/D:    x- / x+          (前后, 0.01m/次)
  Q/E:    y- / y+          (左右, 0.01m/次)
  I/K:    pitch+ / pitch-  (俯仰, 0.02rad/次)
  J/L:    yaw+ / yaw-      (偏航, 0.02rad/次)
  U/O:    roll+ / roll-    (横滚, 0.02rad/次)
  R:      重置到 status 初始位姿
  Space:  暂停/继续发送 (保持当前参考)
  Ctrl+C: 退出 (自动禁用阻抗)

用法:
  ros2 run snowbot_servo_control impedance_keyboard_teleop --side left

可选参数:
  --side left|right|both
  --step-xyz 0.01      平移步长 (米)
  --step-rpy 0.02      旋转步长 (弧度)
  --rate 100           参考发布频率 (Hz)
  --ik                 启用 IK 求解, 将关节角填入 q_d_nullspace
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Accel, Quaternion, Point
from snowbot_ctl_msgs.msg import CartesianImpedance
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as Rot

try:
    import pinocchio as pin
    _HAS_PINOCCHIO = True
except ImportError as _e:
    _HAS_PINOCCHIO = False
    _PINOCCHIO_ERR = str(_e)

# URDF 路径 (与 C++ 控制器使用同一份)
_URDF_PATH = '/home/cc/snowbot_ws/src/dual_arm/urdf/snowbot.urdf'

# 与控制器 status 发布端一致: BEST_EFFORT + VOLATILE
_QOS_LOWLAT = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)


class ArmIK:
    """基于 pinocchio 的 7-DoF 机械臂逆运动学求解器.

    使用 Levenberg-Marquardt 阻尼最小二乘法求解 IK,
    以当前关节角为初值, 迭代收敛到期望末端位姿.

    注意: 目标位姿是 TCP (夹爪中心) 位姿, 内部会根据 tcp_to_ee
    变换转换为 ee_link 目标位姿后求解.
    """

    def __init__(self, urdf_path, ee_link_name, tcp_to_ee=None, joint_indices=None):
        """初始化 pinocchio 模型.

        Args:
            urdf_path: URDF 文件路径
            ee_link_name: 末端 link 名 (如 'arm_r_link_7')
            tcp_to_ee: TCP 相对 ee_link 的位姿 [x,y,z,qx,qy,qz,qw]
                       (与 impedance_params.yaml 中一致)
            joint_indices: 该臂 7 个关节在完整模型中的索引
                           (如不指定, 自动从关节名匹配)
        """
        if not _HAS_PINOCCHIO:
            raise RuntimeError("pinocchio 未安装, 无法使用 IK 功能")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF 不存在: {urdf_path}")

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId(ee_link_name)

        # TCP 相对 ee_link 的变换 (4x4 齐次矩阵)
        # 用于把 TCP 目标位姿转换为 ee_link 目标位姿
        if tcp_to_ee is not None:
            t = np.array(tcp_to_ee[:3], dtype=float)
            q = np.array(tcp_to_ee[3:7], dtype=float)  # [x,y,z,w]
            self.T_ee_to_tcp = np.eye(4)
            # scipy quaternion 是 [x,y,z,w]
            R = Rot.from_quat(q).as_matrix()
            self.T_ee_to_tcp[:3, :3] = R
            self.T_ee_to_tcp[:3, 3] = t
            # TCP 到 ee_link 的变换 (逆)
            self.T_tcp_to_ee = np.linalg.inv(self.T_ee_to_tcp)
        else:
            self.T_ee_to_tcp = np.eye(4)
            self.T_tcp_to_ee = np.eye(4)

        # 提取该臂 7 个关节的索引 (URDF 中可能有其他关节, 如夹爪)
        if joint_indices is not None:
            self.arm_joint_idx = [int(x) for x in joint_indices]
        else:
            # 自动匹配: 找出 ee_link_name 对应的 arm_x_joint_1..7
            arm_prefix = ee_link_name.replace('_link_7', '_joint_')
            self.arm_joint_idx = []
            for i in range(1, 8):
                jname = f'{arm_prefix}{i}'
                jid = self.model.getJointId(jname)
                if jid < self.model.njoints:
                    self.arm_joint_idx.append(int(jid))

        # nq 是完整模型维度, 我们只优化 arm 的 7 个关节
        self.nq = self.model.nq
        # 生成全维度 q 向量中属于本臂的索引 (在 q 向量中的位置)
        # pinocchio 的 q 向量按 joint 顺序排列, 用 idx_qs 数组获取每个 joint 的 q 索引
        # idx_qs[jid] 是该 joint 在 q 中的起始索引
        idx_qs = self.model.idx_qs
        self.q_idx_in_full = []
        for jid in self.arm_joint_idx:
            idx = int(idx_qs[jid])
            # 每个 revolute joint nq=1, 但保险起见用 idx_qs[jid+1]-idx 计算长度
            next_idx = int(idx_qs[jid + 1]) if jid + 1 < len(idx_qs) else self.nq
            self.q_idx_in_full.extend(range(idx, next_idx))
        self.q_idx_in_full = np.array(self.q_idx_in_full, dtype=int)

    def fk(self, q_full):
        """前向运动学, 返回末端位姿 (位置, 旋转矩阵)."""
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        T = self.data.oMf[self.ee_frame_id]
        return T.translation.copy(), T.rotation.copy()

    def compute_jacobian(self, q_full):
        """计算末端 frame 的局部雅可比 (6 x nq), 然后提取本臂 7 列."""
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.framesForwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        Ji = pin.getFrameJacobian(self.model, self.data,
                                  self.ee_frame_id, pin.ReferenceFrame.LOCAL)
        # 提取本臂 7 个关节对应的列
        return Ji[:, self.q_idx_in_full]

    def solve(self, target_pos, target_rot, q_init_full, max_iters=50, tol=1e-4):
        """求解 IK, 返回完整 q 向量 (仅本臂 7 关节被更新).

        Args:
            target_pos: 期望末端位置 (3,)
            target_rot: 期望末端旋转矩阵 (3, 3)
            q_init_full: 初始完整关节角 (nq,), 本臂部分作为 IK 初值
            max_iters: 最大迭代次数
            tol: 收敛阈值 (位置+姿态误差)

        Returns:
            q_full: 解算后的完整关节角向量
            success: 是否收敛
            err: 最终误差范数
        """
        q = q_init_full.copy().astype(float)
        norm_err = float('inf')

        for _ in range(max_iters):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
            T = self.data.oMf[self.ee_frame_id]
            cur_pos = T.translation
            cur_rot = T.rotation

            # 位置误差
            err_p = (target_pos - cur_pos).reshape(3, 1)
            # 姿态误差 (使用旋转矩阵的对数映射)
            dR = target_rot @ cur_rot.T
            # 从旋转矩阵提取旋转向量 (使用 scipy)
            r = Rot.from_matrix(dR).as_rotvec()
            err_r = r.reshape(3, 1)
            err = np.vstack([err_p, err_r])

            norm_err = np.linalg.norm(err)
            if norm_err < tol:
                return q, True, norm_err

            # 提取本臂 7 列雅可比 (在 LOCAL frame 下, 与 err_p/err_r 一致)
            Ji = self.compute_jacobian(q)
            # 阻尼最小二乘: dq = Ji^T (Ji Ji^T + λI)^{-1} err
            lam = 1e-4
            JJT = Ji @ Ji.T + lam * np.eye(6)
            dq_arm = Ji.T @ np.linalg.solve(JJT, err)

            # 步长限制 (防止大步长发散)
            max_step = 0.3  # rad
            n_dq = np.linalg.norm(dq_arm)
            if n_dq > max_step:
                dq_arm *= max_step / n_dq

            # 只更新本臂 7 个关节
            q[self.q_idx_in_full] += dq_arm.flatten()

        # 达到最大迭代, 未收敛
        return q, False, norm_err

    def solve_from_current(self, target_pos, target_rot, q_current):
        """从当前关节角开始 IK, 返回 (arm_q_7, success, err).

        Args:
            target_pos: 期望末端位置 (3,)
            target_rot: 期望末端旋转矩阵 (3, 3)
            q_current: 完整模型当前关节角 (nq,)

        Returns:
            arm_q: 本臂 7 关节角
            success: 是否收敛
            err: 最终误差
        """
        q_full, success, err = self.solve(target_pos, target_rot, q_current)
        arm_q = q_full[self.q_idx_in_full].copy()
        return arm_q, success, err


class KeyboardTeleop(Node):
    def __init__(self, args):
        super().__init__('impedance_keyboard_teleop')
        self.args = args
        self.side = args.side
        self.use_ik = args.ik

        self.enable_pub = self.create_publisher(
            Bool, '/servo_node/impedance_enable', 10)
        if self.side in ('left', 'both'):
            self.left_ref_pub = self.create_publisher(
                CartesianImpedance, '/servo_node/impedance/arm_l_link_7/reference', 1)
            self.left_status_sub = self.create_subscription(
                Float64MultiArray,
                '/servo_node/impedance/arm_l_link_7/status',
                lambda msg: self._status_cb('left', msg), _QOS_LOWLAT)
        if self.side in ('right', 'both'):
            self.right_ref_pub = self.create_publisher(
                CartesianImpedance, '/servo_node/impedance/arm_r_link_7/reference', 1)
            self.right_status_sub = self.create_subscription(
                Float64MultiArray,
                '/servo_node/impedance/arm_r_link_7/status',
                lambda msg: self._status_cb('right', msg), _QOS_LOWLAT)

        # IK 求解器 (pinocchio)
        self.left_ik = None
        self.right_ik = None
        if self.use_ik:
            if not _HAS_PINOCCHIO:
                print(f'[WARN] pinocchio 未安装 ({_PINOCCHIO_ERR}), IK 功能禁用')
                self.use_ik = False
            else:
                # tcp_to_ee 参数 (与 impedance_params.yaml 一致)
                tcp_to_ee = [0.192, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                if self.side in ('left', 'both'):
                    self.left_ik = ArmIK(_URDF_PATH, 'arm_l_link_7', tcp_to_ee)
                if self.side in ('right', 'both'):
                    self.right_ik = ArmIK(_URDF_PATH, 'arm_r_link_7', tcp_to_ee)

        # 当前参考位姿 (左右臂独立维护, 初始 None 等待 status 填充)
        self.left_xyz = None
        self.left_rpy = None
        self.right_xyz = None
        self.right_rpy = None
        # status 首帧捕获的初始位姿 (供 R 键重置)
        self.init_left_xyz = None
        self.init_left_rpy = None
        self.init_right_xyz = None
        self.init_right_rpy = None
        # 当前关节角 (从 status 读取, 作为 IK 初值)
        self.left_q = None      # 7 维, 本臂关节角
        self.right_q = None
        # pinocchio 完整模型维度 (nq, 包含其他关节如夹爪)
        self.left_nq_full = None
        self.right_nq_full = None

        # 首帧稳定性检测: 连续 STABLE_FRAMES 帧位姿变化小于阈值才采用
        # 防止 servo_node 启动瞬间机械臂未归零导致首帧读到非零位姿态
        self._stable_left = []   # 缓存最近几帧的 [xyz, rpy]
        self._stable_right = []
        self._stable_threshold = 1e-3  # 位置/角度 1mm / 1mrad
        self._STABLE_FRAMES = 5

        self.paused = False
        self.dirty = False  # status 到达前不发参考, 让控制器 hold 当前位姿
        self.running = True

        # 周期发布
        self.timer = self.create_timer(1.0 / args.rate, self._publish)
        self._print_help()

    def _print_help(self):
        print('=' * 60)
        print('阻抗键盘 teleop. 按键 (回车后生效):')
        print('  W/S    z-/z+       A/D    x-/x+       Q/E    y-/y+')
        print('  I/K    pitch-/pitch+ J/L   yaw-/yaw+   U/O    roll-/roll+')
        print('  R      重置       Space 暂停/继续     Ctrl+C 退出')
        print('=' * 60)
        print('等待 status 话题获取当前末端位姿...')

    def _status_cb(self, arm, msg):
        """从 status 数组解析当前末端位姿 (位置 + RPY) 和关节角.

        首帧采用稳定性检测: 连续 STABLE_FRAMES 帧位姿变化 < 阈值才
        认为机械臂已稳定, 防止 servo_node 启动瞬间关节未归零导致
        首帧读到非零位姿态 (例如 pitch 40° 的错误值).
        """
        data = msg.data
        if len(data) < 48:
            return
        # 新布局: [36..38] p_tcp_in_base_, [39..41] actual_rpy (xyz 约定)
        xyz = np.array([data[36], data[37], data[38]], dtype=float)
        rpy = np.array([data[39], data[40], data[41]], dtype=float)
        pose = np.concatenate([xyz, rpy])
        # 关节角: [48..54] robot_q_ (n_dof_=7)
        q_arm = np.array(data[48:55], dtype=float) if len(data) >= 55 else None

        if arm == 'left':
            # 持续更新当前关节角 (作为 IK 初值)
            if q_arm is not None:
                self.left_q = q_arm.copy()
            if self.left_xyz is None:
                self._stable_left.append(pose)
                if len(self._stable_left) > self._STABLE_FRAMES:
                    self._stable_left.pop(0)
                if len(self._stable_left) == self._STABLE_FRAMES:
                    # 检查最近 STABLE_FRAMES 帧之间的差异
                    arr = np.array(self._stable_left)
                    delta = np.max(np.abs(arr - arr[-1]))
                    if delta < self._stable_threshold:
                        self.left_xyz = xyz.copy()
                        self.left_rpy = rpy.copy()
                        self.init_left_xyz = xyz.copy()
                        self.init_left_rpy = rpy.copy()
                        self._check_ready()
        else:
            if q_arm is not None:
                self.right_q = q_arm.copy()
            if self.right_xyz is None:
                self._stable_right.append(pose)
                if len(self._stable_right) > self._STABLE_FRAMES:
                    self._stable_right.pop(0)
                if len(self._stable_right) == self._STABLE_FRAMES:
                    arr = np.array(self._stable_right)
                    delta = np.max(np.abs(arr - arr[-1]))
                    if delta < self._stable_threshold:
                        self.right_xyz = xyz.copy()
                        self.right_rpy = rpy.copy()
                        self.init_right_xyz = xyz.copy()
                        self.init_right_rpy = rpy.copy()
                        self._check_ready()

    def _check_ready(self):
        """检查所有激活臂的初始位姿是否已从 status 获取."""
        ready = True
        if self.side in ('left', 'both') and self.left_xyz is None:
            ready = False
        if self.side in ('right', 'both') and self.right_xyz is None:
            ready = False
        if ready:
            # 首次就绪: 开始发布参考 (与当前位姿一致, 不会跳变)
            self.dirty = True
            parts = []
            if self.side in ('left', 'both'):
                parts.append(f'L xyz={self.left_xyz} rpy_deg={np.rad2deg(self.left_rpy)}')
            if self.side in ('right', 'both'):
                parts.append(f'R xyz={self.right_xyz} rpy_deg={np.rad2deg(self.right_rpy)}')
            print(f'\r已获取初始位姿: ' + '  '.join(parts) + '          ', flush=True)

    def _solve_ik(self, arm, xyz, rpy):
        """求解 IK, 返回 7 维关节角. 失败则返回 None.

        输入 xyz/rpy 是 TCP (夹爪中心) 位姿, 内部转换为 ee_link 目标.
        """
        if not self.use_ik:
            return None
        ik = self.left_ik if arm == 'left' else self.right_ik
        q_current = self.left_q if arm == 'left' else self.right_q
        if ik is None or q_current is None:
            return None

        # 期望 TCP 旋转矩阵 (与 C++ eulerAngles(0,1,2) 一致: R = Rx·Ry·Rz)
        R_tcp_target = Rot.from_euler('xyz', rpy).as_matrix()

        # 把 TCP 目标位姿转换为 ee_link 目标位姿
        # T_base_tcp = T_base_ee * T_ee_tcp
        # => T_base_ee = T_base_tcp * T_ee_tcp^{-1} = T_base_tcp * T_tcp_to_ee
        T_tcp_target_4x4 = np.eye(4)
        T_tcp_target_4x4[:3, :3] = R_tcp_target
        T_tcp_target_4x4[:3, 3] = xyz
        T_ee_target_4x4 = T_tcp_target_4x4 @ ik.T_tcp_to_ee

        target_pos = T_ee_target_4x4[:3, 3]
        target_rot = T_ee_target_4x4[:3, :3]

        # 构建完整 q 向量 (仅本臂 7 关节有值, 其余置零)
        q_full = np.zeros(ik.nq)
        q_full[ik.q_idx_in_full] = q_current

        arm_q, success, err = ik.solve_from_current(target_pos, target_rot, q_full)
        if not success:
            print(f'\n[IK WARN] {arm} 未收敛, err={err:.6f}')
        return arm_q

    def _build_msg(self, arm, xyz, rpy):
        msg = CartesianImpedance()
        msg.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        q = Rot.from_euler('xyz', rpy).as_quat()
        msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.pose_twist = Twist()
        msg.pose_accel = Accel()
        # IK 求解关节角填入 q_d_nullspace
        if self.use_ik:
            ik_q = self._solve_ik(arm, xyz, rpy)
            if ik_q is not None:
                msg.q_d_nullspace = ik_q.tolist()
            else:
                msg.q_d_nullspace = [0.0] * 7
        else:
            msg.q_d_nullspace = [0.0] * 7
        return msg

    def _publish(self):
        if self.paused or not self.dirty:
            return
        # 位姿尚未从 status 初始化时不发参考 (让控制器 hold)
        if self.side in ('left', 'both') and self.left_xyz is not None:
            self.left_ref_pub.publish(self._build_msg('left', self.left_xyz, self.left_rpy))
        if self.side in ('right', 'both') and self.right_xyz is not None:
            self.right_ref_pub.publish(self._build_msg('right', self.right_xyz, self.right_rpy))
        self.dirty = False

    def _print_state(self):
        l_str = (f'xyz=[{self.left_xyz[0]:.3f},{self.left_xyz[1]:.3f},{self.left_xyz[2]:.3f}] '
                  f'rpy_deg=[{np.rad2deg(self.left_rpy[0]):.2f},{np.rad2deg(self.left_rpy[1]):.2f},{np.rad2deg(self.left_rpy[2]):.2f}]') \
                 if (self.side in ('left', 'both') and self.left_xyz is not None) else '---'
        r_str = (f'xyz=[{self.right_xyz[0]:.3f},{self.right_xyz[1]:.3f},{self.right_xyz[2]:.3f}] '
                  f'rpy_deg=[{np.rad2deg(self.right_rpy[0]):.2f},{np.rad2deg(self.right_rpy[1]):.2f},{np.rad2deg(self.right_rpy[2]):.2f}]') \
                 if (self.side in ('right', 'both') and self.right_xyz is not None) else '---'
        ik_str = ''
        if self.use_ik:
            if self.side in ('left', 'both') and self.left_q is not None:
                ik_str += f'  L_q=[{self.left_q[0]:.2f},{self.left_q[1]:.2f},{self.left_q[2]:.2f},{self.left_q[3]:.2f},{self.left_q[4]:.2f},{self.left_q[5]:.2f},{self.left_q[6]:.2f}]'
            if self.side in ('right', 'both') and self.right_q is not None:
                ik_str += f'  R_q=[{self.right_q[0]:.2f},{self.right_q[1]:.2f},{self.right_q[2]:.2f},{self.right_q[3]:.2f},{self.right_q[4]:.2f},{self.right_q[5]:.2f},{self.right_q[6]:.2f}]'
        print(f'\rL: {l_str}  R: {r_str}{ik_str}  '
              f'{"[PAUSED]" if self.paused else ""}', end='', flush=True)

    def handle_key(self, key):
        # 位姿未初始化时忽略按键
        if self.side in ('left', 'both') and self.left_xyz is None:
            return
        if self.side in ('right', 'both') and self.right_xyz is None:
            return

        s_xyz = self.args.step_xyz
        s_rpy = self.args.step_rpy
        d_left_xyz = np.zeros(3)
        d_left_rpy = np.zeros(3)
        d_right_xyz = np.zeros(3)
        d_right_rpy = np.zeros(3)

        if key == 'w':
            d_left_xyz[2] += s_xyz; d_right_xyz[2] += s_xyz
        elif key == 's':
            d_left_xyz[2] -= s_xyz; d_right_xyz[2] -= s_xyz
        elif key == 'a':
            d_left_xyz[0] -= s_xyz; d_right_xyz[0] -= s_xyz
        elif key == 'd':
            d_left_xyz[0] += s_xyz; d_right_xyz[0] += s_xyz
        elif key == 'q':
            d_left_xyz[1] -= s_xyz; d_right_xyz[1] += s_xyz  # 镜像
        elif key == 'e':
            d_left_xyz[1] += s_xyz; d_right_xyz[1] -= s_xyz
        elif key == 'i':
            d_left_rpy[1] += s_rpy; d_right_rpy[1] -= s_rpy
        elif key == 'k':
            d_left_rpy[1] -= s_rpy; d_right_rpy[1] += s_rpy
        elif key == 'j':
            d_left_rpy[2] += s_rpy; d_right_rpy[2] -= s_rpy
        elif key == 'l':
            d_left_rpy[2] -= s_rpy; d_right_rpy[2] += s_rpy
        elif key == 'u':
            d_left_rpy[0] += s_rpy; d_right_rpy[0] -= s_rpy
        elif key == 'o':
            d_left_rpy[0] -= s_rpy; d_right_rpy[0] += s_rpy
        elif key == 'r':
            # 重置到 status 首帧捕获的初始位姿
            if self.side in ('left', 'both') and self.init_left_xyz is not None:
                self.left_xyz = self.init_left_xyz.copy()
                self.left_rpy = self.init_left_rpy.copy()
            if self.side in ('right', 'both') and self.init_right_xyz is not None:
                self.right_xyz = self.init_right_xyz.copy()
                self.right_rpy = self.init_right_rpy.copy()
            self.dirty = True
            print('\n[RESET 到初始位姿]')
            return
        elif key == ' ':
            self.paused = not self.paused
            return
        else:
            return

        if self.side in ('left', 'both') and self.left_xyz is not None:
            self.left_xyz += d_left_xyz
            self.left_rpy += d_left_rpy
        if self.side in ('right', 'both') and self.right_xyz is not None:
            self.right_xyz += d_right_xyz
            self.right_rpy += d_right_rpy
        self.dirty = True


def get_key(settings, timeout=0.1):
    """非阻塞读取单字符."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        ch = sys.stdin.read(1)
        return ch
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--side', choices=['left', 'right', 'both'],
                        default='both')
    parser.add_argument('--step-xyz', type=float, default=0.01)
    parser.add_argument('--step-rpy', type=float, default=0.02)
    parser.add_argument('--rate', type=int, default=100)
    parser.add_argument('--ik', action='store_true', default=False,
                        help='启用 IK 求解, 将关节角填入 q_d_nullspace')
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = KeyboardTeleop(args)

    # 启用阻抗
    msg = Bool()
    msg.data = True
    node.enable_pub.publish(msg)
    print('已发送 impedance_enable = True')
    time.sleep(0.2)

    # 终端设为 cbreak 模式
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(settings)
            if key:
                if key == '\x03':  # Ctrl+C
                    break
                node.handle_key(key)
                node._print_state()
    except KeyboardInterrupt:
        pass
    finally:
        # 先恢复终端, 防止 cbreak 模式残留导致终端卡死
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        except Exception:
            pass
        # 关闭时禁用阻抗 (仅在 rclpy 仍可用时发布)
        if rclpy.ok():
            try:
                msg = Bool()
                msg.data = False
                node.enable_pub.publish(msg)
                print('\n已发送 impedance_enable = False (安全关闭)')
                time.sleep(0.3)
            except Exception as e:
                print(f'\n[WARN] 禁用阻抗发布失败: {e}')
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

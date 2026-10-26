#!/usr/bin/env python3
"""Hold 模式测试: 启用阻抗 + 以当前末端位姿作为参考, 实现主动柔顺.

机械臂会保持当前位姿, 但允许外力推动 (K=刚度, D=阻尼).
用于验证阻抗控制是否生效, 以及参数是否合理.

流程:
  1. 订阅 /force_sensor/left/wrench 和 /force_sensor/right/wrench (可选)
  2. 等待 servo_node 启动
  3. 读取当前末端位姿 (从 /servo_node 反馈, 或手动指定)
  4. 把当前位姿作为参考下发到阻抗控制器
  5. 启用阻抗控制
  6. 等待用户按 Ctrl+C 结束, 结束时自动禁用阻抗

注意: 因为 servo_node 的双臂位姿反馈话题名称可能因配置而异,
     这里直接使用 base_link=body_link_2 在 URDF 中的零位姿态作为参考.
     用户可以手动指定参考位姿参数 --xyz 和 --rpy (单位: 米, 弧度).
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Accel, Quaternion, Vector3, Point
from snowbot_ctl_msgs.msg import CartesianImpedance
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.transform import Rotation as R


def rpy_to_quat(rpy):
    """rpy (xyz, 弧度) -> quaternion (xyzw)."""
    q = R.from_euler('xyz', rpy).as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--side', choices=['left', 'right', 'both'],
                        default='both')
    parser.add_argument('--xyz', nargs=3, type=float,
                        default=[0.0, 0.0, 0.0],
                        help='参考位置 [x y z] 米')
    parser.add_argument('--rpy', nargs=3, type=float,
                        default=[0.0, 0.0, 0.0],
                        help='参考姿态 [roll pitch yaw] 弧度')
    parser.add_argument('--q-null', nargs=7, type=float,
                        default=[0.0]*7,
                        help='零空间期望构型 [rad] (7 维)')
    parser.add_argument('--enable-only', action='store_true',
                        help='只发送 enable, 不发送参考 (用 URDF 零位作参考)')
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = Node('impedance_hold_test')

    enable_pub = node.create_publisher(Bool, '/servo_node/impedance_enable', 10)
    left_ref_pub = node.create_publisher(
        CartesianImpedance, '/servo_node/impedance/arm_l_link_7/reference', 1)
    right_ref_pub = node.create_publisher(
        CartesianImpedance, '/servo_node/impedance/arm_r_link_7/reference', 1)

    pose = Pose()
    pose.position = Point(x=args.xyz[0], y=args.xyz[1], z=args.xyz[2])
    pose.orientation = rpy_to_quat(args.rpy)
    zero_twist = Twist()
    zero_accel = Accel()

    def make_msg():
        msg = CartesianImpedance()
        msg.pose = pose
        msg.pose_twist = zero_twist
        msg.pose_accel = zero_accel
        # 零空间期望构型 (默认全零, 即用 yaml 中的 q_d_nullspace_default)
        msg.q_d_nullspace = list(args.q_null)
        return msg

    # 等待订阅者上线
    def wait_sub(pub, name, timeout=5.0):
        t0 = time.time()
        while pub.get_subscription_count() == 0:
            if time.time() - t0 > timeout:
                print(f'警告: {name} 订阅者未上线, 继续执行')
                return False
            rclpy.spin_once(node, timeout_sec=0.1)
        return True

    print('=== 阻抗 Hold 测试 ===')
    wait_sub(enable_pub, '/servo_node/impedance_enable')
    if args.side in ('left', 'both'):
        wait_sub(left_ref_pub, '/servo_node/impedance/arm_l_link_7/reference')
        left_ref_pub.publish(make_msg())
        print(f'已发送左臂参考: xyz={args.xyz} rpy={args.rpy}')
    if args.side in ('right', 'both'):
        wait_sub(right_ref_pub, '/servo_node/impedance/arm_r_link_7/reference')
        right_ref_pub.publish(make_msg())
        print(f'已发送右臂参考: xyz={args.xyz} rpy={args.rpy}')

    # 让参考消息先到达
    time.sleep(0.3)

    # 启用阻抗
    msg = Bool()
    msg.data = True
    enable_pub.publish(msg)
    print('已发送: impedance_enable = True')
    print('阻抗控制已启用. Ctrl+C 退出, 退出时自动禁用.')

    try:
        while rclpy.ok():
            # 周期性重发参考, 防止丢包 (1 Hz)
            if args.side in ('left', 'both'):
                left_ref_pub.publish(make_msg())
            if args.side in ('right', 'both'):
                right_ref_pub.publish(make_msg())
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭时禁用阻抗
        msg = Bool()
        msg.data = False
        enable_pub.publish(msg)
        print('已发送: impedance_enable = False (安全关闭)')
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

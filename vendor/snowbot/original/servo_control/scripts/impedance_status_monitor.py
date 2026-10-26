#!/usr/bin/env python3
"""实时监控阻抗控制器状态.

显示左右臂的 Hamiltonian (能量)、力矩做功、传感器功率、位姿/速度/加速度偏差.
用于调参时观察稳定性, 以及检查阻抗是否生效.

用法:
  ros2 run snowbot_servo_control impedance_status_monitor.py

status message 字段 (24 维):
  [0]  net_energy        = mechanical_energy - hamiltonian
  [1]  tau_power         = dq^T * tau_cmd
  [2]  sensor_power      = twist^T * sensor_wrench
  [3]  hamiltonian       = 1/2 dv^T M dv + 1/2 dx^T K dx
  [4]  hamiltonian_dot   = dH/dt
  [5]  damping_power     = dv^T * F_impedance
  [6:12]  pose_deviation (x,y,z,rx,ry,rz)
  [12:18] twist_deviation
  [18:24] accel_deviation
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ImpedanceStatusMonitor(Node):
    def __init__(self, side: str):
        super().__init__('impedance_status_monitor')
        # 左右臂 topic 由 ee_link 区分:
        #   /servo_node/impedance/arm_l_link_7/status  (左臂)
        #   /servo_node/impedance/arm_r_link_7/status  (右臂)
        ee_link = 'arm_l_link_7' if side == 'left' else 'arm_r_link_7'
        topic = f'/servo_node/impedance/{ee_link}/status'
        self.side = side
        self.create_subscription(
            Float64MultiArray, topic, self.cb, 1)
        self.last_print_time = 0.0
        self.get_logger().info(f'订阅 {topic}')

    def cb(self, msg: Float64MultiArray):
        if len(msg.data) < 24:
            return
        now = self.get_clock().now().nanoseconds() * 1e-9
        if now - self.last_print_time < 0.1:
            return  # 10Hz 显示
        self.last_print_time = now

        d = msg.data
        print(f'[{self.side:5s}] H={d[3]:+.3f} dH={d[4]:+.3f} '
              f'tau_P={d[1]:+.3f} sens_P={d[2]:+.3f} damp_P={d[5]:+.3f}')
        print(f'   dx=[{d[6]:+.3f},{d[7]:+.3f},{d[8]:+.3f},'
              f'{d[9]:+.3f},{d[10]:+.3f},{d[11]:+.3f}]')
        print(f'   dv=[{d[12]:+.3f},{d[13]:+.3f},{d[14]:+.3f},'
              f'{d[15]:+.3f},{d[16]:+.3f},{d[17]:+.3f}]')
        print(f'   da=[{d[18]:+.3f},{d[19]:+.3f},{d[20]:+.3f},'
              f'{d[21]:+.3f},{d[22]:+.3f},{d[23]:+.3f}]')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--side', choices=['left', 'right', 'both'],
                        default='both')
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    nodes = []
    if args.side in ('left', 'both'):
        nodes.append(ImpedanceStatusMonitor('left'))
    if args.side in ('right', 'both'):
        nodes.append(ImpedanceStatusMonitor('right'))

    if not nodes:
        print('没有创建任何监控节点')
        return 1

    from rclpy.executors import MultiThreadedExecutor
    exec = MultiThreadedExecutor()
    for n in nodes:
        exec.add_node(n)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

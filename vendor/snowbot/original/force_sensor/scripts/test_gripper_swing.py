#!/usr/bin/env python3
"""
夹爪开合测试 — 改下面参数
LEFT/RIGHT: 0=不动, 1=半正弦摆动 (0 ~ AMP ~ 0)
FREQ 控制开合周期 (0.5 Hz = 2s 一个开合周期)
AMP 控制幅度 (0=闭合, ~1.047=全开)
方向约定: R8 正值, L8 负值 (L_SIGN/R_SIGN)
速度取位置正弦的导数 (正弦速度), 峰值 = AMP*pi*FREQ

指令链路:
  /gripper/left_cmd  -> force_sensor 力控节点 -> /gripper/left_cmd_sensor -> servo_node -> 电机
  若 force_sensor 节点没在跑, servo_node 在 sensor 超时 200ms 后直接用本脚本的原始指令
"""

LEFT   = 1
RIGHT  = 1
FREQ   = 0.5      # 2s 一个开合周期
AMP    = 0.349    # 20度 = 20*pi/180
TOR    = 0.0
L_SIGN = -1       # 左夹爪: 负值
R_SIGN = +1       # 右夹爪: 正值

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class GripperTest(Node):
    def __init__(self):
        super().__init__('gripper_test')
        self.left_pub  = self.create_publisher(Float64MultiArray, '/gripper/left_cmd', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/gripper/right_cmd', 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f"left={LEFT} right={RIGHT} freq={FREQ} amp={AMP}")

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        w = 2 * math.pi * FREQ
        pos = AMP * (1 - math.cos(w * t)) / 2       # 位置: 0 ~ AMP 半正弦
        vel = AMP * w * math.sin(w * t) / 2         # 速度: 位置导数 (正弦), 峰值 AMP*pi*FREQ
        build = lambda p, v: Float64MultiArray(data=[float(p), float(v), float(TOR)])
        if LEFT:
            self.left_pub.publish(build(L_SIGN * pos, L_SIGN * vel))
        if RIGHT:
            self.right_pub.publish(build(R_SIGN * pos, R_SIGN * vel))

rclpy.init()
rclpy.spin(GripperTest())

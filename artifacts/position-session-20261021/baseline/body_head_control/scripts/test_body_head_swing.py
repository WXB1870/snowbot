#!/usr/bin/env python3
"""
躯干/头部正弦摆动 — 改下面 MASK 控制哪个电机动
顺序: [body1, body2, head1, head2]  1=锁零, 0=正弦摆
"""

MASK  = [1, 0, 1, 1]   # 1=锁零, 0=动. 顺序: body1,body2,head1,head2
HALF  = [0, 1, 0, 0]   # 1=只上摆(0~AMP), 0=全摆(-AMP~AMP)
SIGN  = [1, -1, 1, 1]  # 方向: 1=正向, -1=反向
FREQ  = 0.1
AMP   = 0.5
SPEED = 0.2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SwingTest(Node):
    def __init__(self):
        super().__init__('swing_test')
        self.body_pub = self.create_publisher(Float64MultiArray, '/body/cmd', 10)
        self.head_pub = self.create_publisher(Float64MultiArray, '/head/cmd', 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f"mask={MASK} half={HALF}")

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        full = AMP * math.sin(2 * math.pi * FREQ * t)
        half = AMP * (math.sin(2 * math.pi * FREQ * t) + 1) / 2   # 0 ~ AMP
        a = [SIGN[i] * (half if HALF[i] else full) for i in range(4)]
        b1, b2 = (a[0] if MASK[0]==0 else 0), (a[1] if MASK[1]==0 else 0)
        h1, h2 = (a[2] if MASK[2]==0 else 0), (a[3] if MASK[3]==0 else 0)

        build = lambda j1, j2: Float64MultiArray(data=[float(j1), float(j2)])
        self.body_pub.publish(build(b1, b2))
        self.head_pub.publish(build(h1, h2))

rclpy.init()
rclpy.spin(SwingTest())

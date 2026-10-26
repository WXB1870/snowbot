#!/usr/bin/env python3
"""仿真用 JointPositionCommand 多关节测试。

双臂多个关节做不同频率的正弦运动，并打印 JntCtlResVisu 中的
原始目标与处理后目标。不会访问 EtherCAT。
"""

import math

import rclpy
from rclpy.node import Node

from snowbot_ctl_msgs.msg import JointPositionCommand, JntCtlResVisu


FREQUENCY = 0.08      # Hz
PUBLISH_HZ = 50.0

# 关节顺序为 J1~J7，单位 rad。正负号用于制造明显的双臂联动动作。
LEFT_AMPLITUDE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3]
RIGHT_AMPLITUDE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3]
LEFT_SIGN = [1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
RIGHT_SIGN = [-1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]


class JointPositionCommandTest(Node):
    def __init__(self):
        super().__init__("test_joint_position_command")

        self.command_pub = self.create_publisher(
            JointPositionCommand, "/joint_position_cmd", 10
        )
        self.vis_sub = self.create_subscription(
            JntCtlResVisu,
            "/ctl_msg_pub/jnt_ctl_result_publish",
            self.visualization_callback,
            10,
        )

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_log_time = 0.0
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_command)

        self.get_logger().info(
            "开始发布 /joint_position_cmd：双臂 J1~J7 同频正弦目标"
        )

    def publish_command(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time
        phase = 2.0 * math.pi * FREQUENCY * elapsed
        left_position = [
            sign * amplitude * math.sin(phase)
            for amplitude, sign in zip(LEFT_AMPLITUDE, LEFT_SIGN)
        ]
        right_position = [
            sign * amplitude * math.sin(phase + math.pi)
            for amplitude, sign in zip(RIGHT_AMPLITUDE, RIGHT_SIGN)
        ]

        msg = JointPositionCommand()
        msg.l_arm_position = left_position
        msg.r_arm_position = right_position
        self.command_pub.publish(msg)

        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            self.get_logger().info(
                "发送目标："
                f"L=[{left_position[0]:+.2f}, {left_position[1]:+.2f}, "
                f"{left_position[2]:+.2f}, {left_position[3]:+.2f}, ...] "
                f"R=[{right_position[0]:+.2f}, {right_position[1]:+.2f}, "
                f"{right_position[2]:+.2f}, {right_position[3]:+.2f}, ...] rad"
            )

    def visualization_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        self.get_logger().info(
            "JntCtlResVisu: "
            f"ori L=[{msg.l_des_pos_ori[0]:+.2f}, {msg.l_des_pos_ori[1]:+.2f}, "
            f"{msg.l_des_pos_ori[2]:+.2f}, {msg.l_des_pos_ori[3]:+.2f}, ...] "
            f"R=[{msg.r_des_pos_ori[0]:+.2f}, {msg.r_des_pos_ori[1]:+.2f}, "
            f"{msg.r_des_pos_ori[2]:+.2f}, {msg.r_des_pos_ori[3]:+.2f}, ...]; "
            f"processed L=[{msg.l_des_pos[0]:+.2f}, {msg.l_des_pos[1]:+.2f}, "
            f"{msg.l_des_pos[2]:+.2f}, {msg.l_des_pos[3]:+.2f}, ...] "
            f"R=[{msg.r_des_pos[0]:+.2f}, {msg.r_des_pos[1]:+.2f}, "
            f"{msg.r_des_pos[2]:+.2f}, {msg.r_des_pos[3]:+.2f}, ...]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointPositionCommandTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

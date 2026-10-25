#!/usr/bin/env python3
"""启用或禁用阻抗控制.

用法:
  ros2 run snowbot_servo_control impedance_enable on      # 启用
  ros2 run snowbot_servo_control impedance_enable off     # 禁用
  ros2 run snowbot_servo_control impedance_enable toggle  # 切换

也可以直接用 ros2 topic pub 命令替代:
  ros2 topic pub --once /servo_node/impedance_enable std_msgs/Bool '{data: true}'

注意: 启用阻抗前应先确认机械臂已上电使能并完成回零,
     且阻抗参考位姿已下发或保持在当前末端位姿.
"""
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('on', 'off', 'toggle'):
        print(__doc__)
        return 1

    cmd = sys.argv[1]
    rclpy.init(args=sys.argv)
    node = Node('impedance_enable_cli')
    pub = node.create_publisher(Bool, '/servo_node/impedance_enable', 10)

    # 等待至少一个订阅者 (servo_node) 上线
    print(f'等待 servo_node 订阅 /servo_node/impedance_enable ...')
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return 1

    if cmd == 'on':
        value = True
    elif cmd == 'off':
        value = False
    else:  # toggle - 需要先查询当前状态
        # servo_node 不发布当前 enable 状态, 只能从外部记录; 这里默认切到 on
        print('注意: toggle 模式需要外部状态记录, 这里默认切到 ON. '
              '如需 OFF 请直接传入 off.')
        value = True

    msg = Bool()
    msg.data = value
    pub.publish(msg)
    print(f'已发送: impedance_enable = {value}')

    # 给消息一点时间送达 (Jazzy: sleep_for 需传 Duration 对象, 不再支持 float)
    node.get_clock().sleep_for(Duration(seconds=0.2))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

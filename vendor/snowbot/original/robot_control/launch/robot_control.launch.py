from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_source = LaunchConfiguration("control_source")

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_source",
            default_value="topic",
            choices=["demo", "vr", "topic"],
            description="控制来源：demo=演示轨迹，vr=VR遥操作，topic=关节位置Topic",
        ),
        Node(
            package="robot_control",
            executable="robot_control_node",
            name="robot_upperMonitor",
            output="screen",
            parameters=[{"control_source": control_source}],
        ),
    ])

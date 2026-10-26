"""Display launch for the dual_arm URDF in ROS 2 (Jazzy).

Replaces the old ROS 1 style XML `display.launch` file.  Loads the
URDF from `urdf/snowbot.urdf`, starts `robot_state_publisher` +
`joint_state_publisher_gui`, and opens RViz with the package's
`dual_arm.rviz` config.

Usage:
    ros2 launch dual_arm display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dual_arm')

    default_urdf = os.path.join(pkg_share, 'urdf', 'snowbot.urdf')
    default_rviz = os.path.join(pkg_share, 'dual_arm.rviz')

    urdf_file = LaunchConfiguration('urdf', default=default_urdf)
    rviz_file = LaunchConfiguration('rvizcfg', default=default_rviz)

    # Read the URDF so we can pass it as a parameter (the modern ROS 2 way,
    # rather than the old `textfile:=` attribute).
    with open(default_urdf, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=default_urdf),
        DeclareLaunchArgument('rvizcfg', default_value=default_rviz),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])

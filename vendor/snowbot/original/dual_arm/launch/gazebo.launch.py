"""Gazebo (modern, Ignition-based) launch for the dual_arm URDF.

Replaces the old ROS 1 style XML `gazebo.launch` file, which depended
on the now-removed Classic Gazebo (`gazebo_ros` `empty_world.launch`,
`spawn_model` etc.).

ROS 2 Jazzy ships with the new Gazebo (formerly Ignition Gazebo) and
the bridge package `ros_gz`.  This launch file:

    1. Starts Gazebo with an empty world.
    2. Spawns the URDF as a robot model via the `ros_gz_sim` create
       action.
    3. Publishes a static `base_link → base_footprint` TF.
    4. Starts `ros_gz_bridge` to forward `/joint_state` and clock.

Usage:
    ros2 launch dual_arm gazebo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dual_arm')

    urdf_file = LaunchConfiguration(
        'urdf',
        default=os.path.join(pkg_share, 'urdf', 'snowbot.urdf'))

    # 1. Gazebo Sim — empty world (provided by ros_gz_sim).
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Spawn the URDF in Gazebo.  The `create` executable reads the
    #    `file` argument and instantiates the model.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'dual_arm',
            '-file', urdf_file,
        ],
    )

    # 3. Static base_footprint → base_link TF (replaces the old
    #    `static_transform_publisher` ROS 1 node).
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
    )

    # 4. Bridge joint_states + clock between Gazebo and ROS 2.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_state@ros_gz_msg/JointState[sensor_msgs/msg/JointState',
            '/clock@ros_gz_msg/Clock[rosgraph_msgs/msg/Clock',
        ],
        remappings=[('/joint_state', '/joint_states')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=os.path.join(
            pkg_share, 'urdf', 'snowbot.urdf')),
        gz_launch,
        spawn,
        static_tf,
        bridge,
    ])

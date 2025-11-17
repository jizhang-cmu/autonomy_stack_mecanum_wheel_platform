#!/usr/bin/env python3
"""
Launch file for Unitree WebRTC control node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Unitree control node."""

    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.12.1',
        description='IP address of the Unitree Go2 robot'
    )

    connection_method_arg = DeclareLaunchArgument(
        'connection_method',
        default_value='LocalAP',
        description='Connection method: LocalAP, LocalSTA, or Remote'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='wireless_controller',
        description='Control mode: sport_cmd or wireless_controller'
    )

    # Create node
    unitree_control_node = Node(
        package='unitree_webrtc_ros',
        executable='unitree_control',
        name='unitree_control',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'connection_method': LaunchConfiguration('connection_method'),
            'control_mode': LaunchConfiguration('control_mode'),
        }],
        remappings=[
            # Uncomment if you need to remap topics
            # ('cmd_vel', '/robot/cmd_vel'),
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        connection_method_arg,
        control_mode_arg,
        unitree_control_node,
    ])

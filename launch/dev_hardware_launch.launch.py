#!/usr/bin/env python3
"""Launch joystick and teleop nodes on dev machine."""

import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')

    return launch.LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0}]  # or 1 if PS4 is second
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_joy')],
            parameters=[os.path.join(ursula_share, 'config', 'controller_teleop.yaml')]
        ),
    ])
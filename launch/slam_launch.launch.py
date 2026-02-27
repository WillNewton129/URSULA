#!/usr/bin/env python3
"""Launch slam_toolbox for mapping."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(ursula_share, 'config', 'slam_toolbox_params.yaml')
            ]
        ),
    ])
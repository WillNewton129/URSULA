#!/usr/bin/env python3
"""
dev_hardware_launch.launch.py

Operator interface running on the dev laptop.
Runs: joy_node + teleop_twist_joy + RViz

Connects to the Jetson over the ROS network automatically via
ROS_DOMAIN_ID - ensure both machines have the same ROS_DOMAIN_ID set.

Usage:
  Terminal 1 (Jetson): ros2 launch ursula jetson_hardware_launch.launch.py
  Terminal 2 (Laptop): ros2 launch ursula dev_hardware_launch.launch.py

Controls (PS4 controller):
  Hold RB         - deadman switch, must hold to move
  Left stick up   - forward
  Left stick down - backward
  Right stick     - turn left/right
  LB (button 4)   - emergency stop
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')

    return LaunchDescription([

        # ----------------------------------------------------------------
        # Joystick driver
        # ----------------------------------------------------------------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0}]
        ),

        # ----------------------------------------------------------------
        # Teleop - converts joystick to cmd_vel
        # Publishes to /cmd_vel_joy which twist_mux picks up at high priority
        # ----------------------------------------------------------------
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_joy')],
            parameters=[os.path.join(ursula_share, 'config', 'ps4_teleop.yaml')]
        ),

        # ----------------------------------------------------------------
        # RViz - visualisation and Nav2 goal setting
        # Displays map, robot model, laser scan, odometry, planned path
        # ----------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(ursula_share, 'rviz', 'ursula.rviz')],
        ),

    ])

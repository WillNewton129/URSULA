#!/usr/bin/env python3
"""
sim_teleop_launch.launch.py

Lightweight teleop launcher for simulation.
Run this in a SEPARATE terminal alongside gazebo_sim_launch.

FIX LOG:
  - teleop_twist_joy now remaps /cmd_vel -> /cmd_vel_joy so twist_mux
    (running in gazebo_sim_launch) picks it up at priority 100.
    Previously it published directly to /cmd_vel, bypassing twist_mux
    entirely. This meant Gazebo received joystick commands fine in
    isolation, but broke as soon as Nav2 was also publishing to /cmd_vel.
    With the mux in place, joystick always wins over Nav2.

Usage:
  Terminal 1 (sim):     ros2 launch ursula gazebo_sim_launch.launch.py
  Terminal 2 (teleop):  ros2 launch ursula sim_teleop_launch.launch.py

Controls (PS4 / F310 — hold RB to enable):
  Left stick up/down  — forward/backward
  Right stick L/R     — turn left/right
  LB (button 4)       — emergency stop
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ursula_share = get_package_share_directory('ursula')

    # Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'use_sim_time': True,
            'device_id': 0,
        }]
    )

    # Teleop — publishes to /cmd_vel_joy (FIX: was publishing to /cmd_vel directly)
    # twist_mux in gazebo_sim_launch forwards /cmd_vel_joy -> /cmd_vel at priority 100
    joystick_teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_joy')],   # FIX: remapping added
        parameters=[
            os.path.join(ursula_share, 'config', 'controller_teleop.yaml'),
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        joy_node,
        joystick_teleop,
    ])
#!/usr/bin/env python3
"""
nav2_launch.launch.py
Launch the Nav2 autonomous navigation stack for URSULA.

In ROS 2 Humble, nav2_controller does NOT have a 'cmd_vel_topic' parameter —
that parameter was added in later distributions and is silently ignored here.
The correct way to redirect Nav2 velocity output is a node-level remapping on
the controller_server node, which is what this file does:

    controller_server: /cmd_vel  →  /cmd_vel_nav

twist_mux.yaml subscribes to /cmd_vel_nav at priority 10 (lowest), so the
joystick (priority 100) always overrides autonomous navigation.

Usage (sim):
  ros2 launch ursula nav2_launch.launch.py use_sim_time:=true

Usage (real robot, called from jetson_hardware_launch.launch.py):
  ros2 launch ursula nav2_launch.launch.py use_sim_time:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')
    nav2_params  = os.path.join(ursula_share, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,

        # ── Controller server ──────────────────────────────────────────────
        # IMPORTANT: remappings=[('/cmd_vel', '/cmd_vel_nav')] is the ONLY
        # reliable way to redirect nav2 output in Humble.  The YAML parameter
        # 'cmd_vel_topic' does not exist in this distribution.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Planner server ─────────────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Behavior server (renamed from nav2_recoveries in Foxy) ─────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── BT Navigator ───────────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Lifecycle manager ──────────────────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]}
            ]
        ),
    ])
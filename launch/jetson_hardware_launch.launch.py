#!/usr/bin/env python3
"""Launch serial bridge, lidar, and odometry on Jetson."""

import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')
    urdf_file = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # Serial bridge to Arduino
        Node(
            package='ursula',
            executable='serial_command_bridge.py',
            name='serial_command_bridge',
            output='screen',
            parameters=[os.path.join(ursula_share, 'config', 'serial_bridge_params.yaml')]
        ),

        # RPLIDAR driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}]  # Adjust serial port
        ),

        # RF2O odometry node
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry_node',
            output='screen',
            parameters=[{'scan_topic': '/scan', 'odom_frame': 'odom', 'base_frame': 'base_link'}]
        ),
        # Robot state publisher - publish TF Tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # twist_mux to combine cmd_vel from teleop and Nav2
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[os.path.join(ursula_share, 'config', 'twist_mux.yaml')],
            remappings=[('/cmd_vel_out', '/cmd_vel')]
        ),
    ])

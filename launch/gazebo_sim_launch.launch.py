#!/usr/bin/env python3
"""
gazebo_sim_launch.launch.py

Launches the full URSULA simulation stack:
  - Gazebo Classic with the maze world
  - robot_state_publisher (URDF -> TF)
  - Spawn the robot into Gazebo
  - twist_mux  (ADDED - was missing, causing /cmd_vel_joy to be ignored)
  - slam_toolbox
  - Nav2
  - Foxglove bridge
  - RViz2

FIX LOG (vs previous version):
  1. Added twist_mux node — without it /cmd_vel_joy never reached /cmd_vel.
  2. Removed the four static_transform_publisher nodes for wheel/susp links.
     Those were publishing zero-offset transforms that conflicted with the
     diff drive plugin's own wheel TF and with robot_state_publisher.
     robot_state_publisher + joint_state_publisher already handle all fixed
     joints in the URDF; no static TF overrides are needed.
  3. sim_teleop_launch.launch.py publishes joystick to /cmd_vel_joy
     (remapping added there). twist_mux then forwards the highest-priority
     active source to /cmd_vel which Gazebo's diff drive plugin reads.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    ursula_share = get_package_share_directory('ursula')

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    use_rviz_arg     = DeclareLaunchArgument('use_rviz',   default_value='true')
    use_slam_arg     = DeclareLaunchArgument('slam',       default_value='true')
    use_nav2_arg     = DeclareLaunchArgument('nav2',       default_value='true')
    use_foxglove_arg = DeclareLaunchArgument('foxglove',   default_value='true')
    x_spawn_arg      = DeclareLaunchArgument('x_spawn',    default_value='0.0')
    y_spawn_arg      = DeclareLaunchArgument('y_spawn',    default_value='-4.0')
    yaw_spawn_arg    = DeclareLaunchArgument('yaw_spawn',  default_value='0.0')

    use_rviz     = LaunchConfiguration('use_rviz')
    use_slam     = LaunchConfiguration('slam')
    use_nav2     = LaunchConfiguration('nav2')
    use_foxglove = LaunchConfiguration('foxglove')
    x_spawn      = LaunchConfiguration('x_spawn')
    y_spawn      = LaunchConfiguration('y_spawn')
    yaw_spawn    = LaunchConfiguration('yaw_spawn')

    # ------------------------------------------------------------------ #
    # Process URDF / xacro                                                #
    # ------------------------------------------------------------------ #
    urdf_file = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # ------------------------------------------------------------------ #
    # Gazebo                                                               #
    # ------------------------------------------------------------------ #
    world_file = os.path.join(ursula_share, 'worlds', 'ursula_maze.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    # ------------------------------------------------------------------ #
    # Robot State Publisher                                                #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True,
        }]
    )

    # ------------------------------------------------------------------ #
    # Joint State Publisher                                                #
    # Publishes zero positions for joints the diff drive plugin doesn't   #
    # drive (suspension rods, front wheels). Without this, RViz shows     #
    # "no transform" warnings for those links.                            #
    # NOTE: joint_state_publisher and the Gazebo diff drive plugin both   #
    # publish /joint_states — they will merge. The plugin's values for    #
    # rear_left/right will override JSP's zeros for those joints.         #
    # ------------------------------------------------------------------ #
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # ------------------------------------------------------------------ #
    # Spawn robot in Gazebo                                                #
    # Increased delay to 8s — gives Gazebo time to fully load the world   #
    # and robot_state_publisher time to latch robot_description.          #
    # ------------------------------------------------------------------ #
    spawn_entity = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_ursula',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'ursula',
                    '-x', x_spawn,
                    '-y', y_spawn,
                    '-z', '0.12',
                    '-Y', yaw_spawn,
                ]
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # Twist Mux  (FIX: was missing from sim launch)                       #
    #                                                                      #
    # Priority chain:                                                      #
    #   /cmd_vel_joy  (100) — joystick, from sim_teleop_launch            #
    #   /cmd_vel_nav  (10)  — Nav2, from nav2_launch                      #
    # Output: /cmd_vel — read by Gazebo diff drive plugin                 #
    # ------------------------------------------------------------------ #
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(ursula_share, 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # ------------------------------------------------------------------ #
    # SLAM Toolbox                                                         #
    # ------------------------------------------------------------------ #
    slam_toolbox = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(ursula_share, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': True}
        ]
    )

    # ------------------------------------------------------------------ #
    # Nav2 — delayed 15s to allow SLAM to establish map->odom TF first    #
    # ------------------------------------------------------------------ #
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                condition=IfCondition(use_nav2),
                launch_description_source=PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'nav2_launch.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # Foxglove bridge                                                      #
    # ------------------------------------------------------------------ #
    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'topic_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_sim_time': True,
        }]
    )

    # ------------------------------------------------------------------ #
    # RViz2                                                                #
    # ------------------------------------------------------------------ #
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(ursula_share, 'rviz', 'ursula.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_rviz_arg,
        use_slam_arg,
        use_nav2_arg,
        use_foxglove_arg,
        x_spawn_arg,
        y_spawn_arg,
        yaw_spawn_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        twist_mux,          # FIX: added
        slam_toolbox,
        nav2,
        foxglove_bridge,
        rviz,
    ])
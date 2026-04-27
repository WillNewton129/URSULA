#!/usr/bin/env python3
"""
sim_full_test_launch.launch.py

Simulation equivalent of robot_launch.launch.py.

Mirrors the real robot launch as closely as possible so that every feature
can be verified in Gazebo before going near the hardware.  The argument
names and defaults are intentionally identical to robot_launch.launch.py
so your muscle memory (and the web panel) transfers directly.

What is the same as the real robot:
  - SLAM toolbox mapping / localization modes
  - ursula_manager (map save service, status publisher)
  - Nav2 full stack
  - twist_mux priority chain  (joystick > Nav2)
  - Foxglove bridge on port 8765
  - Gazebo camera plugin publishes /oak_d/rgb/image_raw + depth topics
  - Mock detector publishes Detection2DArray on /oak/nn/detections
  - detection_mapper.py subscribes to those and publishes markers

What is different from the real robot:
  - Gazebo diff-drive plugin replaces RPLidar + RF2O + serial bridge + Arduino
  - Gazebo camera plugin replaces depthai_ros_driver (same topic names)
  - mock_detector.py replaces tracker_yolov4_spatial_node (no VPU needed)
  - use_sim_time: true everywhere

Launch arguments (identical to robot_launch.launch.py):
  slam_mode:=mapping|localization   (default: mapping)
  nav2:=true|false                  (default: true)
  foxglove:=true|false              (default: true)
  camera:=true|false                (default: true)   gates mock detector too
  detection:=true|false             (default: true)
  use_rviz:=true|false              (default: true)

Spawn position args:
  x_spawn, y_spawn, yaw_spawn

Usage:
  # Full test (mapping + Nav2 + camera + detection + Foxglove + RViz):
  ros2 launch ursula sim_full_test_launch.launch.py

  # Step 1 - just SLAM, no Nav2/detection (verify motors + map building):
  ros2 launch ursula sim_full_test_launch.launch.py nav2:=false detection:=false camera:=false

  # Step 2 - SLAM + Nav2 (verify autonomous navigation + estop):
  ros2 launch ursula sim_full_test_launch.launch.py detection:=false camera:=false

  # Step 3 - Add mock camera + detection (verify marker pipeline):
  ros2 launch ursula sim_full_test_launch.launch.py

  # Localization mode (after saving a map in mapping mode):
  ros2 launch ursula sim_full_test_launch.launch.py slam_mode:=localization nav2:=true

  # No RViz (e.g. headless or Foxglove-only session):
  ros2 launch ursula sim_full_test_launch.launch.py use_rviz:=false

Joystick (run in a separate terminal):
  ros2 launch ursula sim_teleop_launch.launch.py

Map saving (while sim is running):
  ros2 service call /ursula/save_map std_srvs/srv/Trigger {}

Map saving — alternative direct slam_toolbox call:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/tmp/sim_map'}}"

Switch to localization after saving:
  Ctrl+C the launch, then:
  ros2 launch ursula sim_full_test_launch.launch.py slam_mode:=localization

Foxglove:
  Connect from any device on the same network:
  ws://<sim-machine-ip>:8765
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    ursula_share = get_package_share_directory('ursula')
    urdf_file    = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # ------------------------------------------------------------------ #
    # Launch arguments — identical names to robot_launch.launch.py        #
    # ------------------------------------------------------------------ #
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description='"mapping" starts fresh, "localization" loads saved map'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Launch Nav2 autonomous navigation stack'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start Foxglove bridge WebSocket server on port 8765'
    )
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Enable Gazebo camera plugin topics + mock detector'
    )
    detection_arg = DeclareLaunchArgument(
        'detection', default_value='true',
        description='Launch mock_detector + detection_mapper'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'
    )
    x_spawn_arg   = DeclareLaunchArgument('x_spawn',   default_value='0.0')
    y_spawn_arg   = DeclareLaunchArgument('y_spawn',   default_value='-4.0')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='0.0')

    slam_mode     = LaunchConfiguration('slam_mode')
    use_nav2      = LaunchConfiguration('nav2')
    use_foxglove  = LaunchConfiguration('foxglove')
    use_camera    = LaunchConfiguration('camera')
    use_detection = LaunchConfiguration('detection')
    use_rviz      = LaunchConfiguration('use_rviz')
    x_spawn       = LaunchConfiguration('x_spawn')
    y_spawn       = LaunchConfiguration('y_spawn')
    yaw_spawn     = LaunchConfiguration('yaw_spawn')

    # ------------------------------------------------------------------ #
    # Gazebo world                                                         #
    # ------------------------------------------------------------------ #
    world_file = os.path.join(ursula_share, 'worlds', 'ursula_maze.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world':   world_file,
            'verbose': 'false',
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # Robot state publisher                                                #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time':      True,
        }]
    )

    # ------------------------------------------------------------------ #
    # Joint state publisher                                                #
    # Publishes zero states for suspension / front wheel joints so        #
    # robot_state_publisher can compute TF for all links.                 #
    # ------------------------------------------------------------------ #
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # ------------------------------------------------------------------ #
    # Spawn robot — delayed 8 s so Gazebo fully loads the world first     #
    # ------------------------------------------------------------------ #
    spawn_entity = TimerAction(
        period=30.0,
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
    # Twist mux                                                            #
    # /cmd_vel_joy  priority 100 — joystick (sim_teleop_launch.launch.py) #
    # /cmd_vel_nav  priority 10  — Nav2 controller_server                 #
    # Output: /cmd_vel → Gazebo diff-drive plugin                         #
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
    # SLAM Toolbox — MAPPING mode                                          #
    # ------------------------------------------------------------------ #
    slam_toolbox_mapping = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'mapping'),
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
    # SLAM Toolbox — LOCALIZATION mode                                     #
    #                                                                      #
    # Uses slam_toolbox_localization.yaml but overrides map_file_name to  #
    # point at /tmp/sim_map (where ursula_manager saves sim maps).        #
    # Before using this mode you must have saved a map with:              #
    #   ros2 service call /ursula/save_map std_srvs/srv/Trigger {}        #
    # ------------------------------------------------------------------ #
    slam_toolbox_localization = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'localization'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(ursula_share, 'config', 'slam_toolbox_localization.yaml'),
            {
                'use_sim_time':    True,
                # Override the Jetson path — sim maps go to /tmp/sim_maps/
                'map_file_name':   '/tmp/sim_maps/ursula_map',
            }
        ]
    )

    # ------------------------------------------------------------------ #
    # URSULA Manager                                                       #
    # Provides /ursula/save_map service and /ursula/status publisher.     #
    # map_save_dir overridden to /tmp/sim_maps so we don't pollute the   #
    # Jetson maps directory with sim data.                                 #
    # ------------------------------------------------------------------ #
    ursula_manager = Node(
        package='ursula',
        executable='ursula_manager.py',
        name='ursula_manager',
        output='screen',
        parameters=[{
            'map_save_dir': '/tmp/sim_maps',
            'use_sim_time': True,
        }]
    )

    # ------------------------------------------------------------------ #
    # Foxglove bridge                                                      #
    # Connect from any device: ws://<sim-machine-ip>:8765                 #
    # ------------------------------------------------------------------ #
    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port':              8765,
            'address':           '0.0.0.0',
            'topic_whitelist':   ['.*'],
            'send_buffer_limit': 10000000,
            'use_sim_time':      True,
            'max_qos_depth':     5,
        }]
    )

    # ------------------------------------------------------------------ #
    # Mock detector                                                        #
    # Replaces tracker_yolov4_spatial_node in simulation.                 #
    # Publishes synthetic Detection2DArray on /oak/nn/detections so       #
    # detection_mapper.py can be verified end-to-end.                     #
    # Delayed 12 s to allow robot to spawn and TF tree to settle.         #
    # Gated by both camera AND detection args (mirrors real robot logic). #
    # ------------------------------------------------------------------ #
    mock_detector = TimerAction(
        period=32.0,
        actions=[
            Node(
                condition=IfCondition(use_camera),
                package='ursula',
                executable='mock_detector.py',
                name='mock_detector',
                output='screen',
                parameters=[{
                    'use_sim_time':       True,
                    'publish_hz':         0.2,   # One detection every 5 s — low noise
                    'image_topic':        '/oak_d/rgb/image_raw',
                }]
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # Detection mapper                                                     #
    # Identical to real robot — subscribes to /oak/nn/detections,        #
    # publishes /ursula/detection_markers for Foxglove 3D panel.          #
    # Delayed 14 s — needs mock_detector to be running first.            #
    # ------------------------------------------------------------------ #
    detection_mapper = TimerAction(
        period=34.0,
        actions=[
            Node(
                condition=IfCondition(use_detection),
                package='ursula',
                executable='detection_mapper.py',
                name='detection_mapper',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # Nav2 — delayed 15 s                                                  #
    # Gives SLAM time to establish map->odom TF before Nav2 starts.       #
    # Velocity output remapped inside nav2_launch: /cmd_vel -> /cmd_vel_nav
    # twist_mux picks up /cmd_vel_nav at priority 10.                     #
    # Joystick at priority 100 always overrides — this is the estop test. #
    # ------------------------------------------------------------------ #
    nav2 = TimerAction(
        period=36.0,
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
        # Args
        slam_mode_arg,
        nav2_arg,
        foxglove_arg,
        camera_arg,
        detection_arg,
        use_rviz_arg,
        x_spawn_arg,
        y_spawn_arg,
        yaw_spawn_arg,

        # Sim environment
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,       # delayed 8 s

        # Core stack — starts immediately after gazebo
        twist_mux,

        # SLAM (one fires depending on slam_mode)
        slam_toolbox_mapping,
        slam_toolbox_localization,

        # Management + monitoring
        ursula_manager,
        foxglove_bridge,

        # Mock camera pipeline — delayed 12 s + 14 s
        mock_detector,
        detection_mapper,

        # Nav2 — delayed 15 s
        nav2,

        # Visualisation
        rviz,
    ])
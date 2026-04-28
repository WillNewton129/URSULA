#!/usr/bin/env python3
"""
sim_full_test_launch.launch.py

Full simulation stack — updated to use ursula_open_lab.world (larger,
more open, better for building a complete map quickly).

KEY CHANGES FROM PREVIOUS VERSION:
  1. Default world changed to ursula_open_lab.world (20m x 16m open lab).
     Previous maze world still available via world:=maze argument.
  2. Sim map save dir changed to ~/ros2_ws/maps/sim  (persists across reboots).
     Was /tmp/sim_maps which was wiped on every reboot, losing saved maps.
  3. Spawn position updated: x=0, y=-6 (south end of new world, facing north).
     Override with x_spawn/y_spawn args if needed.
  4. Localization mode now prints a clear confirmation to the terminal on startup.

ARGUMENTS (identical to previous version — your muscle memory is safe):
  world:=open_lab|maze        (default: open_lab)
  slam_mode:=mapping|localization  (default: mapping)
  nav2:=true|false            (default: true)
  foxglove:=true|false        (default: true)
  camera:=true|false          (default: true)
  detection:=true|false       (default: true)
  use_rviz:=true|false        (default: true)
  x_spawn, y_spawn, yaw_spawn

TYPICAL WORKFLOW:
  # Step 1 — build a map
  ros2 launch ursula sim_full_test_launch.launch.py nav2:=false detection:=false camera:=false

  # Step 2 — drive around with joystick (separate terminal):
  ros2 launch ursula sim_teleop_launch.launch.py

  # Step 3 — save the map when it looks complete:
  ros2 service call /ursula/save_map std_srvs/srv/Trigger {}
  # Map files appear in ~/ros2_ws/maps/sim/
  # PNG preview auto-generated at ~/ros2_ws/maps/sim/ursula_map.png
  # Open it: xdg-open ~/ros2_ws/maps/sim/ursula_map.png

  # Step 4 — verify map exists and looks good:
  ls -la ~/ros2_ws/maps/sim/
  xdg-open ~/ros2_ws/maps/sim/ursula_map.png

  # Step 5 — Ctrl+C the mapping launch, then relaunch in localization:
  ros2 launch ursula sim_full_test_launch.launch.py slam_mode:=localization nav2:=false detection:=false camera:=false

  # Step 6 — verify localization is actually running (separate terminal):
  ros2 param get /slam_toolbox mode       # Should print: localization
  ros2 topic echo /map --once             # Map should have data immediately, before driving

MOCK DETECTOR NOTE:
  The mock_detector.py publishes SYNTHETIC detections — it does NOT do
  vision-based object recognition. It randomly cycles through COCO class IDs
  (person, bottle, chair, laptop, cell phone, book) regardless of what the
  camera sees. The objects in ursula_open_lab.world are there for spatial
  reference and to give the map distinctive LiDAR features.

  The pipeline to verify:
    /oak_d/rgb/image_raw   → mock_detector watches this to know camera is live
    /oak/nn/detections     → mock_detector publishes here (synthetic)
    /ursula/detection_markers → detection_mapper converts to RViz/Foxglove markers

  To check: ros2 topic hz /oak/nn/detections   (should be ~0.2 Hz)

JOINT_CMD WARNING:
  You may see: [gzclient] Error advertising topic [/ursula/joint_cmd]
  This is a harmless internal Gazebo Classic warning from the diff-drive plugin.
  It does not affect functionality. Ignore it.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    ursula_share = get_package_share_directory('ursula')
    urdf_file    = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # Sim maps now saved here — survives reboots unlike /tmp
    sim_map_dir = os.path.expanduser('~/ros2_ws/maps/sim')
    os.makedirs(sim_map_dir, exist_ok=True)

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='open_lab',
        description='"open_lab" (new, larger) or "maze" (original small maze)'
    )
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
    y_spawn_arg   = DeclareLaunchArgument('y_spawn',   default_value='0.0')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='1.5708')  # facing north

    world_choice  = LaunchConfiguration('world')
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
    # World file selection                                                 #
    # ------------------------------------------------------------------ #
    open_lab_world = os.path.join(ursula_share, 'worlds', 'ursula_open_lab.world')
    maze_world     = os.path.join(ursula_share, 'worlds', 'ursula_maze.world')

    # We pick the world file using PythonExpression on the world arg.
    # LaunchConfiguration doesn't support if/else directly, so we use
    # a Python expression that evaluates at launch time.
    world_file = PythonExpression([
        f'"{open_lab_world}" if "',
        world_choice,
        f'" == "open_lab" else "{maze_world}"'
    ])

    # ------------------------------------------------------------------ #
    # Gazebo                                                               #
    # ------------------------------------------------------------------ #
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

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # ------------------------------------------------------------------ #
    # Spawn robot — delayed 30 s for Gazebo to fully load world           #
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
    # Reads from ~/ros2_ws/maps/sim/ursula_map (saved by ursula_manager). #
    # IMPORTANT: localization mode in slam_toolbox does NOT pre-populate  #
    # the map topic until it receives at least one scan to localise on.   #
    # Give it 5-10 seconds after the laser starts publishing.             #
    #                                                                      #
    # To VERIFY localization is actually running (not remapping):         #
    #   ros2 param get /slam_toolbox mode    → should print "localization" #
    #   ros2 topic echo /map --once          → data should be non-empty   #
    #                                          BEFORE you drive anywhere  #
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
                'use_sim_time':  True,
                'map_file_name': sim_map_dir + '/ursula_map',
            }
        ]
    )

    # ------------------------------------------------------------------ #
    # Localization startup message — printed to terminal when launched    #
    # in localization mode so you know what to check                      #
    # ------------------------------------------------------------------ #
    localization_info = LogInfo(
        condition=LaunchConfigurationEquals('slam_mode', 'localization'),
        msg=(
            '\n'
            '========================================================\n'
            ' LOCALIZATION MODE\n'
            '========================================================\n'
            f' Loading map from: {sim_map_dir}/ursula_map\n'
            ' To verify localization is working:\n'
            '   ros2 param get /slam_toolbox mode\n'
            '     → Should print: String value is: localization\n'
            '   ros2 topic echo /map --once\n'
            '     → Should show map data BEFORE you drive\n'
            '   In RViz: map should appear fully drawn on startup\n'
            '========================================================\n'
        )
    )

    # ------------------------------------------------------------------ #
    # URSULA Manager                                                       #
    # Maps saved to ~/ros2_ws/maps/sim so they survive reboots.           #
    # PNG preview auto-generated after each save.                         #
    # ------------------------------------------------------------------ #
    ursula_manager = Node(
        package='ursula',
        executable='ursula_manager.py',
        name='ursula_manager',
        output='screen',
        parameters=[{
            'map_save_dir': sim_map_dir,
            'use_sim_time': True,
        }]
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
            'port':              8765,
            'address':           '0.0.0.0',
            'topic_whitelist':   ['.*'],
            'send_buffer_limit': 10000000,
            'use_sim_time':      True,
            'max_qos_depth':     5,
        }]
    )

    # ------------------------------------------------------------------ #
    # Mock detector — delayed 32 s (after robot spawns)                   #
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
                    'use_sim_time':  True,
                    'publish_hz':    0.2,
                    'image_topic':   '/oak_d/rgb/image_raw',
                }]
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # Detection mapper — delayed 34 s                                     #
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
    # Nav2 — delayed 36 s                                                  #
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
        world_arg,
        slam_mode_arg,
        nav2_arg,
        foxglove_arg,
        camera_arg,
        detection_arg,
        use_rviz_arg,
        x_spawn_arg,
        y_spawn_arg,
        yaw_spawn_arg,

        # Info messages
        localization_info,

        # Sim environment
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,

        # Core stack
        twist_mux,

        # SLAM
        slam_toolbox_mapping,
        slam_toolbox_localization,

        # Management
        ursula_manager,
        foxglove_bridge,

        # Detection pipeline
        mock_detector,
        detection_mapper,

        # Nav2
        nav2,

        # Visualisation
        rviz,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    parameters = {
        'frame_id':                  'base_link',
        'odom_frame_id':             'odom',
        'approx_sync':               True,
        'approx_sync_max_interval':  0.02,
        'queue_size':                10,
        'qos':                       1,
    }

    remappings = [
        ('rgb/image',       '/oak/rgb/image_raw'),
        ('rgb/camera_info', '/oak/rgb/camera_info'),
        ('depth/image',     '/oak/stereo/image_raw'),
    ]

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'oak-d-base-frame']
        ),

        # Converts depth image to laser scan for Nav2 costmap
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth',             '/oak/stereo/image_raw'),
                ('depth_camera_info', '/oak/stereo/camera_info'),
            ],
            parameters=[{
                'scan_height': 10,
                'range_min':   0.3,
                'range_max':   8.0,
            }]
        ),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                **parameters,
                'publish_null_when_lost': False,
                'guess_frame_id': 'base_link',
            }],
            remappings=remappings
        ),
        
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **parameters,
                'map_frame_id':           'map',
                'subscribe_depth':        True,
                'subscribe_rgb':          True,
                'database_path':          '~/.ros/rtabmap.db',
                'Mem/IncrementalMemory':  'true',
                'Mem/InitWithSavedMap':   'true',
                'Vis/MinInliers':         '10',
                'Reg/Strategy':           '0',
                'RGBD/OptimizeMaxError':  '3.0',
                'RGBD/AngularUpdate':     '0.1',
                'RGBD/LinearUpdate':      '0.1',
                # Occupancy grid for Nav2
                'Grid/FromDepth':         'true',
                'Grid/3D':                'false',
                'Mem/OccupancyGrid':      'true',
            }],
            remappings=remappings + [('odom', '/odom')]
        ),

        #Node(
            #package='rtabmap_viz',
            #executable='rtabmap_viz',
            #name='rtabmap_viz',
            #output='screen',
            #parameters=[{
                #**parameters,
                #'subscribe_odom_info': True,
            #}],
            #remappings=remappings + [('odom', '/odom')]
        #),

    ])
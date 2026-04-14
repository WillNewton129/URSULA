import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    depthai_examples_path = get_package_share_directory('depthai_examples')
    default_resources_path = os.path.join(depthai_examples_path, 'resources')

    monoResolution = LaunchConfiguration('monoResolution', default='400p')

    declare_mono_res = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Mono camera resolution: 800p, 720p, 400p')

    # YOLOv4 spatial tracker node (runs inference on OAK-D VPU)
    tracker_node = launch_ros.actions.Node(
        package='depthai_examples',
        executable='tracker_yolov4_spatial_node',
        name='tracker_yolov4_spatial_node',
        output='screen',
        parameters=[{
    'tf_prefix': 'oak',
    'monoResolution': monoResolution,
    'resourceBaseFolder': '/home/uoljetson/oak_camera_ws/src/depthai-ros/depthai_examples/resources',
    'nnName': 'yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob',
    'sync_nn': True,
    'fullFrameTracking': False,
}]
    )

    # Detection visualiser node (publishes RViz2 markers)
    visualiser_node = launch_ros.actions.Node(
        package='my_robot_bringup',
        executable='detection_visualiser',
        name='detection_visualiser',
        output='screen',
    )

    return LaunchDescription([
        declare_mono_res,
        tracker_node,
        visualiser_node,
    ])
import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    video_device_arg = DeclareLaunchArgument('video_device', default_value='/dev/video0')

    camera_config = os.path.join(
        get_package_share_directory('baggage_advisor'),
        'config', 'camera.yaml'
    )

    container = ComposableNodeContainer(
        name='baggage_advisor_container',
        package='rclcpp_components',
        namespace='',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='baggage_advisor',
                plugin='baggage_advisor::advisor',
                name='advisor',
            ),
            ComposableNode(
                package='barcode_scan',
                plugin='barcode_scan::scanner',
                name='scanner',
                remappings=[
                    ('image_raw', 'webcam/image_raw')
                ]
            ),
            ComposableNode(
                package='image_draw',
                plugin='image_draw::image_draw',
                name='image_draw',
                remappings=[
                    ('image_raw', 'webcam/image_raw'),
                    ('image_marked', 'webcam/image_marked')
                ]
            ),
            ComposableNode(
                package='v4l2_camera',
                namespace='webcam',
                plugin='v4l2_camera::V4L2Camera',
                name='camera',
                parameters=[
                    camera_config,
                    {"video_device": launch.substitutions.LaunchConfiguration('video_device')}
                ]
            )
        ],
        output='screen',
        emulate_tty=True
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        namespace='',
        executable='rqt_image_view',
        name='camera_view'
    )

    return launch.LaunchDescription([
        container,
        video_device_arg,
        rqt_image_view_node
    ])

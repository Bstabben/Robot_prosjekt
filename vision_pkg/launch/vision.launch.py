from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('vision_pkg'), 'config', 'hsv_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='4',
            description='USB camera device index (e.g. 1 for /dev/video1)',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish annotated debug image on vision/debug_image',
        ),

        Node(
            package='vision_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                config_file,
                {'device_id': LaunchConfiguration('device_id')},
            ],
        ),

        Node(
            package='vision_pkg',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[
                config_file,
                {'publish_debug_image': LaunchConfiguration('publish_debug_image')},
            ],
        ),
    ])

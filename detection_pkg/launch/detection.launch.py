from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('detection_pkg'), 'config', 'detection_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish annotated debug image on vision/debug_image',
        ),

        Node(
            package='detection_pkg',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[
                config_file,
                {'publish_debug_image': LaunchConfiguration('publish_debug_image')},
            ],
        ),
    ])

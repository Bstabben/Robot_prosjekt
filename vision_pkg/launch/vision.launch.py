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
        DeclareLaunchArgument(
            'table_z',
            default_value='0.0',
            description='Table surface Z in base_link frame (metres) — measure with TCP',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description=(
                'URL to camera calibration file, e.g. '
                'file:///home/user/calibration.yaml. '
                'Leave empty before calibration is done.'
            ),
        ),

        Node(
            package='vision_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'device_id': LaunchConfiguration('device_id'),
                    'calibration_url': LaunchConfiguration('calibration_file'),
                },
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

        Node(
            package='vision_pkg',
            executable='transform_node',
            name='transform_node',
            output='screen',
            parameters=[
                config_file,
                {'table_z': LaunchConfiguration('table_z')},
            ],
        ),
    ])

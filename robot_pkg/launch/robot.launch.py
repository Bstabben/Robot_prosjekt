from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_pkg'), 'config', 'robot_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'approach_height',
            default_value='0.10',
            description='Distance above cube surface to stop at (metres)',
        ),

        Node(
            package='robot_pkg',
            executable='motion_node',
            name='motion_node',
            output='screen',
            parameters=[
                config_file,
                {'approach_height': LaunchConfiguration('approach_height')},
            ],
        ),

        Node(
            package='robot_pkg',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen',
            parameters=[config_file],
        ),
    ])

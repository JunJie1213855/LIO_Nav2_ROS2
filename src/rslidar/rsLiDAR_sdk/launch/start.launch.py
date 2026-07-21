import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('rslidar_sdk')

    config_path = LaunchConfiguration(
        'config_path',
        default=os.path.join(pkg_dir, 'config', 'config.yaml')
    )

    rviz_config = os.path.join(pkg_dir, 'rviz', 'rviz2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=config_path),

        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            parameters=[{'config_path': config_path}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])

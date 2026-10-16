import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('efficient_online_segmentation')
    default_params = os.path.join(pkg_dir, 'config', 'gazebo_mid360_sep.yaml')

    params_file = LaunchConfiguration('params_file')

    ground_separation_node = Node(
        package='efficient_online_segmentation',
        executable='ground_separation_node',
        name='ground_separation',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ground_separation 节点参数 YAML。',
        ),
        ground_separation_node,
    ])

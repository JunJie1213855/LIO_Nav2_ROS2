from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = join(get_package_share_directory('dddmr_ros2'), 'config', 'planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('map_topic', default_value='/global_points'),
        DeclareLaunchArgument('path_topic', default_value='/init_path'),
        Node(package='dddmr_ros2', executable='planner_node', name='dddmr_planner',
             output='screen', parameters=[LaunchConfiguration('params_file')],
             remappings=[('/global_points', LaunchConfiguration('map_topic')),
                         ('/init_path', LaunchConfiguration('path_topic'))]),
    ])

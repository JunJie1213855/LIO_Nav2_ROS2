"""Cartographer 2D 在线 SLAM 启动文件（2D 单线 LiDAR 差分小车）

依赖: /scan (LaserScan), /imu (Imu), /odom (diff_drive 里程计)
Cartographer 发布 /map 和 map→odom TF。

用法:
  ros2 launch nav2_planner_bringup cartographer_2d_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_planner_bringup')
    config_dir = os.path.join(pkg_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_2d.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),
            ('odom', '/odom'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        cartographer_node,
        occupancy_grid_node,
    ])

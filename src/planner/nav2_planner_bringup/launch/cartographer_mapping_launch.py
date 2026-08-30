"""Cartographer 2D 在线建图启动文件

替代 slam_toolbox，与 FAST-LIO 里程计配合使用。

数据流:
  /scan (LaserScan, livox_frame)  ─┐
  /livox/imu (Imu)                 ─┤→ cartographer_node → /map + map→odom TF
  /odom (Odometry, 运动先验)       ─┘

用法:
  ros2 launch nav2_planner_bringup cartographer_mapping_launch.py
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
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Cartographer 核心 SLAM 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_mapping.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/livox/imu'),
            ('odom', '/odom'),
        ],
    )

    # OccupancyGrid 发布节点 (将 submap 转换为 /map 话题)
    # Nav2 的 global_costmap 订阅此话题
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

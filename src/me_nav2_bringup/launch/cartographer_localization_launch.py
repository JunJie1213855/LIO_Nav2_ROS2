"""Cartographer 2D 纯定位启动文件

加载已建好的地图 (.pbstream)，只做定位不建图。
用于替代 KISS-Matcher 重定位 → 发布 map→odom TF。

与 map_server 配合使用:
  - map_server: 加载静态 .pgm 地图 → 发布 /map 话题
  - Cartographer: 加载 .pbstream → 发布 map→odom TF (重定位)

数据流:
  /scan (LaserScan)  ─┐
  /livox/imu (Imu)    ─┤→ cartographer_node → map→odom TF
  /odom (运动先验)     ─┘

用法:
  ros2 launch me_nav2_bringup cartographer_localization_launch.py \
      load_state_filename:=/path/to/map.pbstream
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('me_nav2_bringup')
    config_dir = os.path.join(pkg_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    load_state_filename = LaunchConfiguration('load_state_filename')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_load_state_filename = DeclareLaunchArgument(
        'load_state_filename',
        default_value=os.path.join(pkg_dir, 'map', 'map.pbstream'),
        description='Path to the saved .pbstream map file')

    # Cartographer 纯定位节点
    # 注意: 不发布 /map (由 map_server 负责)，只发布 map→odom TF
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_localization.lua',
            '-load_state_filename', load_state_filename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/livox/imu'),
            ('odom', '/odom'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_load_state_filename,
        cartographer_node,
    ])

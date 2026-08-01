"""Cartographer 2D 精简建图/定位启动文件

无外部里程计，Cartographer 自己做 scan matching。
需要: /scan (LaserScan), /livox/imu (Imu), TF: base_footprint→livox_frame (URDF 静态)

建图用法:
  ros2 launch me_nav2_bringup cartographer_simple_launch.py

定位用法:
  ros2 launch me_nav2_bringup cartographer_simple_launch.py \
      pure_localization:=true load_pbstream:=/ws/src/me_nav2_bringup/map/map.pbstream
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('me_nav2_bringup')
    config_dir = os.path.join(pkg_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pure_localization = LaunchConfiguration('pure_localization', default='false')
    load_pbstream = LaunchConfiguration('load_pbstream', default='')
    use_rviz = LaunchConfiguration('rviz', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_pure_localization = DeclareLaunchArgument(
        'pure_localization', default_value='false')
    declare_load_pbstream = DeclareLaunchArgument(
        'load_pbstream', default_value='')
    declare_use_rviz = DeclareLaunchArgument(
        'rviz', default_value='false')

    # Cartographer 节点参数
    carto_args = [
        '-configuration_directory', config_dir,
        '-configuration_basename', 'cartographer_simple.lua',
    ]
    # 纯定位模式: 加载 pbstream
    carto_args_conditional = []
    # 注意: 不能同时传 conditional 和 unconditional，用 PythonExpression 处理

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=carto_args,
        remappings=[
            ('scan', '/scan'),
            ('imu', '/livox/imu'),
        ],
    )

    # OccupancyGrid 发布（建图模式需要）
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

    # TF → Odometry 桥接（Nav2 需要 /odom 话题）
    tf_to_odom_node = Node(
        package='me_nav2_bringup',
        executable='tf_to_odom.py',
        name='tf_to_odometry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz
    rviz_config = os.path.join(pkg_dir, 'rviz', 'cartographer_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_pure_localization,
        declare_load_pbstream,
        declare_use_rviz,
        cartographer_node,
        occupancy_grid_node,
        tf_to_odom_node,
        rviz_node,
    ])

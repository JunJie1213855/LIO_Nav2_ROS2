"""Nav2 在线 SLAM 导航启动文件（配合 Cartographer 2D SLAM）

不启动 map_server / AMCL —— 全局代价地图的 static_layer 直接订阅
Cartographer occupancy_grid_node 发布的 /map 话题。

依赖: /map (Cartographer), /scan, /odom, map→odom TF (Cartographer)

用法:
  ros2 launch nav2_planner nav2_online_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    me_share_path = get_package_share_directory('nav2_planner')
    params_file = os.path.join(me_share_path, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'bt_navigator',
        'behavior_server',
    ]

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
    ])

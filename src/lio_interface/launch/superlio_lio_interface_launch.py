import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    lio_interface_node = Node(
        package='lio_interface',
        executable='lio_interface_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odometry_sub': '/lio/odom',          # ← Super-LIO 的里程计话题
        }],
        remappings=[
            ('/cloud_registered', LaunchConfiguration('cloud_topic')),
            # 默认 /cloud_registered → 重映射到 Super-LIO 的 /lio/cloud_world
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='是否使用仿真时间'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/lio/cloud_world',
            description='Super-LIO 输出点云 topic'),
        lio_interface_node,
    ])

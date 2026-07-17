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
        emulate_tty=True,  # 开启提示颜色
        parameters=[{
            'use_sim_time': True,
            'odometry_sub': '/Odometry',
        }],
        remappings=[
            ('/cloud_registered', LaunchConfiguration('cloud_topic')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic', default_value='/cloud_registered',
            description='FAST-LIO 输出点云 topic'),
        lio_interface_node,
    ])

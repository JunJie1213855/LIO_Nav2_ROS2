"""3D→2D 切片 — 精简版（无需 LIO 里程计）

直接从 Gazebo 仿真 /livox/lidar 转成 /scan。
与 pointcloud_to_laserscan_launch.py 的区别：cloud_in 直接订阅 /livox/lidar 而非 /registered_scan。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('me_nav2_bringup'),
        'config',
        'Pointcloud2d_3d.yaml'
    )

    pc2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='Pointcloud2d_3d',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('cloud_in', '/livox/lidar'),   # 直接拿 Gazebo 仿真原始点云
            ('scan', '/scan'),
        ]
    )

    return LaunchDescription([pc2l_node])

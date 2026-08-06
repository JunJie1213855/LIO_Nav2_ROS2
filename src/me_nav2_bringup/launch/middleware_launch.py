"""
中间层合并 launch：lio_interface + sensor_scan_generation + pointcloud_to_laserscan
三个 C++ 节点共用一个 ros2 launch 进程，省下 2 个 Python 解释器 (~72MB)。

数据流:
  /cloud_registered → lio_interface → /registered_scan → sensor_scan_generation
    → odom→base_footprint TF → pointcloud_to_laserscan → /scan

用法:
  ros2 launch me_nav2_bringup middleware_launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")

    zlim_config = os.path.join(
        get_package_share_directory("me_nav2_bringup"),
        "config", "Pointcloud2d_3d_zlim.yaml")

    # 1. lio_interface: LIO 世界帧 → odom 帧
    lio_interface_node = Node(
        package="lio_interface",
        executable="lio_interface_node",
        name="lio_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": use_sim_time,
            "odometry_sub": "/Odometry",
        }],
        remappings=[("/cloud_registered", cloud_topic)],
    )

    # 2. sensor_scan_generation: 发布 odom→base_footprint TF + /odom
    sensor_scan_node = Node(
        package="sensor_scan_generation",
        executable="sensor_scan_generation_node",
        name="sensor_scan_generation",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": use_sim_time,
            "lidar_frame": lidar_frame,
            "base_footprint_frame": base_frame,
        }],
    )

    # 3. pointcloud_to_laserscan: 3D→2D 切片（zlim 负值方案）
    pc2l_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="Pointcloud2d_3d",
        output="screen",
        parameters=[zlim_config],
        remappings=[
            ("cloud_in", "/registered_scan"),
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use /clock for simulated time"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered",
                              description="FAST-LIO output cloud topic"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame",
                              description="LiDAR TF frame name"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint",
                              description="Robot base TF frame name"),
        lio_interface_node,
        sensor_scan_node,
        pc2l_node,
    ])

"""
中间层合并 launch：lio_interface + sensor_scan_generation + pointcloud_to_laserscan
三个 C++ 节点共用一个 ros2 launch 进程，省下 2 个 Python 解释器 (~72MB)。

数据流:
  /cloud_registered → lio_interface → /registered_scan → sensor_scan_generation
    → odom→base_footprint TF → pointcloud_to_laserscan → /scan

用法:
  # FAST-LIO (默认，zlim 负值切片)
  ros2 launch nav2_planner_bringup middleware_launch.py use_sim_time:=true

  # Point-LIO (重力对齐，云 Z 朝上；但切片在 base_footprint 机体帧内做，负值)
  ros2 launch nav2_planner_bringup middleware_launch.py use_sim_time:=False \
      odometry_sub:=/aft_mapped_to_init \
      pc2l_config:=Pointcloud2d_3d_zlim.yaml
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    odometry_sub = LaunchConfiguration("odometry_sub")
    pc2l_config = LaunchConfiguration("pc2l_config")

    pc2l_config_path = PathJoinSubstitution([
        get_package_share_directory("nav2_planner_bringup"),
        "config",
        pc2l_config,
    ])

    # 1. lio_interface: LIO 世界帧 → odom 帧
    lio_interface_node = Node(
        package="lio_interface",
        executable="lio_interface_node",
        name="lio_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": use_sim_time,
            "odometry_sub": odometry_sub,
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

    # 3. pointcloud_to_laserscan: 3D→2D 切片（默认 zlim 负值，可切换正值）
    pc2l_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="Pointcloud2d_3d",
        output="screen",
        parameters=[pc2l_config_path],
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
        DeclareLaunchArgument("odometry_sub", default_value="/Odometry",
                              description="LIO odometry topic (/Odometry 或 /aft_mapped_to_init)"),
        DeclareLaunchArgument("pc2l_config", default_value="Pointcloud2d_3d_zlim.yaml",
                              description="3D→2D 切片配置文件名（位于 nav2_planner_bringup/config 下）"),
        lio_interface_node,
        sensor_scan_node,
        pc2l_node,
    ])

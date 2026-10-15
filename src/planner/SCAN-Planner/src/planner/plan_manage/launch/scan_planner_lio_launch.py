"""SCAN-Planner + LIO 一体化启动文件

将 SCAN-Planner 与 FAST-LIO 里程计管线整合：
  - 建图：SCAN-Planner 局部 ESDF 占据栅格 + FAST-LIO 全局 PCD 地图
  - 定位：FAST-LIO 3D 里程计 → lio_interface → SCAN-Planner body_pose
  - 导航：SCAN-Planner 局部避障轨迹规划 → /cmd_vel

用法:
  ros2 launch scan_planner scan_planner_lio_launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # SCAN-Planner 核心规划节点
    # 将 LIO 管线话题映射到 SCAN-Planner 的输入
    scan_planner_node = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[planner_yaml],
        remappings=[
            # LIO 管线 → SCAN-Planner 输入
            ("body_pose", "/odom"),                       # odom→base_footprint (sensor_scan_generation)
            ("sensor_pose", "/odom"),                     # IMU≈body, 共用
            ("cloud", "/registered_scan"),                # lio 点云
        ],
    )

    # 闭环控制器: BSpline 轨迹 → /cmd_vel
    closed_loop_controller = Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        output="screen",
        parameters=[controllers_yaml],
        remappings=[
            ("body_pose", "/odom"),
            ("cmd_vel", "/cmd_vel"),
        ],
    )

    # SCAN-Planner 可视化硬编码 "world" 帧, 发布 world→odom identity TF 桥接
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        world_tf,
        scan_planner_node,
        closed_loop_controller,
    ])

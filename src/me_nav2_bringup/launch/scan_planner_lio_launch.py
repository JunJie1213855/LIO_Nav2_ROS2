"""SCAN-Planner + LIO 一体化启动文件

将 SCAN-Planner 与 FAST-LIO 里程计管线整合：
  - 建图：SCAN-Planner 局部 ESDF 占据栅格 + FAST-LIO 全局 PCD 地图
  - 定位：FAST-LIO 3D 里程计 → lio_interface → SCAN-Planner body_pose
  - 导航：SCAN-Planner 局部避障轨迹规划 → /cmd_vel

用法:
  ros2 launch me_nav2_bringup scan_planner_lio_launch.py use_sim_time:=true
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

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    navi_mode = LaunchConfiguration("navi_mode", default="1")
    use_pcd_map = LaunchConfiguration("use_pcd_map", default="false")
    pcd_map_file = LaunchConfiguration("pcd_map_file", default="")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false")
    declare_navi_mode = DeclareLaunchArgument(
        "navi_mode", default_value="1",
        description="1=Rviz2DGoal, 2=waypoints, 3=initial_path")
    declare_use_pcd_map = DeclareLaunchArgument(
        "use_pcd_map", default_value="false")
    declare_pcd_map_file = DeclareLaunchArgument(
        "pcd_map_file", default_value="")

    # Z 轴过滤节点: 去除天花板和地面，只保留机器人高度范围的点云
    z_filter = Node(
        package="me_nav2_bringup",
        executable="cloud_z_filter.py",
        name="cloud_z_filter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "z_min": -1.0,
            "z_max": 2.0,
        }],
        remappings=[
            ("cloud_in", "/registered_scan"),
            ("cloud_out", "/registered_scan_filtered"),
        ],
    )

    # SCAN-Planner 核心规划节点
    # 将 LIO 管线话题映射到 SCAN-Planner 的输入
    scan_planner_node = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[planner_yaml, {
            "use_sim_time": use_sim_time,
            "fsm.navi_mode": navi_mode,
            # LIO 管线: 点云已经在 odom 坐标系，不需要额外外参变换
            "grid_map.cloud_is_world": True,
            "grid_map.need_extrinsic": False,
            "grid_map.frame_id": "odom",
            # PCD 全局地图（可选）
            "grid_map.use_pcd_map": use_pcd_map,
            "grid_map.pcd_map_file": pcd_map_file,
        }],
        remappings=[
            # LIO 管线 → SCAN-Planner 输入
            ("body_pose", "/odom"),                       # odom→base_footprint (sensor_scan_generation)
            ("sensor_pose", "/odom"),                      # IMU≈body, 共用
            ("cloud", "/registered_scan_filtered"),        # Z 轴过滤后的点云
        ],
    )

    # 闭环控制器: BSpline 轨迹 → /cmd_vel
    closed_loop_controller = Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        output="screen",
        parameters=[controllers_yaml, {
            "use_sim_time": use_sim_time,
        }],
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
        declare_use_sim_time,
        declare_navi_mode,
        declare_use_pcd_map,
        declare_pcd_map_file,
        world_tf,
        z_filter,
        scan_planner_node,
        closed_loop_controller,
    ])

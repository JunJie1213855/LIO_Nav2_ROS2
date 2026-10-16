"""Gazebo + FAST-LIO + FAR Planner 联合仿真 launch 文件

管线:
  Gazebo → FAST-LIO → lio_interface → sensor_scan_generation
         → FAR Planner（可见图全局规划）→ far_local_planner（VFH 局部避障）→ /cmd_vel

用法:
  ros2 launch me_nav2_bringup far_planner_lio_launch.py use_sim_time:=true

说明:
  - FAR Planner 的 world_frame 是 "map"，而 LIO 管线用 "odom"，
    这里加了一个 map→odom 的 identity TF 桥接。
  - odom 和 /registered_scan 都是 odom 帧，FAR 内部会用 TF 转到 map 帧。
  - far_local_planner：VFH 间隙跟随局部规划器 + 启动原地旋转建图，
    订阅 /way_point + /odom + /registered_scan + /navigation_boundary。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    far_share = get_package_share_directory("far_planner")
    config_yaml = os.path.join(far_share, "config", "default.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")

    # FAR Planner 全局规划节点（可见图）
    # robot_dim 已按 simple_car 调小为 0.4（原 0.8 为 CMU 大车值）。
    far_node = Node(
        package="far_planner",
        executable="far_planner",
        name="far_planner",
        output="screen",
        parameters=[config_yaml, {
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            ("/odom_world", "/odom"),                     # 里程计 (odom 帧)
            ("/terrain_cloud", "/terrain_map"),           # 地形点云（主输入，intensity=高度）
            ("/terrain_local_cloud", "/registered_scan"),  # 局部点云（动态障碍用，静态环境不启用）
            ("/scan_cloud", "/registered_scan"),          # 扫描点云（动态障碍用）
        ],
    )

    # LIO 点云 → FAR terrain cloud（intensity 重写为高度 z）
    terrain_cloud_gen = Node(
        package="nav2_planner_bringup",
        executable="terrain_cloud_generator.py",
        name="terrain_cloud_generator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # far_local_planner: /way_point → /cmd_vel（VFH 间隙跟随 + 启动旋转建图）
    local_planner = Node(
        package="nav2_planner_bringup",
        executable="far_local_planner.py",
        name="far_local_planner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_linear_vel": 0.5,
            "max_angular_vel": 1.0,
            "arrival_dist": 0.3,
            "kp_linear": 0.8,
            "kp_angular": 2.0,
            "obstacle_range": 2.5,
            "robot_radius": 0.35,
            "safe_margin": 0.25,
            "stop_dist": 0.50,
            "slow_dist": 1.20,
            "z_min": 0.05,
            "z_max": 2.0,
            "n_bins": 72,
            "boundary_sample_step": 0.15,
            "boundary_range": 3.0,
            "init_rotate_enable": True,
            "init_rotate_vel": 0.4,
            "init_rotate_duration": 18.0,
            "startup_delay": 6.0,
            "control_rate": 10.0,
        }],
    )

    # map → odom identity TF（FAR 的 world_frame 是 map，桥接到 LIO 的 odom 帧）
    # 注意：静态 TF 不加 use_sim_time，否则启动时 /clock 未就绪会以时间戳 0 发布，
    #       导致 FAR 查不到 map→odom，里程计/点云初始化失败。
    map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    return LaunchDescription([
        declare_use_sim_time,
        map_tf,
        terrain_cloud_gen,
        far_node,
        local_planner,
    ])
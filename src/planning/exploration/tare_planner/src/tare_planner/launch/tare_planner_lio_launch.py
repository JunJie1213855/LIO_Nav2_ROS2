"""Gazebo + FAST-LIO + TARE Planner 联合仿真 launch 文件

管线:
  Gazebo → FAST-LIO (里程计+点云) → lio_interface → sensor_scan_generation
         → TARE 探索规划 → waypoint_follower(带局部避障) → /cmd_vel

用法:
  ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tare_share = get_package_share_directory("tare_planner")
    config_yaml = os.path.join(tare_share, "gazebo_indoor.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")

    # TARE 直接使用 sensor_scan_generation 发布的 odom 帧点云
    # 无需 cloud_z_filter（其只支持世界/body 帧的 Z 轴过滤）

    # TARE 探索规划节点
    tare_node = Node(
        package="tare_planner",
        executable="tare_planner_node",
        name="tare_planner_node",
        output="screen",
        parameters=[config_yaml, {
            "use_sim_time": use_sim_time,
        }],
    )

    # waypoint_follower: /way_point → /cmd_vel
    waypoint_follower = Node(
        package="tare_planner",
        executable="waypoint_follower.py",
        name="waypoint_follower",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_linear_vel": 0.5,
            "max_angular_vel": 1.0,
            "arrival_dist": 0.3,
            "kp_linear": 0.8,
            "kp_angular": 2.0,
            "stop_dist": 0.45,
            "slow_dist": 0.8,
            "robot_half_width": 0.35,
            "check_height_min": 0.05,
            "check_height_max": 0.60,
        }],
    )

    # world → odom identity TF (TARE 可视化用 world 帧)
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        world_tf,
        tare_node,
        waypoint_follower,
    ])
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    body_pose = LaunchConfiguration("body_pose_topic")
    sensor_pose = LaunchConfiguration("sensor_pose_topic")
    cloud = LaunchConfiguration("cloud_topic")
    use_car = PythonExpression(["'", LaunchConfiguration("controller_type"), "' == 'car'"])

    # SCAN-Planner 核心规划节点
    # 将 LIO 管线话题映射到 SCAN-Planner 的输入
    scan_planner_node = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[planner_yaml, {
            "use_sim_time": use_sim_time,
            "grid_map.sensor_type": "lidar",
            "grid_map.cloud_is_world": ParameterValue(
                LaunchConfiguration("cloud_is_world"), value_type=bool),
            "grid_map.need_extrinsic": False,
            "fsm.use_controller_progress": ParameterValue(use_car, value_type=bool),
        }],
        remappings=[
            # LIO 管线 → SCAN-Planner 输入
            ("body_pose", body_pose),
            ("sensor_pose", sensor_pose),
            ("cloud", cloud),
        ],
    )

    # 闭环控制器: BSpline 轨迹 → /cmd_vel
    closed_loop_controller = Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("controller_type"), "' == 'legacy'"])),
        output="screen",
        parameters=[controllers_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
            ("body_pose", body_pose),
            ("cmd_vel", "/cmd_vel"),
        ],
    )

    car_controller = Node(
        package="scan_planner",
        executable="closed_loop_controller_car",
        name="closed_loop_controller_car",
        condition=IfCondition(use_car),
        output="screen",
        parameters=[controllers_yaml, {"use_sim_time": use_sim_time}],
        remappings=[("body_pose", body_pose), ("cmd_vel", "/cmd_vel")],
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
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("controller_type", default_value="legacy", choices=["legacy", "car"],
                              description="car: 差速小车空间路径跟踪；legacy: 原闭环控制器"),
        DeclareLaunchArgument("body_pose_topic", default_value="/odom",
                              description="车体位姿，odom -> base_footprint"),
        DeclareLaunchArgument("sensor_pose_topic", default_value="/odom",
                              description="雷达位姿；传感器系点云应使用 /registered_odometry"),
        DeclareLaunchArgument("cloud_topic", default_value="/registered_scan"),
        DeclareLaunchArgument("cloud_is_world", default_value="true",
                              description="false: 用 sensor_pose 将传感器系点云转换到规划坐标系"),
        world_tf,
        scan_planner_node,
        closed_loop_controller,
        car_controller,
    ])

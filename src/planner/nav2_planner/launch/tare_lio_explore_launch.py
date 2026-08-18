"""TARE-Planner + FAST-LIO 自主探索一体化启动文件

管线:
  Gazebo → FAST-LIO → lio_interface → sensor_scan_generation
                                            ↓
                                       TARE-Planner → /way_point
                                            ↓
                                   waypoint_follower → /cmd_vel → 机器人

用法:
  ros2 launch nav2_planner tare_lio_explore_launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tare_share = get_package_share_directory("tare_planner")
    garage_yaml = os.path.join(tare_share, "garage.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    # map→odom identity TF: TARE waypoints 在 map 帧，需桥接到 odom
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # TARE-Planner 节点（不自动启动，等数据就绪后再触发）
    tare_node = Node(
        package="tare_planner",
        executable="tare_planner_node",
        name="tare_planner_node",
        output="screen",
        parameters=[garage_yaml, {
            "use_sim_time": use_sim_time,
            "kUseTerrainHeight": False,
            "kCheckTerrainCollision": False,
            "kAutoStart": False,           # 等管线稳定后再手动/定时触发
            "kSensorRange": 15.0,
            "kRushHome": False,            # 不回家
        }],
        remappings=[
            ("/registered_scan", "/registered_scan"),
            ("/state_estimation_at_scan", "/odom"),
        ],
    )

    # Waypoint Follower: /way_point → /cmd_vel（降低速度防撞墙）
    waypoint_follower = Node(
        package="nav2_planner",
        executable="waypoint_follower.py",
        name="waypoint_follower",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_linear_vel": 0.5,
            "max_angular_vel": 0.8,
            "goal_tolerance": 0.3,
        }],
    )

    # TARE RViz
    tare_rviz_config = os.path.join(tare_share, "tare_planner_ground.rviz")
    tare_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_tare",
        output="screen",
        arguments=["-d", tare_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 延迟 15 秒触发探索（等 LIO 管线和点云积累稳定）
    start_explore_trigger = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'sleep 1 && source /ws/install/setup.bash && '
                     'ros2 topic pub -1 /start_exploration std_msgs/msg/Bool "data: true"'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        map_to_odom_tf,
        tare_node,
        waypoint_follower,
        tare_rviz,
        start_explore_trigger,
    ])

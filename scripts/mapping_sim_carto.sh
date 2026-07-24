#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# ============================================================================
# 仿真建图启动脚本 — Cartographer 版
#
# 与 mapping_sim.sh 的区别:
#   - 用 Cartographer 替代 slam_toolbox
#   - 不需要 KISS-Matcher (Cartographer 自己发布 map→odom)
#   - Cartographer 自带回环检测
# ============================================================================

# GUI 遥控
gnome-terminal --title="GUI控制" -- bash -c "
source install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

# ----------------------------------------------------------------------------
# FAST-LIO 里程计
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py"

# 里程计接口 (TF 桥接)
gnome-terminal --title="lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

# ----------------------------------------------------------------------------
# Gazebo 仿真环境
gnome-terminal --title="Gazebo 仿真" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch get_urdf get_urdf_launch.py"

# 点云组装 + 里程计发布
gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 3D 点云转 2D LaserScan
gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"

# ----------------------------------------------------------------------------
# Cartographer 2D 在线建图 (替代 slam_toolbox)
# 订阅: /scan + /livox/imu + /odom
# 发布: /map (OccupancyGrid) + map→odom TF
gnome-terminal --title="Cartographer 建图" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup cartographer_mapping_launch.py"

# ============================================================================
# 建图完成后，保存地图:
#
#   1. 结束轨迹 (不再接收新数据):
#      ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
#
#   2. 导出 .pbstream → .pgm + .yaml:
#      ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
#          -pbstream_filename <保存路径>.pbstream \
#          -map_filestem <保存路径>
#
#   3. 或者用 map_saver 直接保存 /map 话题:
#      ros2 run nav2_map_server map_saver_cli -f <地图名>
# ============================================================================

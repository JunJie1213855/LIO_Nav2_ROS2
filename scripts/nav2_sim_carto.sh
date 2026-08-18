#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# ============================================================================
# 仿真导航启动脚本 — Cartographer 纯定位版
#
# 与 nav2_sim.sh 的区别:
#   - 用 Cartographer 纯定位替代 KISS-Matcher 重定位
#   - Cartographer 加载 .pbstream 地图，发布 map→odom TF
#   - map_server 加载 .pgm 静态地图 (从 pbstream 导出)
# ============================================================================

# 使用说明:
#   1. 先用 mapping_sim_carto.sh 建图并保存
#   2. 将 pbstream 导出为 pgm: ros2 run cartographer_ros cartographer_pbstream_to_ros_map ...
#   3. 修改下方 MAP_PB 和 MAP_PGM 路径
#   4. 运行此脚本

# GUI 遥控
gnome-terminal --title="GUI控制" -- bash -c "
source install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

# ----------------------------------------------------------------------------
# FAST-LIO 里程计
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py"

# 里程计接口
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
ros2 launch nav2_planner pointcloud_to_laserscan_launch.py"

# ----------------------------------------------------------------------------
# Cartographer 纯定位 (替代 KISS-Matcher)
# 加载 .pbstream 地图，只做定位不建图
# 发布: map→odom TF (重定位)
gnome-terminal --title="Cartographer 定位" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner cartographer_localization_launch.py \
    load_state_filename:=src/planner/nav2_planner/map/map.pbstream"

# ----------------------------------------------------------------------------
# Nav2 导航栈
# map_server 在 my_nav2_launch.py 中加载 .pgm 地图
# 注意: 需要确保 map→odom TF 由 Cartographer 发布后 Nav2 才能正常工作
gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner my_nav2_launch.py"

# ============================================================================
# 注意事项:
#   1. map.pbstream 需要提前准备好 (从建图阶段导出)
#   2. Nav2 的 map_server 加载 .pgm (从 pbstream 转换)
#   3. Cartographer 纯定位需要几秒钟初始化，等终端打印定位成功后
#       在 RViz 中给定 2D Pose Estimate 即可开始导航
#   4. 如果 Cartographer 脱离定位，在 RViz 中重新给初始位姿
# ============================================================================

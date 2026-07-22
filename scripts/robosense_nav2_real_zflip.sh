#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 实机导航启动脚本

# ========== robosense lidar SDK ==========
gnome-terminal --title="robosense lidar SDK" -- bash -c "
source install/setup.bash;
ros2 launch rslidar_sdk driver_only.launch.py"

sleep 3

# ========== 建图 fast lio ==========
# fast_lio
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py"

# lio_interface
gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface fastlio_lio_interface_launch.py use_sim_time:=False"

# ---------

# ========== 中间层 ==========
gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch gld_robot_description gld_robot_description_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch_zlim.py"


# ========== 重定位 ==========
gnome-terminal --title="small_gicp 重定位" -- bash -c "
source install/setup.bash;
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py"

# gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
# source install/setup.bash;
# ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py"


# ========== nav2 导航 ==========
gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup my_nav2_launch.py"
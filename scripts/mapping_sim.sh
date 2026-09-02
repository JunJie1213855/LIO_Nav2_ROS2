#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 仿真建图启动脚本
# 可能会导致 /cmd_vel 话题被占用
# gui控制小车
gnome-terminal --title="GUI控制" -- bash -c "
source install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

# Gazebo 仿真环境
gnome-terminal --title="Gazebo 仿真" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch get_urdf get_urdf_launch.py"

# -----------------------------------------------------------------------------------
# 使用fast-lio作为里程计
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py"

# 里程计接口
gnome-terminal --title="lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

gnome-terminal --title="slam_toolbox 建图" -- bash -c "
source install/setup.bash;
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup my_nav2_launch.py"

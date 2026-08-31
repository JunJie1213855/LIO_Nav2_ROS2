#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1


SESSION=mapping_sim

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

new_win() { # new_win <窗口名> <命令>
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}
# 仿真建图启动脚本

# 杀死之前的 Gazebo 进程
# killall -9 gzserver gzclient

# 键盘控制
# ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 可能会导致 /cmd_vel 话题被占用
# gui控制小车
tmux new-session -d -s "$SESSION" -n "GUI控制" "bash -c 'source install/setup.bash; ros2 run gui_teleop gui_teleop_node; exec bash'"

# -----------------------------------------------------------------------------------
# 使用fast-lio作为里程计
new_win "FAST-LIO 里程计" "ros2 launch fast_lio_robosense mapping.launch.py"

# 里程计接口
new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py"

# ------------------------------------------------------------------------------------

# 使用point-lio作为里程计
# new_win "点云格式转换器"  "
# source install/setup.bash;
# ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args \
#   -p pcd_topic:=/livox/lidar \
#   -p n_scan:=50 \
#   -p horizon_scan:=360 \
#   -p ang_bottom:=7.22 \
#   -p ang_res_y:=1.248"

# new_win "Point-LIO 里程计"  "
# source install/setup.bash;
# ros2 launch point_lio point_lio.launch.py \
#   point_lio_cfg_dir:=/home/pio/Nav2_3D_ws/src/localization/point_lio/config/mid360_sim.yaml"

# new_win "lio_interface"  "
# source install/setup.bash;
# ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio"

# ------------------------------------------------------------------------------------

# Gazebo 仿真环境
# new_win "Gazebo 仿真"  "
# killall -9 gzserver gzclient;
# source install/setup.bash;
# ros2 launch get_urdf get_urdf_launch.py"

new_win "Gazebo 仿真"  "killall -9 gzserver gzclient; source install/setup.bash; ros2 launch get_urdf get_urdf_launch.py"

# 中间层

new_win "sensor_scan_generation"  "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 中间层， 3D点云转2D

new_win "3d点云转2d"  "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

# new_win "slam_toolbox 建图"  "
# source install/setup.bash;
# ros2 launch slam_toolbox online_async_launch.py"

# slam toolbox 建图，2D代价地图
new_win "slam_toolbox 建图"  "ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

# new_win "Nav2 导航"  "
# source install/setup.bash;
# ros2 launch nav2_planner_bringup my_nav2_launch.py"

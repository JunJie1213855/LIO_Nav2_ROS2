#!/usr/bin/env bash
# Gazebo + FAST-LIO + SCAN-Planner 联合仿真
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=scan_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  sensor_scan_generation_node scan_planner_node \
  closed_loop_controller closed_loop_controller_car 2>/dev/null

tmux kill-session -t "$SESS" 2>/dev/null
# ================ 关闭相关进程 ================
W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# ================ Gazebo ================
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"


# ================ fast-lio2 ================
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py rviz:=false use_sim_time:=true"


# ================ 中间层 ================
W "lio_if"    "ros2 launch lio_interface lio_interface_launch.py"
W "sensor"    "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ================ scan planner ================
# SCAN-Planner 直接使用 LIO 输出的 /registered_scan 点云。
W "SCAN"      "ros2 launch scan_planner scan_planner_lio_launch.py \
               controller_type:=car cloud_is_world:=true"

# ================ scan planner 可视化================
W "SP-RViz"   "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_scan -p use_sim_time:=true -- -d $WS/src/bringup/nav2_planner_bringup/rviz/scan_planner.rviz"

echo "========================================="
echo " Gazebo + FAST-LIO + SCAN-Planner"
echo " 窗口: Gazebo | FAST-LIO | lio_if | sensor | SCAN | SP-RViz"
echo " attach: tmux attach -t $SESS"
echo ""
echo " 操作: 切到 SP-RViz 窗口, 用 '2D Goal Pose' 点目标"
echo "========================================="

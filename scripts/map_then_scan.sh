#!/usr/bin/env bash
# 步骤1: 手动遥控建图 → 保存 PCD
# 步骤2: SCAN-Planner 加载 PCD 地图做全局规划

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=map_scan
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
MAP_FILE="$WS/src/planner/nav2_planner_bringup/pcd/test_world.pcd"

killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  sensor_scan_generation_node scan_planner_node \
  closed_loop_controller cloud_z_filter 2>/dev/null
sleep 2
tmux kill-session -t "$SESS" 2>/dev/null

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"
sleep 6

W "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py rviz:=false"
sleep 3
W "lio_if"    "ros2 launch lio_interface lio_interface_launch.py"
W "sensor"    "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
sleep 2

# SCAN-Planner 加载 PCD 地图，不再依赖局部滑动窗口
W "SCAN"      "ros2 launch nav2_planner_bringup scan_planner_lio_launch.py \
    z_min:=0.15 z_max:=3.0 \
    double_cylinder_radius:=0.30 double_cylinder_offset:=0.0 \
    body_height:=0.25 obstacles_inflation_z_down:=0.1 \
    optimization.lambda_collision:=10.0 optimization.dist0:=0.2 \
    grid_map.sliding_map_size_x:=50.0 grid_map.sliding_map_size_y:=50.0 \
    grid_map.local_update_range_x:=25.0 grid_map.local_update_range_y:=25.0 \
    use_pcd_map:=true pcd_map_file:=$MAP_FILE"

W "GUI"       "ros2 run gui_teleop gui_teleop_node"
W "SP-RViz"   "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_scan -p use_sim_time:=true -- -d /ws/src/planner/nav2_planner_bringup/rviz/scan_planner.rviz"

echo "========================================="
echo " 步骤1: 用 GUI 遥控小车在环境里走一圈建图"
echo " 步骤2: 保存 PCD: docker exec lio_nav2 bash -c 'source /ws/install/setup.bash && ros2 service call /map_save std_srvs/srv/Trigger'"
echo " 步骤3: cp PCD到 $MAP_FILE"
echo " 步骤4: 重新运行本脚本，SCAN-Planner 加载完整地图"
echo "========================================="

#!/usr/bin/env bash
# Gazebo + FAST-LIO + SCAN-Planner 联合仿真
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=scan_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  sensor_scan_generation_node scan_planner_node \
  closed_loop_controller cloud_z_filter 2>/dev/null
sleep 2
tmux kill-session -t "$SESS" 2>/dev/null

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# Gazebo
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"
sleep 6

W "FAST-LIO" "ros2 launch fast_lio mapping.launch.py rviz:=false"
sleep 3
W "lio_if"    "ros2 launch lio_interface lio_interface_launch.py"
W "sensor"    "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
sleep 2
W "SCAN"      "ros2 launch me_nav2_bringup scan_planner_lio_launch.py \
    z_min:=0.15 z_max:=3.0 \
    double_cylinder_radius:=0.30 double_cylinder_offset:=0.0 \
    body_height:=0.25 obstacles_inflation_z_down:=0.1 \
    optimization.lambda_collision:=10.0 optimization.dist0:=0.2 \
    grid_map.sliding_map_size_x:=50.0 grid_map.sliding_map_size_y:=50.0 \
    grid_map.local_update_range_x:=25.0 grid_map.local_update_range_y:=25.0"
sleep 2
W "SP-RViz"   "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_scan -p use_sim_time:=true -- -d /ws/src/me_nav2_bringup/rviz/scan_planner.rviz"

echo "========================================="
echo " Gazebo + FAST-LIO + SCAN-Planner"
echo " 窗口: Gazebo | FAST-LIO | lio_if | sensor | SCAN | SP-RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " 操作: 切到 SP-RViz 窗口, 用 '2D Goal Pose' 点目标"
echo "========================================="

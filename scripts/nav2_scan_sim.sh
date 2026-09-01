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
# ================ 关闭相关进程 ================
W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# ================ Gazebo ================
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"
sleep 6

# ================ fast-lio2 ================
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py rviz:=false"
sleep 3

# ================ 中间层 ================
W "lio_if"    "ros2 launch lio_interface lio_interface_launch.py"
W "sensor"    "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
sleep 2

# ================ scan planner ================
# z_min:=0.3 z_max:=3.0 很重要，最好替换成地面分割算法，否则容易把当前目标的障碍物都丢失，然后路径变成一条直线
W "SCAN"      "ros2 launch nav2_planner_bringup scan_planner_lio_launch.py \
    z_min:=0.15 z_max:=3.0 \ 
    double_cylinder_radius:=0.45 double_cylinder_offset:=0.18 \
    body_height:=0.25 obstacles_inflation_z_down:=1.0 \
    optimization.lambda_collision:=50.0 optimization.dist0:=3.0 \
    grid_map.p_occ:=0.3 grid_map.p_hit:=0.9 grid_map.p_miss:=0.1 \
    grid_map.sliding_map_size_x:=50.0 grid_map.sliding_map_size_y:=50.0 \
    grid_map.local_update_range_x:=25.0 grid_map.local_update_range_y:=25.0 \
    use_pcd_map:=true pcd_map_file:=/ws/PCD/map.pcd"
sleep 2

# ================ scan planner 可视化================
W "SP-RViz"   "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_scan -p use_sim_time:=true -- -d /ws/src/planner/nav2_planner_bringup/rviz/scan_planner.rviz"

echo "========================================="
echo " Gazebo + FAST-LIO + SCAN-Planner"
echo " 窗口: Gazebo | FAST-LIO | lio_if | sensor | SCAN | SP-RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " 操作: 切到 SP-RViz 窗口, 用 '2D Goal Pose' 点目标"
echo "========================================="

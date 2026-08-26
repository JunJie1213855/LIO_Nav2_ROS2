#!/usr/bin/env bash
# Gazebo + FAST-LIO + TARE Planner 联合仿真（探索模式）
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=tare_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

killall -9 gzserver gzclient fastlio_mapping cloud_z_filter \
  tare_planner_node waypoint_follower lio_interface_node \
  sensor_scan_generation_node 2>/dev/null

tmux kill-session -t "$SESS" 2>/dev/null

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# Gazebo (含 indoor 世界)
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"


W "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py rviz:=false"

W "lio_if"   "ros2 launch lio_interface lio_interface_launch.py"
W "sensor"   "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

W "TARE"     "ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true"

W "RViz"     "rviz2 -d /ws/src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz"

echo "========================================="
echo " Gazebo + FAST-LIO + TARE Explorer"
echo " 窗口: Gazebo | FAST-LIO | lio_if | sensor | TARE | RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " TARE 会自动开始探索 (kAutoStart: true)"
echo " 观察 RViz 中的 exploration_path(绿) 和 way_point(红)"
echo "========================================="

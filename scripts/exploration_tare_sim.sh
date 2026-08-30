#!/usr/bin/env bash
# Gazebo + FAST-LIO + TARE-Planner 自主探索一键启动
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=tare
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 清理
killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  tare_planner_node sensor_scan_generation_node waypoint_follower 2>/dev/null
sleep 2
tmux kill-session -t "$SESS" 2>/dev/null

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# Gazebo
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"
sleep 6

W "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py"
sleep 3
W "lio_if"    "ros2 launch lio_interface lio_interface_launch.py"
W "sensor"    "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
sleep 2
W "TARE"      "ros2 launch nav2_planner_bringup tare_lio_explore_launch.py"

echo "========================================="
echo " Gazebo + FAST-LIO + TARE 自主探索"
echo " 15 秒后自动触发探索"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo "========================================="

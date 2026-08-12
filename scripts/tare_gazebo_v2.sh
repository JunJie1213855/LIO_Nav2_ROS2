#!/usr/bin/env bash
# Gazebo + FAST-LIO + TARE-Planner 自主探索 (Docker, Intel GPU)
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
SESSION=tare_gz

# GPU 加速
export LIBGL_ALWAYS_SOFTWARE=0
export OGRE_RTT_MODE=Copy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 清理残留
killall -9 gzserver gzclient fastlio_mapping lio_interface_node tare_planner_node sensor_scan_generation_node cloud_z_filter waypoint_follower 2>/dev/null
sleep 2

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# Gazebo (已降物理频率到 200Hz)
tmux new-session -d -s "$SESSION" -n "Gazebo" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"

sleep 6

new_win "FAST-LIO" "ros2 launch fast_lio mapping.launch.py"
sleep 3
new_win "lio_if"     "ros2 launch lio_interface lio_interface_launch.py"
sleep 1
new_win "sensor"     "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
sleep 1
new_win "TARE"       "ros2 launch me_nav2_bringup tare_lio_explore_launch.py"

echo "===== Gazebo + FAST-LIO + TARE (会话: $SESSION) ====="
echo "窗口: Gazebo | FAST-LIO | lio_if | sensor | TARE"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"

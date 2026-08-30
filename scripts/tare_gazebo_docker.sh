#!/usr/bin/env bash
# Docker 内 Gazebo + FAST-LIO + TARE-Planner 自主探索
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
SESSION=tare_gz
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# Gazebo: LiDAR→/livox/lidar, IMU→/livox/imu, 订阅/cmd_vel
tmux new-session -d -s "$SESSION" -n "Gazebo" \
  "bash -c 'killall -9 gzserver gzclient 2>/dev/null; cd $WORKSPACE_ROOT && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py; exec bash'"

sleep 5

new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py"
new_win "lio_if"     "ros2 launch lio_interface lio_interface_launch.py"
new_win "sensor"     "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
new_win "TARE"       "ros2 launch nav2_planner_bringup tare_lio_explore_launch.py"

echo "===== Gazebo + FAST-LIO + TARE (会话: $SESSION) ====="
echo "窗口: Gazebo | FAST-LIO | lio_if | sensor | TARE"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"

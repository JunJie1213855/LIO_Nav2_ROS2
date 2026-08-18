#!/usr/bin/env bash
# rosbag + FAST-LIO + TARE-Planner 测试 (无 GUI 遥控)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
SESSION=tare_bag
BAG_DIR="${1:-/dataset/robosense/robosenseAiry-slamtoolbox}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

tmux new-session -d -s "$SESSION" -n "bag" \
  "bash -c 'source /opt/ros/humble/setup.bash && ros2 bag play $BAG_DIR/robosenseAiry-slamtoolbox_0.db3 --clock; exec bash'"

new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py use_sim_time:=true rviz:=true"
new_win "robot_desc" "ros2 launch gld_robot_description robosenseAiry_description_launch.py rviz:=false"
new_win "lio_if" "ros2 launch lio_interface lio_interface_launch.py use_sim_time:=true"
new_win "sensor" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
new_win "TARE" "ros2 launch nav2_planner tare_lio_explore_launch.py use_sim_time:=true"

echo "===== rosbag + FAST-LIO + TARE (会话: $SESSION) ====="
echo "窗口: bag | FAST-LIO | robot_desc | lio_if | sensor | TARE"
echo ""
echo "TARE 窗口包含: tare_planner_node + waypoint_follower + RViz"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"

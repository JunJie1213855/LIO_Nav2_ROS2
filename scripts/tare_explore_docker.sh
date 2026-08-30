#!/usr/bin/env bash
# Docker 容器内 Gazebo + FAST-LIO + TARE-Planner 自主探索 (tmux)
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/tare_explore_docker.sh
#   docker exec -it lio_nav2 tmux attach -t tare_explore
#   docker exec lio_nav2 tmux kill-session -t tare_explore

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=tare_explore

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ── 窗口 1: Gazebo 仿真环境 ────────────────────────────────────
tmux new-session -d -s "$SESSION" -n "Gazebo" \
  "bash -c 'killall -9 gzserver gzclient 2>/dev/null; cd $WORKSPACE_ROOT && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py; exec bash'"

# 等 Gazebo 启动
sleep 5

# ── 窗口 2: FAST-LIO 里程计 ────────────────────────────────────
new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py"

# ── 窗口 3: lio_interface ──────────────────────────────────────
new_win "lio_if" "ros2 launch lio_interface lio_interface_launch.py"

# ── 窗口 4: sensor_scan_generation ─────────────────────────────
new_win "sensor" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ── 窗口 5: TARE 自主探索 + waypoint_follower + RViz ────────────
new_win "TARE" "ros2 launch nav2_planner_bringup tare_lio_explore_launch.py"

# ── 窗口 6: GUI 遥控 (可选干预) ────────────────────────────────
new_win "GUI遥控" "ros2 run gui_teleop gui_teleop_node"

echo "===== Gazebo + FAST-LIO + TARE 自主探索 (会话: $SESSION) ====="
echo "窗口: Gazebo | FAST-LIO | lio_if | sensor | TARE | GUI遥控"
echo ""
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"

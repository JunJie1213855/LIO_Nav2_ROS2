#!/usr/bin/env bash
# 仿真建图 - tmux 版 (2 窗口 × 4 pane)
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

SESSION="mapping_sim"
tmux kill-session -t "$SESSION" 2>/dev/null

source install/setup.bash

# ── Window 1: 核心传感器 ────────────────────────────────────────────
tmux new-session -d -s "$SESSION" -n "core"

# Pane 0: GUI 控制
tmux send-keys -t "$SESSION:core.0" \
  "source install/setup.bash && ros2 run gui_teleop gui_teleop_node" C-m
tmux select-pane -t "$SESSION:core.0" -T "GUI控制"

# Pane 1: FAST-LIO 里程计 (右)
tmux split-window -h -t "$SESSION:core"
tmux send-keys -t "$SESSION:core.1" \
  "source install/setup.bash && ros2 launch fast_lio mapping.launch.py" C-m
tmux select-pane -t "$SESSION:core.1" -T "FAST-LIO"

# Pane 2: lio_interface (下左)
tmux split-window -v -t "$SESSION:core.0"
tmux send-keys -t "$SESSION:core.2" \
  "source install/setup.bash && ros2 launch lio_interface lio_interface_launch.py" C-m
tmux select-pane -t "$SESSION:core.2" -T "lio_interface"

# Pane 3: Gazebo (下右)
tmux split-window -v -t "$SESSION:core.1"
tmux send-keys -t "$SESSION:core.3" \
  "killall -9 gzserver gzclient 2>/dev/null; source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py" C-m
tmux select-pane -t "$SESSION:core.3" -T "Gazebo"

# ── Window 2: 导航 ──────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "nav"

# Pane 0: sensor_scan
tmux send-keys -t "$SESSION:nav.0" \
  "source install/setup.bash && ros2 launch sensor_scan_generation sensor_scan_generation_launch.py" C-m
tmux select-pane -t "$SESSION:nav.0" -T "sensor_scan"

# Pane 1: 3D→2D (右)
tmux split-window -h -t "$SESSION:nav"
tmux send-keys -t "$SESSION:nav.1" \
  "source install/setup.bash && ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py" C-m
tmux select-pane -t "$SESSION:nav.1" -T "3d→2d"

# Pane 2: SLAM Toolbox (下左)
tmux split-window -v -t "$SESSION:nav.0"
tmux send-keys -t "$SESSION:nav.2" \
  "source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/me_nav2_bringup/config/slam_toolbox_params.yaml" C-m
tmux select-pane -t "$SESSION:nav.2" -T "slam_toolbox"

# Pane 3: Nav2 (下右)
tmux split-window -v -t "$SESSION:nav.1"
tmux send-keys -t "$SESSION:nav.3" \
  "source install/setup.bash && ros2 launch me_nav2_bringup my_nav2_launch.py" C-m
tmux select-pane -t "$SESSION:nav.3" -T "Nav2"

# 附加到 session，默认显示窗口 1 (core)
tmux select-window -t "$SESSION:core"
tmux attach -t "$SESSION"

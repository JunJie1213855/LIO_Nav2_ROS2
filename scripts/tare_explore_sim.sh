#!/usr/bin/env bash
# Gazebo + FAST-LIO + TARE-Planner 自主探索 (gnome-terminal)
# 用法: bash scripts/tare_explore_sim.sh

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

# ── Gazebo 仿真环境 ──────────────────────────────────────────────
gnome-terminal --title="Gazebo 仿真" -- bash -c "
killall -9 gzserver gzclient 2>/dev/null;
source $WORKSPACE_ROOT/install/setup.bash;
ros2 launch get_urdf get_urdf_launch.py"

sleep 3

# ── FAST-LIO 3D 里程计 ──────────────────────────────────────────
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source $WORKSPACE_ROOT/install/setup.bash;
ros2 launch fast_lio mapping.launch.py"

# ── 里程计接口 ──────────────────────────────────────────────────
gnome-terminal --title="lio_interface" -- bash -c "
source $WORKSPACE_ROOT/install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

# ── 传感器扫描生成 ──────────────────────────────────────────────
gnome-terminal --title="sensor_scan_generation" -- bash -c "
source $WORKSPACE_ROOT/install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ── TARE-Planner 自主探索 + waypoint follower + RViz ──────────────
gnome-terminal --title="TARE 自主探索" -- bash -c "
source $WORKSPACE_ROOT/install/setup.bash;
ros2 launch me_nav2_bringup tare_lio_explore_launch.py"

# ── GUI 手动控制 (可选, 用于干预) ────────────────────────────────
gnome-terminal --title="GUI控制" -- bash -c "
source $WORKSPACE_ROOT/install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

echo "===== Gazebo + FAST-LIO + TARE 自主探索 ====="
echo "窗口: Gazebo | FAST-LIO | lio_if | sensor | TARE | GUI控制"
echo ""
echo "管线:"
echo "  Gazebo → FAST-LIO → lio_interface → sensor_scan_generation"
echo "                                          ↓"
echo "                                     TARE-Planner → /way_point"
echo "                                          ↓"
echo "                                 waypoint_follower → /cmd_vel → Gazebo"

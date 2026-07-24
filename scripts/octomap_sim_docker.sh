#!/usr/bin/env bash
# ===========================================================================
# 容器内 OctoMap 3D 八叉树建图 — Gazebo + FAST-LIO + OctoMap
#
# 用法（宿主机）:
#   xhost +local:docker                          # 允许容器访问 X11
#   docker exec -it lio_nav2 /ws/scripts/octomap_sim_docker.sh
#   docker exec -it lio_nav2 tmux attach -t octomap_sim  # 查看各窗口
#
# 停止: docker exec lio_nav2 tmux kill-session -t octomap_sim
#
# 切换 LIO 算法: 取消 Fast-LIO 那行，取消另一种算法的注释
# ===========================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=octomap_sim

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ── GUI 控制小车 ──────────────────────────────────────────────────────
tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"

# ── Gazebo 仿真 ───────────────────────────────────────────────────────
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py rviz:=false"

# ── FAST-LIO 里程计 ──────────────────────────────────────────────────
new_win "FAST-LIO" "ros2 launch fast_lio mapping.launch.py"
# --- Super-LIO ---
# new_win "Super-LIO" "ros2 launch super_lio sim_gazebo.py"
# --- Point-LIO ---
# new_win "Point-LIO" "ros2 launch point_lio point_lio.launch.py"

# ── lio_interface (camera_init → odom 坐标转换) ──────────────────────
new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py"
# 如果切换了 LIO 算法, 需要同时切换 lio_type:
# new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py lio_type:=superlio"
# new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio"

# ── sensor_scan_generation (odom → base_footprint 转换) ───────────────
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ── OctoMap 八叉树建图 ────────────────────────────────────────────────
new_win "OctoMap" "ros2 launch lio_octomap octomap_mapping.launch.py"
# 如果切换了 LIO 算法, 需要同时切换 lio_type:
# new_win "OctoMap" "ros2 launch lio_octomap octomap_mapping.launch.py lio_type:=superlio"
# new_win "OctoMap" "ros2 launch lio_octomap octomap_mapping.launch.py lio_type:=pointlio"

# ── RViz 可视化八叉树 + 点云 ─────────────────────────────────────────
new_win "RViz" "ros2 run rviz2 rviz2 -d /ws/src/lio_octomap/rviz/octomap.rviz"

echo "=============================================="
echo " tmux 会话: $SESSION"
echo " 查看输出:  tmux attach -t $SESSION"
echo " 切换窗口:  Ctrl-b n / p"
echo " 退出查看:  Ctrl-b d"
echo " 全部停止:  tmux kill-session -t $SESSION"
echo "=============================================="
echo ""
echo "RViz 中查看:"
echo "  Fixed Frame: camera_init"
echo "  → /octomap_binary  (八叉树占据地图)"
echo "  → /registered_scan  (点云叠加)"
echo "  → TF 树"
echo ""
echo "切换 LIO 算法时请同时修改 lio_interface 和 OctoMap 的 lio_type"
echo "并手动调整 RViz 的 Fixed Frame (camera_init / world / odom)"

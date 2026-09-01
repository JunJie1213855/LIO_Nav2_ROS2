#!/usr/bin/env bash
# 容器内 RoboSense Airy 数据集 — SCAN-Planner + LIO 建图定位导航一体化 (tmux)
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/robo_scan_planner_docker.sh
#   docker exec -it lio_nav2 tmux attach -t robo_scan_planner
#   docker exec lio_nav2 tmux kill-session -t robo_scan_planner
#
# 管线:
#   bag → FAST-LIO → lio_interface → SCAN-Planner (ESDF建图+局部规划)
#                    robot_desc → 静态 TF
#
# 数据流:
#   /rslidar_points ─→ FAST-LIO ─→ /Odometry + /cloud_registered
#                                      ↓
#                                 lio_interface ─→ /registered_scan (odom系)
#                                               ─→ /registered_odometry
#                                      ↓
#                                 SCAN-Planner ─→ /grid_map/occupancy (ESDF地图)
#                                             ─→ /planning/bspline (轨迹)
#                                             ─→ /cmd_vel (控制指令)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=robo_scan_planner
BAG_DIR="${1:-/dataset/robosense/robosenseAiry-slamtoolbox}"

# FastRTPS 的 "invalid allocator" bug 导致 RViz 无法创建订阅，换 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ── 窗口 1: 数据集回放 ──────────────────────────────────────────────
tmux new-session -d -s "$SESSION" -n "bag播放" \
  "bash -c 'source /opt/ros/humble/setup.bash && ros2 bag play $BAG_DIR/robosenseAiry-slamtoolbox_0.db3 --clock; exec bash'"

# ── 窗口 2: FAST-LIO 3D 里程计 ──────────────────────────────────────
new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py \
    use_sim_time:=true rviz:=true"

# ── 窗口 3: 机器人描述 (静态 TF: base_footprint→livox_frame) ────────
new_win "robot_desc" "ros2 launch gld_robot_description robosenseAiry_description_launch.py rviz:=false"

# ── 窗口 4: LIO 接口 (坐标系转换) ────────────────────────────────────
new_win "lio_if" "ros2 launch lio_interface lio_interface_launch.py use_sim_time:=true"

# ── 窗口 5: 传感器扫描生成 (odom→base_footprint TF + /odom 话题) ────
new_win "sensor" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ── 窗口 6: SCAN-Planner (ESDF建图 + 局部规划) ────────────────────
new_win "SCAN-Planner" "ros2 launch nav2_planner_bringup scan_planner_lio_launch.py \
    use_sim_time:=true navi_mode:=1"

# ── 窗口 7: SCAN-Planner RViz ──────────────────────────────────────
new_win "SP-RViz" "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_scan_planner -p use_sim_time:=true -- -d /ws/src/planner/nav2_planner_bringup/rviz/scan_planner.rviz"

echo "===== SCAN-Planner + LIO 建图定位导航一体化 (会话: $SESSION) ====="
echo "窗口: bag播放 | FAST-LIO | robot_desc | lio_if | sensor | SCAN-Planner | SP-RViz"
echo ""
echo "SP-RViz 窗口: 独立 RViz (scan_planner.rviz), 与 FAST-LIO 的 RViz 不冲突"
echo ""
echo "操作步骤:"
echo "  1. docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "  2. 等待 FAST-LIO 稳定"
echo "  3. 切换到 SP-RViz 窗口 (Fixed Frame: odom)"
echo "  4. 用 '2D Goal Pose' 工具点击目标位姿"
echo "  5. SCAN-Planner 规划 B-spline 轨迹并视觉显示"
echo ""
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"

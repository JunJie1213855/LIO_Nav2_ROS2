#!/usr/bin/env bash
# 容器内运行版 Cartographer 纯定位 + Nav2 导航脚本
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
#
# 用法（宿主机）：
#   xhost +local:docker
#   docker exec -it lio_nav2 /ws/scripts/nav2_sim_carto_docker.sh
#   docker exec -it lio_nav2 tmux attach -t nav2_carto    # 查看各节点输出
#
# tmux 快捷键：Ctrl-b n / p 切换窗口，Ctrl-b d 退出查看（节点继续运行）
# 停止全部：docker exec lio_nav2 tmux kill-session -t nav2_carto
#
# ============================================================================
# 与 nav2_sim_docker.sh 的区别:
#   用 Cartographer 纯定位替代 KISS-Matcher 重定位
#   - Cartographer 加载 .pbstream 地图，发布 map→odom TF
#   - map_server 加载 .pgm 静态地图 (从 pbstream 导出)
# ============================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=nav2_carto

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

new_win() { # new_win <窗口名> <命令>
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# gui控制小车 (tkinter, 需要 X11)
tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"

# --- FAST-LIO 里程计 ---
new_win "FAST-LIO" "ros2 launch fast_lio mapping.launch.py"

# 里程计接口
new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py"

# --- Gazebo 仿真 ---
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

# 点云组装 + 里程计发布
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 3D点云转2D LaserScan
new_win "pc2laser" "ros2 launch nav2_planner pointcloud_to_laserscan_launch.py"

# --- Cartographer 纯定位 (替代 KISS-Matcher) ---
# 加载 .pbstream 地图，只做定位不建图
# 发布: map→odom TF (重定位)
new_win "Carto定位" "ros2 launch nav2_planner cartographer_localization_launch.py \
    load_state_filename:=/ws/src/planner/nav2_planner/map/map.pbstream"

# --- Nav2 导航栈 ---
# map_server 在 my_nav2_launch.py 中加载 .pgm 地图 (从 pbstream 导出)
new_win "Nav2" "ros2 launch nav2_planner my_nav2_launch.py"

echo "已在 tmux 会话 '$SESSION' 中启动全部节点。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "全部停止: tmux kill-session -t $SESSION"
echo ""
echo "============================================================"
echo " 注意事项:"
echo "   1. 确保 map.pbstream 已准备好 (从建图阶段导出)"
echo "   2. Nav2 map_server 加载的 .pgm 需要与 pbstream 对应"
echo "   3. 等 Cartographer 终端打印定位成功后,"
echo "      在 RViz 中用 '2D Pose Estimate' 给初始位姿"
echo "   4. 如果脱离定位, 在 RViz 中重新给初始位姿即可"
echo "============================================================"

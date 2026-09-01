#!/usr/bin/env bash
# 容器内运行版 Cartographer 建图脚本
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
#
# 用法（宿主机）：
#   xhost +local:docker
#   docker exec -it lio_nav2 /ws/scripts/mapping_sim_carto_docker.sh
#   docker exec -it lio_nav2 tmux attach -t mapping_carto  # 查看各节点输出
#
# tmux 快捷键：Ctrl-b n / p 切换窗口，Ctrl-b d 退出查看（节点继续运行）
# 停止全部：docker exec lio_nav2 tmux kill-session -t mapping_carto
#
# ============================================================================
# 与 mapping_sim_docker.sh 的区别:
#   用 Cartographer 替代 slam_toolbox
#   - 自带回环检测
#   - 发布 /map (OccupancyGrid) + map→odom TF
#   - 不需要 KISS-Matcher
# ============================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=mapping_carto

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

new_win() { # new_win <窗口名> <命令>
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# gui控制小车 (tkinter, 需要 X11)
tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"

# ============== FAST-LIO 里程计 ==============
new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py"

# 里程计接口 (odom TF 桥接)
new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py"

# ============== Gazebo 仿真 ==============
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

# ============== 点云组装 + 里程计发布 ==============
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ============== 3D点云转2D LaserScan ==============
new_win "pc2laser" "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

# ============== Cartographer 2D 在线建图 (替代 slam_toolbox) ==============
# 订阅: /scan + /livox/imu + /odom
# 发布: /map (OccupancyGrid) + map→odom TF
new_win "Cartographer" "ros2 launch nav2_planner_bringup cartographer_mapping_launch.py"

# ============== RViz 可视化 Cartographer 建图 ==============
# 显示: /map (Cartographer OccupancyGrid) + /scan + TF + 机器人模型 + /registered_scan
new_win "RViz" "ros2 run rviz2 rviz2 -d /ws/src/planner/nav2_planner_bringup/rviz/cartographer_mapping.rviz"

echo "已在 tmux 会话 '$SESSION' 中启动全部节点。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "全部停止: tmux kill-session -t $SESSION"
echo ""
echo "============================================================"
echo " 建图完成后保存地图:"
echo ""
echo "   1. 结束轨迹:"
echo "      ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \"{trajectory_id: 0}\""
echo ""
echo "   2. 导出 pbstream → pgm + yaml:"
echo "      ros2 run cartographer_ros cartographer_pbstream_to_ros_map \\"
echo "          -pbstream_filename /ws/src/planner/nav2_planner_bringup/map/map.pbstream \\"
echo "          -map_filestem /ws/src/planner/nav2_planner_bringup/map/my_map"
echo "============================================================"

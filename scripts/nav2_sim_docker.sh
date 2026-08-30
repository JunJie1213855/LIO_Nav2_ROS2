#!/usr/bin/env bash
# 容器内运行版 nav2_sim.sh
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
# 用法（宿主机）：
#   xhost +local:docker
#   docker exec -it lio_nav2 /ws/scripts/nav2_sim_docker.sh
#   docker exec -it lio_nav2 tmux attach -t nav2_sim   # 查看各节点输出
# tmux 快捷键：Ctrl-b n / p 切换窗口，Ctrl-b d 退出查看（节点继续运行）
# 停止全部：docker exec lio_nav2 tmux kill-session -t nav2_sim

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=nav2_sim

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

# 每个窗口：进工作空间 -> source -> 启动节点；节点退出后保留 shell 方便看报错
new_win() { # new_win <窗口名> <命令>
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# gui控制小车（tkinter，需要 X11）
tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"

# 使用 fast-lio 作为里程计（与 nav2_sim.sh 一致）
new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping.launch.py"

# 里程计接口
new_win "lio_interface" "ros2 launch lio_interface lio_interface_launch.py"

# Gazebo 仿真环境
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 3d点云转2d
new_win "pc2laser" "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

# KISS + GICP 重定位（导航模式核心：基于先验 PCD 地图重定位，发布 map->odom）
new_win "KISS+GICP" "ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py"

# Nav2 导航（使用已有地图，非 slam 模式）
# RViz 可视化（独立窗口，与 Nav2 解耦，关掉 Nav2 也不影响看地图和点云）
# new_win "RViz" "ros2 run rviz2 rviz2 -d /ws/src/planner/nav2_planner_bringup/rviz/nav2.rviz"
new_win "Nav2" "ros2 launch nav2_planner_bringup my_nav2_launch.py"

echo "已在 tmux 会话 '$SESSION' 中启动全部节点。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "全部停止: tmux kill-session -t $SESSION"
echo ""
echo "注意：导航模式需要预先建好的地图，请确认以下配置正确："
echo "  - nav2_planner_bringup/launch/my_nav2_launch.py 中 map_yaml_file 指向已有 .yaml"
echo "  - global_relocalization_kiss_matcher 中 prior_pcd_file 指向已有 .pcd"
echo "  - 在 RViz 中用 '2D Pose Estimate' 给初始位姿，或用 'Nav2 Goal' 发送目标"

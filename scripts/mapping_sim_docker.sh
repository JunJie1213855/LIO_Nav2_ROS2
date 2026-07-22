#!/usr/bin/env bash
# 容器内运行版 mapping_sim.sh
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
# 用法（宿主机）：
#   xhost +local:docker                          # 允许容器访问 X11（每次开机执行一次）
#   docker exec -it lio_nav2 /ws/scripts/mapping_sim_docker.sh
#   docker exec -it lio_nav2 tmux attach -t mapping_sim   # 查看各节点输出
# tmux 快捷键：Ctrl-b n / p 切换窗口，Ctrl-b d 退出查看（节点继续运行）
# 停止全部：docker exec lio_nav2 tmux kill-session -t mapping_sim

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=mapping_sim

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

# 每个窗口：进工作空间 -> source -> 启动节点；节点退出后保留 shell 方便看报错
new_win() { # new_win <窗口名> <命令>
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# gui控制小车界面（tkinter，需要 X11）
tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"


# --- FAST-LIO（推荐，仿真稳定）---
new_win "FAST-LIO" "ros2 launch fast_lio mapping.launch.py"

# lio camera -> body => /odom -> livox_frame 
new_win "lio_interface" "ros2 launch lio_interface fastlio_lio_interface_launch.py"

# Gazebo 仿真环境
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

# 中间层转换
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 3d点云转2d
new_win "pc2laser" "ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"

# RViz 可视化（独立窗口，与 Nav2 解耦，关掉 Nav2 也不影响看 SLAM 建图）
# new_win "RViz" "ros2 run rviz2 rviz2 -d /ws/src/me_nav2_bringup/rviz/nav2.rviz"

# slam_toolbox 建图
new_win "slam_toolbox" "ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/me_nav2_bringup/config/slam_toolbox_params.yaml"

# Nav2 导航（可选——关掉不影响建图和 RViz 显示）
# new_win "Nav2" "ros2 launch me_nav2_bringup my_nav2_launch.py"

echo "已在 tmux 会话 '$SESSION' 中启动全部节点。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "全部停止: tmux kill-session -t $SESSION"

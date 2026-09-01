#!/usr/bin/env bash
# 容器内运行版 mapping_sim.sh
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
# 用法（宿主机）：
#   xhost +local:docker                          # 允许容器访问 X11（每次开机执行一次）
#   docker exec -it lio_nav2 /ws/scripts/mapping_sim.sh [0|1|2]
#   docker exec -it lio_nav2 tmux attach -t mapping_sim   # 查看各节点输出
# 第一个参数选择 LIO 算法（缺省 0）:
#   0 = fast-lio   1 = point-lio   2 = super-lio
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



# ============== Gazebo 仿真环境 ==============
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

# ============== LIO 算法选择（脚本第一个参数）==============
# 用法: bash scripts/mapping_sim.sh [0|1|2]
#   0 = fast-lio   (默认)
#   1 = point-lio
#   2 = super-lio
LIO_SEL="${1:-0}"
case "$LIO_SEL" in
  0) 
    LIO_WIN="FAST-LIO"
    LIO_CMD="ros2 launch fast_lio_robosense mapping.launch.py"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py"                     # 默认 fastlio
    ;;
  1)
    LIO_WIN="Point-LIO"
    LIO_CMD="ros2 launch point_lio point_lio.launch.py"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio"
    ;;
  2)
    LIO_WIN="Super-LIO"
    LIO_CMD="ros2 launch super_lio sim_gazebo.py"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py lio_type:=superlio"
    ;;
  *)
    echo "无效参数 '$LIO_SEL'，用法: $0 [0|1|2] (0=fast-lio 1=point-lio 2=super-lio)"
    exit 1
    ;;
esac

new_win "$LIO_WIN" "$LIO_CMD"

# ============== lio interface（按所选 LIO 自动匹配话题）==============
# lio camera -> body => /odom -> livox_frame
new_win "lio_interface" "$LIO_IFACE"

# ============== 中间层转换  ==============
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ============== 3d点云转2d ==============
new_win "pc2laser" "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

# ============== RViz 可视化 slam_toolbox 建图过程 ==============
new_win "RViz" "ros2 run rviz2 rviz2 -d /ws/src/planner/nav2_planner_bringup/rviz/nav2.rviz"

# ============== slam_toolbox 建图 ==============
new_win "slam_toolbox" "ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

# ============== Nav2 导航（可选——关掉不影响建图和 RViz 显示） ==============
# new_win "Nav2" "ros2 launch nav2_planner_bringup my_nav2_launch.py"

echo "已在 tmux 会话 '$SESSION' 中启动全部节点 (LIO: $LIO_WIN)。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "全部停止: tmux kill-session -t $SESSION"

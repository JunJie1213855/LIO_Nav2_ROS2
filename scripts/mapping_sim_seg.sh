#!/usr/bin/env bash
# 容器内运行版：Gazebo + LIO + 地面分割(GroundSeg) + SCAN-Planner 联合仿真探索
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
# 用法（宿主机）：
#   xhost +local:docker                          # 允许容器访问 X11（每次开机执行一次）
#   docker exec -it lio_nav2 /ws/scripts/mapping_sim_seg.sh [0|1|2]
#   docker exec -it lio_nav2 tmux attach -t mapping_sim   # 查看各节点输出
# 第一个参数选择 LIO 算法（缺省 0）:
#   0 = fast-lio   1 = point-lio   2 = super-lio
# 操作：切到 SP-RViz 窗口，用 '2D Goal Pose' 点目标开始探索。
# tmux 快捷键：Ctrl-b n / p 切换窗口，Ctrl-b d 退出查看（节点继续运行）
# 停止全部：docker exec lio_nav2 tmux kill-session -t mapping_sim

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=mapping_sim

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  sensor_scan_generation_node scan_planner_node closed_loop_controller \
  cloud_z_filter 2>/dev/null
# sleep 2

# 每个窗口：进工作空间 -> source -> 启动节点；节点退出后保留 shell 方便看报错
new_win() { # new_win <窗口名> <命令>
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ============== Gazebo 仿真环境 ==============
tmux new-session -d -s "$SESSION" -n "Gazebo" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && \
    ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"
# sleep 6

# ============== LIO 算法选择（脚本第一个参数）==============
# 用法: bash scripts/mapping_sim_seg.sh [0|1|2]
#   0 = fast-lio   (默认)
#   1 = point-lio
#   2 = super-lio
LIO_SEL="${1:-0}"
case "$LIO_SEL" in
  0)
    LIO_WIN="FAST-LIO"
    LIO_CMD="ros2 launch fast_lio_robosense mapping.launch.py rviz:=false use_sim_time:=true"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py"                     # 默认 fastlio
    ;;
  1)
    LIO_WIN="Point-LIO"
    LIO_CMD="ros2 launch point_lio point_lio.launch.py rviz:=false"
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
# sleep 3

# ============== 地面分割（GroundSeg，去地面）==============
# 消费 Gazebo 直接输出的 /livox/lidar，去地面后输出 /no_ground_cloud（livox_frame 系）。
# scan_planner_lio_launch.py 里的 cloud_frame_transform 会把 /no_ground_cloud
# 从 livox_frame 转到 odom 系，再经 cloud_z_filter 只去天花板。
new_win "GroundSeg" "ros2 launch efficient_online_segmentation ground_separation.launch.py"
# sleep 2

# ============== lio interface（按所选 LIO 自动匹配话题）==============
# lio camera -> body => /odom -> livox_frame
new_win "lio_interface" "$LIO_IFACE"

# ============== 中间层转换 ==============
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
# sleep 2

# ============== SCAN-Planner 探索规划 ==============
# 地面已由 GroundSeg 去掉，z_min:=0.0 只保留天花板过滤（z_max:=3.0）。
# 输入：body_pose=/odom、cloud=/registered_scan_filtered；输出：/cmd_vel 轨迹。
new_win "SCAN" "ros2 launch scan_planner scan_planner_lio_launch.py use_sim_time:=true \
    z_min:=0.0 z_max:=3.0 \
    double_cylinder_radius:=0.45 double_cylinder_offset:=0.18 \
    body_height:=0.25 obstacles_inflation_z_down:=1.0 \
    optimization.lambda_collision:=50.0 optimization.dist0:=3.0 \
    grid_map.p_occ:=0.3 grid_map.p_hit:=0.9 grid_map.p_miss:=0.1"
# sleep 2

# ============== SCAN-Planner 可视化 ==============
new_win "SP-RViz" "ros2 run rviz2 rviz2 -d src/planner/nav2_planner_bringup/rviz/scan_planner.rviz"

echo "已在 tmux 会话 '$SESSION' 中启动 Gazebo + LIO + GroundSeg + SCAN-Planner (LIO: $LIO_WIN)。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "操作: 切到 SP-RViz 窗口, 用 '2D Goal Pose' 点目标开始探索"
echo "全部停止: tmux kill-session -t $SESSION"

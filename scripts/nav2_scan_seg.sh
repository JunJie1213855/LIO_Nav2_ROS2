#!/usr/bin/env bash
# 容器内运行版：Gazebo + LIO + 地面分割(GroundSeg) + SCAN-Planner 联合仿真探索
# 容器里没有 gnome-terminal，用 tmux 的多个窗口代替多个终端窗口。
# 用法（宿主机）：
#   xhost +local:docker                          # 允许容器访问 X11（每次开机执行一次）
#   docker exec -it lio_nav2 /ws/scripts/nav2_scan_seg.sh [0|1|2]
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
  sensor_scan_generation_node scan_planner_node closed_loop_controller closed_loop_controller_car \
  ground_separation_node cloud_only_segmentation_node efficient_online_segmentation_node cloud_z_filter 2>/dev/null
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
# 用法: bash scripts/nav2_scan_seg.sh [0|1|2]
#   0 = fast-lio   (默认)
#   1 = point-lio
#   2 = super-lio
LIO_SEL="${1:-0}"
case "$LIO_SEL" in
  0)
    LIO_WIN="FAST-LIO"
    # GroundSeg needs dense samples: keep all valid preprocessing points and publish
    # the undistorted scan before the matching voxel filter (filter_size_surf unchanged).
    LIO_CMD="ros2 launch fast_lio_robosense mapping.launch.py rviz:=false use_sim_time:=true point_filter_num:=1 dense_publish_en:=true"
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

# ============== FAST-LIO 位姿转换 ==============
# /Odometry -> /registered_odometry (odom -> livox_frame)
# /registered_odometry + /registered_scan -> /odom (odom -> base_footprint)
new_win "lio_interface" "$LIO_IFACE use_sim_time:=true"
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ============== 地面分割（配准点云中去地面）==============
# 同步 /registered_scan 与 /odom，使用 YAML 中的车体到雷达外参做局部分割。
# 输出 /ground_seg/odom、/ground_cloud、/no_ground_cloud；不广播 TF。
new_win "GroundSeg" "ros2 launch efficient_online_segmentation ground_separation.launch.py \
    use_sim_time:=true input_cloud_is_world:=true \
    cloud_topic:=/registered_scan odometry_topic:=/odom \
    odometry_is_sensor_pose:=false body_frame_id:=base_footprint \
    pub_odometry_topic:=/ground_seg/odom"

# ============== SCAN-Planner + 闭环控制 ==============
# 使用配对后的 /ground_seg/odom 和世界系点云，不再在规划器中投影。
new_win "SCAN" "ros2 launch scan_planner scan_planner_lio_launch.py \
    use_sim_time:=true controller_type:=car body_pose_topic:=/ground_seg/odom \
    sensor_pose_topic:=/ground_seg/odom \
    cloud_topic:=/no_ground_cloud cloud_is_world:=true"

new_win "SP-RViz" "ros2 run rviz2 rviz2 \
    -d \$(ros2 pkg prefix --share nav2_planner_bringup)/rviz/scan_planner.rviz \
    --ros-args -r __node:=rviz2_scan_seg -r /registered_scan:=/no_ground_cloud \
    -p use_sim_time:=true"

echo "已在 tmux 会话 '$SESSION' 中启动 Gazebo + LIO + GroundSeg + SCAN-Planner (LIO: $LIO_WIN)。"
echo "查看输出: tmux attach -t $SESSION   (Ctrl-b n/p 切换窗口, Ctrl-b d 退出)"
echo "规划输入: /ground_seg/odom (配对位姿), /no_ground_cloud (odom 系配准非地面点云)"
echo "操作: 切到 SP-RViz 窗口, 用 '2D Goal Pose' 点目标开始探索"
echo "全部停止: tmux kill-session -t $SESSION"

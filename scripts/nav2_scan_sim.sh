#!/usr/bin/env bash
# Gazebo + LIO + SCAN-Planner 联合仿真（tmux 多窗口，结构参照 mapping_sim.sh）
# 用法: bash scripts/nav2_scan_sim.sh [0|1|2] [--dry-run]
#   0 = fast-lio（默认），1 = point-lio，2 = super-lio
# 查看: tmux attach -t scan_gz；停止: tmux kill-session -t scan_gz
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
SESSION=scan_gz
LIO_SEL="${1:-0}"
DRY_RUN=false

# ============== 参数与环境 ==============
case "$LIO_SEL" in
  --dry-run) LIO_SEL=0; DRY_RUN=true ;;
  0|1|2) ;;
  -h|--help) echo "用法: $0 [0|1|2] [--dry-run]"; exit 0 ;;
  *) echo "无效参数 '$LIO_SEL'，用法: $0 [0|1|2] [--dry-run]" >&2; exit 2 ;;
esac
case "${2:-}" in
  --dry-run) DRY_RUN=true ;;
  '') ;;
  *) echo "无效选项: $2" >&2; exit 2 ;;
esac
if (( $# > 2 )); then echo "参数过多，用法: $0 [0|1|2] [--dry-run]" >&2; exit 2; fi

if [[ -f /usr/share/gazebo/setup.sh ]]; then source /usr/share/gazebo/setup.sh; fi
source "$WORKSPACE_ROOT/install/setup.bash"
for tool in ros2 tmux timeout; do command -v "$tool" >/dev/null; done
for package in get_urdf lio_interface sensor_scan_generation scan_planner ign_sim_pointcloud_tool rviz2; do
  ros2 pkg prefix "$package" >/dev/null
done
RVIZ_CONFIG="$(ros2 pkg prefix --share nav2_planner_bringup)/rviz/scan_planner.rviz"
[[ -f "$RVIZ_CONFIG" ]] || { echo "缺少 RViz 配置: $RVIZ_CONFIG" >&2; exit 1; }

# ============== LIO 算法选择（脚本第一个参数）==============
# 仿真雷达统一经 CloudConvert 转成 /velodyne_points；雷达和 IMU 在同一 link。
case "$LIO_SEL" in
  0)
    LIO_WIN="FAST-LIO"
    LIO_SHARE="$(ros2 pkg prefix --share fast_lio_robosense)"
    LIO_CONFIG="$LIO_SHARE/config/mid360.yaml"
    LIO_CMD="ros2 run fast_lio_robosense fastlio_mapping --ros-args \
      --params-file \"$LIO_CONFIG\" -p use_sim_time:=true \
      -p common.lid_topic:=/velodyne_points \
      -p preprocess.scan_line:=50 -p preprocess.timestamp_unit:=0 \
      -p 'mapping.gravity_ref:=[0.0, 0.0, -9.81]' \
      -p 'mapping.extrinsic_T:=[0.0, 0.0, 0.0]' \
      -p mapping.extrinsic_est_en:=false -p pcd_save.pcd_save_en:=false"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py lio_type:=fastlio use_sim_time:=true"
    ;;
  1)
    LIO_WIN="Point-LIO"
    LIO_SHARE="$(ros2 pkg prefix --share point_lio)"
    LIO_CONFIG="$LIO_SHARE/config/mid360_sim.yaml"
    LIO_CMD="ros2 run point_lio pointlio_mapping --ros-args \
      --params-file \"$LIO_CONFIG\" -p use_sim_time:=true \
      -p common.lid_topic:=/velodyne_points \
      -p 'mapping.gravity:=[0.0, 0.0, -9.81]' \
      -p 'mapping.gravity_init:=[0.0, 0.0, -9.81]' \
      -p 'mapping.extrinsic_T:=[0.0, 0.0, 0.0]' \
      -p pcd_save.pcd_save_en:=false"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio use_sim_time:=true"
    ;;
  2)
    LIO_WIN="Super-LIO"
    LIO_SHARE="$(ros2 pkg prefix --share super_lio)"
    LIO_CONFIG="$LIO_SHARE/config/sim_gazebo.yaml"
    LIO_CMD="ros2 run super_lio super_lio_node --ros-args \
      --params-file \"$LIO_CONFIG\" -p use_sim_time:=true \
      -p lio.ros.lidar_topic:=/velodyne_points \
      -p 'lio.extrinsic.lidar_imu:=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]'"
    LIO_IFACE="ros2 launch lio_interface lio_interface_launch.py lio_type:=superlio use_sim_time:=true"
    ;;
esac
[[ -f "$LIO_CONFIG" ]] || { echo "缺少 LIO 配置: $LIO_CONFIG" >&2; exit 1; }

# 每个窗口：进工作空间 -> source -> 启动节点；退出后保留 shell 方便查看报错。
# 和 mapping_sim.sh 一样，调用形式为 new_win "窗口名" "命令"。
new_win() {
  local shell_body tmux_command
  if "$DRY_RUN"; then printf '[%s] %s\n' "$1" "$2"; return; fi
  printf -v shell_body 'cd %q && { if [[ -f /usr/share/gazebo/setup.sh ]]; then source /usr/share/gazebo/setup.sh; fi; } && source %q && %s; exec bash' \
    "$WORKSPACE_ROOT" "$WORKSPACE_ROOT/install/setup.bash" "$2"
  printf -v tmux_command 'bash -c %q' "$shell_body"
  if [[ "$1" == Gazebo ]]; then
    tmux new-session -d -s "$SESSION" -n "$1" "$tmux_command"
  else
    tmux new-window -t "$SESSION" -n "$1" "$tmux_command"
  fi
}

wait_for_topic() {
  if "$DRY_RUN"; then echo "[等待] $1 首条消息，最长 60 秒"; return; fi
  echo "等待 $1 数据……"
  timeout 60s ros2 topic echo "$1" "$2" --once --qos-reliability best_effort >/dev/null || {
    echo "等待 $1 超时。查看日志: tmux attach -t $SESSION" >&2; exit 1;
  }
}

# ============== 关闭本脚本旧会话 ==============
if ! "$DRY_RUN"; then tmux kill-session -t "$SESSION" 2>/dev/null || true; fi

# ============== Gazebo 仿真环境 ==============
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py rviz:=false"
wait_for_topic /clock rosgraph_msgs/msg/Clock

# ============== 仿真点云转换（补充 ring/time 字段）==============
new_win "CloudConvert" "ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node \
  --ros-args -p use_sim_time:=true -p pcd_topic:=/livox/lidar \
  -p n_scan:=50 -p horizon_scan:=360 -p ang_bottom:=7.2192682 -p ang_res_y:=1.2733804"

# ============== LIO 与中间层 ==============
new_win "$LIO_WIN" "$LIO_CMD"
new_win "lio_interface" "$LIO_IFACE"
new_win "sensor_scan" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
wait_for_topic /odom nav_msgs/msg/Odometry

# ============== SCAN-Planner + 闭环控制 ==============
new_win "SCAN" "ros2 launch scan_planner scan_planner_lio_launch.py \
  use_sim_time:=true controller_type:=car cloud_is_world:=true"

# ============== SCAN-Planner 可视化 ==============
new_win "SP-RViz" "ros2 run rviz2 rviz2 -d \"$RVIZ_CONFIG\" \
  --ros-args -r __node:=rviz2_scan -p use_sim_time:=true"

if "$DRY_RUN"; then
  echo '命令预览完成，未启动或停止进程。'
else
  echo "已在 tmux 会话 '$SESSION' 中启动 Gazebo + $LIO_WIN + SCAN-Planner。"
  echo "查看输出: tmux attach -t $SESSION (Ctrl-b n/p 切换窗口，Ctrl-b d 退出)"
  echo "全部停止: tmux kill-session -t $SESSION"
  echo "操作: 切到 SP-RViz 窗口，用 '2D Goal Pose' 点目标（odom 坐标系）。"
fi

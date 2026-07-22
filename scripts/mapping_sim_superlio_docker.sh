#!/usr/bin/env bash
# ===========================================================================
# 容器内 Super-LIO + Gazebo 联合仿真建图
#
# Super-LIO 自己就是一套完整的 LiDAR-惯性 SLAM 系统:
#   输入:  /livox/lidar (PointCloud2) + /livox/imu (Imu)
#   输出:  /lio/odom (里程计) + /lio/cloud_world (地图点云) + /lio/path (轨迹)
#         TF: world → imu
#
#   配合 Super-LIO 自带的 RViz 配置即可看到实时建图效果。
#
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/mapping_sim_superlio_docker.sh
#   docker exec -it lio_nav2 tmux attach -t mapping_sim
# 停止:
#   docker exec lio_nav2 tmux kill-session -t mapping_sim
# ===========================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=mapping_sim
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
# 发布: /livox/lidar (PointCloud2), /livox/imu (Imu)
# TF:   base_footprint → chassis → livox_frame
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py rviz:=false"

# ── Super-LIO SLAM ────────────────────────────────────────────────────
# lidar_type:=4 = VELO32 → 订阅标准 sensor_msgs/PointCloud2
# lidar_type:=1 = LIVOX  → 订阅 Livox CustomMsg (仿真不适用)
#
# 使用 launch 参数覆盖 yaml 默认值, 匹配 Gazebo 的话题名
# new_win "Super-LIO" "ros2 launch super_lio Livox_mid360.py \
#     lio.sensor.lidar_type:=4 \
#     lio.ros.lidar_topic:=/livox/lidar \
#     lio.ros.imu_topic:=/livox/imu"

# # ── 诊断窗口 ──────────────────────────────────────────────────────────
# # 延迟 5 秒等待所有节点就绪, 然后检查话题是否正常
# new_win "诊断" "sleep 5; \
#     echo '=== 话题列表 ==='; ros2 topic list | grep -E 'livox|lio|velodyne|imu|world'; \
#     echo; echo '=== Super-LIO odom (最后一条) ==='; \
#     timeout 3 ros2 topic echo /lio/odom --once 2>/dev/null || echo '(暂无数据)'; \
#     echo; echo '=== 如上面 /lio/odom 有数据说明 Super-LIO 正常 ==='; \
#     echo '=== 在 Super-LIO 的 RViz 中可看到 world 帧下的地图点云 ==='; \
#     exec bash"

echo "=============================================="
echo " tmux 会话: $SESSION"
echo " 查看输出:  tmux attach -t $SESSION"
echo " 切换窗口:  Ctrl-b n / p"
echo " 退出查看:  Ctrl-b d"
echo " 全部停止:  tmux kill-session -t $SESSION"
echo "=============================================="
echo ""
echo "Super-LIO 自带 RViz 会自动打开 (Livox_mid360.py 默认 rviz:=true)"
echo "RViz 中查看:"
echo "  Fixed Frame: world"
echo "  → /lio/cloud_world  (地图点云)"
echo "  → /lio/odom          (轨迹)"
echo "  → /lio/path          (路径)"

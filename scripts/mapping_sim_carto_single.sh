#!/usr/bin/env bash
# 精简建图 — 只用 Gazebo + Cartographer
# 移除: FAST-LIO, lio_interface, sensor_scan_generation
# 管线: Gazebo → /livox/lidar → pointcloud_to_laserscan → /scan → Cartographer 2D
#       Gazebo → /livox/imu ─────────────────────────────────→ Cartographer
#       Cartographer → /map (OccupancyGrid) + odom TF
#
# 用法: docker exec -it lio_nav2 /ws/scripts/mapping_sim_carto_simple_docker.sh
# 查看: docker exec -it lio_nav2 tmux attach -t mapping_simple
# 停止: docker exec lio_nav2 tmux kill-session -t mapping_simple

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=mapping_simple

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ============== gui 控制 ==============
tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"

# ============== gazebo 仿真 ==============
new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

# ============== 3D→2D 切片 (直接拿仿真 LiDAR 点云, 不经过 LIO) ==============
new_win "pc2laser" "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_simple_launch.py"

# ============== Cartographer 2D 建图 (无外部里程计, 纯 scan matching + IMU) ==============
# + TF→Odometry 桥接 (给 Nav2 导航用) 
new_win "Cartographer" "ros2 launch nav2_planner_bringup cartographer_simple_launch.py rviz:=true"

echo "===== 精简建图已启动 (会话: $SESSION) ====="
echo "tmux 窗口: GUI控制 | Gazebo | pc2laser | Cartographer"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "保存地图:"
echo "  docker exec lio_nav2 bash -c \"source /opt/ros/humble/setup.bash && source install/setup.bash && \\"
echo "    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \\"
echo "      '{filename: \\\"/ws/src/planner/nav2_planner_bringup/map/map_simple.pbstream\\\", include_unfinished_submaps: true}' && \\"
echo "    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory '{trajectory_id: 0}'\""

#!/usr/bin/env bash
# 精简导航 — 只用 Gazebo + Cartographer 纯定位 + Nav2
# 移除: FAST-LIO, lio_interface, sensor_scan_generation, KISS-Matcher
# 管线: Gazebo → /livox/lidar → pointcloud_to_laserscan → /scan → Cartographer 纯定位 → map→odom TF
#                                     map_server (加载 .pgm) → /map → Nav2
#
# 用法: docker exec -it lio_nav2 /ws/scripts/nav2_sim_carto_simple_docker.sh
# 查看: docker exec -it lio_nav2 tmux attach -t nav2_simple
# 停止: docker exec lio_nav2 tmux kill-session -t nav2_simple

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=nav2_simple
PBSTREAM="${1:-/ws/src/planner/nav2_planner/map/map_simple.pbstream}"

tmux kill-session -t "$SESSION" 2>/dev/null
killall -9 gzserver gzclient 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ---- 节点 ----

tmux new-session -d -s "$SESSION" -n "GUI控制" \
  "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && ros2 run gui_teleop gui_teleop_node; exec bash'"

new_win "Gazebo" "ros2 launch get_urdf get_urdf_launch.py"

# 3D→2D 切片
new_win "pc2laser" "ros2 launch nav2_planner pointcloud_to_laserscan_simple_launch.py"

# Cartographer 纯定位 + TF→Odometry
# 带 -pure_localization 命令行参数 + 加载 pbstream
new_win "Carto定位" "ros2 run cartographer_ros cartographer_node \
    -configuration_directory /ws/src/planner/nav2_planner/config \
    -configuration_basename cartographer_simple.lua \
    -load_state_filename $PBSTREAM \
    -pure_localization \
    --ros-args -r scan:=/scan -r imu:=/livox/imu -p use_sim_time:=true"

# 上一步 Cartographer 已 start occupancy_grid_node（封装在 cartographer_simple_launch 里）
# 这里单独启动 occupancy_grid 和 tf_to_odom
new_win "odom桥接" "ros2 run nav2_planner tf_to_odom.py --ros-args -p use_sim_time:=true"

# Nav2 导航
new_win "Nav2" "ros2 launch nav2_planner my_nav2_launch.py"

echo "===== 精简导航已启动 (会话: $SESSION) ====="
echo "tmux 窗口: GUI控制 | Gazebo | pc2laser | Carto定位 | odom桥接 | Nav2"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "定位操作:"
echo "  1. 等 Carto定位 窗口初始化完成"
echo "  2. RViz 中用 '2D Pose Estimate' 给初始位姿"
echo "  3. RViz 中用 'Nav2 Goal' 发送导航目标"
echo ""
echo "如果定位不准: 让机器人原地转几圈 + 重新给 Pose Estimate"

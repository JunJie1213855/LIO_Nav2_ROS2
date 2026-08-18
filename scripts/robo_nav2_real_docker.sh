#!/usr/bin/env bash
# 容器内 RoboSense Airy 数据集导航 (tmux 版)
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/robo_nav2_real_docker.sh [bag路径]
#   docker exec -it lio_nav2 tmux attach -t robo_nav2
#   docker exec lio_nav2 tmux kill-session -t robo_nav2
#
# 与建图脚本的区别: SLAM Toolbox → KISS-Matcher + Nav2
# 前置: 需要已建好地图 (.pgm + .yaml) 和 3D 点云 (.pcd)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=robo_nav2
BAG_DIR="${1:-/dataset/robosense/nav1}"

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ---- 先启动所有节点（等待 /clock 和传感器数据）----

tmux new-session -d -s "$SESSION" -n "FAST-LIO" \
  "bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
    ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py use_sim_time:=true rviz:=true; exec bash'"

new_win "robot_desc"   "ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false use_sim_time:=true"
new_win "lio_if"       "ros2 launch lio_interface lio_interface_launch.py use_sim_time:=true"
new_win "sensor"       "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
new_win "pc2laser"     "ros2 launch nav2_planner pointcloud_to_laserscan_launch_robo.py"

# KISS-Matcher 全局重定位（加载 .pcd 先验地图 → map→odom TF）
new_win "KISS+GICP"    "ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch_robo.py"

# Nav2 导航栈（加载 .pgm 静态地图 + 规划/控制）
new_win "Nav2"         "ros2 launch nav2_planner my_nav2_launch.py"

# RViz（导航视角: /map + /scan + /plan + TF + 代价地图）
new_win "RViz"         "ros2 run rviz2 rviz2 -d /ws/src/planner/nav2_planner/rviz/nav2.rviz"

# ---- 最后播放 bag ----
sleep 3

tmux new-window -t "$SESSION" -n "bag播放" \
  "bash -c 'source /opt/ros/humble/setup.bash && \
    echo \"播放: $BAG_DIR\" && \
    ros2 bag play $BAG_DIR --clock; exec bash'"

echo "===== RoboSense Airy 数据集导航 (会话: $SESSION) ====="
echo "窗口: FAST-LIO | robot_desc | lio_if | sensor | pc2laser | KISS+GICP | Nav2 | RViz | bag播放"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "前置条件:"
echo "  1. .pgm + .yaml 地图已在 my_nav2_launch.py 中配置"
echo "  2. dense_map.pcd 已在 KISS-Matcher launch 中配置"
echo "  3. 导航时在 RViz 中先给 '2D Pose Estimate' 初始位姿, 再给 'Nav2 Goal' 目标"

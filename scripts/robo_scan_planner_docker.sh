#!/usr/bin/env bash
# 容器内 RoboSense Airy 数据集 — FAST-LIO + SCAN-Planner 规划 (tmux)
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/robo_scan_planner_docker.sh [bag路径]
#   docker exec -it lio_nav2 tmux attach -t robo_scan_planner
#   docker exec lio_nav2 tmux kill-session -t robo_scan_planner

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=robo_scan_planner
BAG_DIR="${1:-/dataset/robosense/mapping}"

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ---- 先启动所有节点 ----

# FAST-LIO 里程计
tmux new-session -d -s "$SESSION" -n "FAST-LIO" \
  "bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
    ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py use_sim_time:=true rviz:=true; exec bash'"

# 机器人描述 + 静态 TF
new_win "robot_desc" "ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false use_sim_time:=true"

# 话题桥接: /Odometry → /LIO/odom_vehicle + /LIO/odom_imu
# cloud 直接用 /cloud_registered 不桥接
new_win "topic_relay" "python3 /tmp/odom_relay.py"

# SCAN-Planner 规划器（实机模式, lidar 传感器, 开环控制）
new_win "SCAN-Planner" "ros2 run scan_planner scan_planner_node \
    --ros-args \
    -p use_sim_time:=true \
    -p fsm.navi_mode:=1 \
    -p grid_map.sensor_type:=lidar \
    -p grid_map.cloud_is_world:=true \
    -p grid_map.need_extrinsic:=false \
    -r body_pose:=/LIO/odom_vehicle \
    -r sensor_pose:=/LIO/odom_imu \
    -r cloud:=/cloud_registered \
    -r move_base_simple/goal:=/move_base_simple/goal \
    -r initial_path:=/initial_path \
    --params-file /ws/install/scan_planner/share/scan_planner/config/planner.yaml"

# RViz（SCAN-Planner 可视化: 体素图 + 点云 + B-spline 轨迹）
new_win "RViz" "ros2 run rviz2 rviz2 -d /tmp/scan_planner.rviz"

# ---- 最后播放 bag ----
sleep 3

tmux new-window -t "$SESSION" -n "bag播放" \
  "bash -c 'source /opt/ros/humble/setup.bash && \
    echo \"播放: $BAG_DIR\" && \
    ros2 bag play $BAG_DIR --clock --loop; exec bash'"

echo "===== FAST-LIO + SCAN-Planner 数据集规划 (会话: $SESSION) ====="
echo "窗口: FAST-LIO | robot_desc | topic_relay | SCAN-Planner | RViz | bag播放"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "SCAN-Planner 话题桥接:"
echo "  /Odometry         → /LIO/odom_vehicle  (body_pose)"
echo "  /Odometry         → /LIO/odom_imu      (sensor_pose)"
echo "  /cloud_registered → SCAN-Planner 直接订阅"
echo ""
echo "在 RViz 中用 '2D Nav Goal' 发送目标点触发规划"

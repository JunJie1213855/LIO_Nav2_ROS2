#!/usr/bin/env bash
# 容器内 RoboSense Airy 数据集建图 — Point-LIO 版 (tmux)
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/robo_mapping_pointlio_docker.sh [bag路径]
#   docker exec -it lio_nav2 tmux attach -t robo_mapping_pointlio
#   docker exec lio_nav2 tmux kill-session -t robo_mapping_pointlio

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=robo_mapping_pointlio
BAG_DIR="${1:-/dataset/robosense/mapping}"

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ---- 先启动所有节点 ----

# Point-LIO（自带 RViz, robosenseAiry 配置）
tmux new-session -d -s "$SESSION" -n "Point-LIO" \
  "bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
    ros2 launch point_lio point_lio_robosenseAiry.launch.py rviz:=true; exec bash'"

# lio_interface — 用 pointlio 模式（话题: /aft_mapped_to_init → /cloud_registered）
new_win "robot_desc"   "ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false use_sim_time:=true"
new_win "lio_if"       "ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio use_sim_time:=true"
new_win "sensor"       "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
new_win "pc2laser"     "ros2 launch nav2_planner pointcloud_to_laserscan_launch_robo.py"
new_win "slam_toolbox" "ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner/config/slam_toolbox_params.yaml"
new_win "RViz"         "ros2 run rviz2 rviz2 -d /ws/src/planner/nav2_planner/rviz/nav2.rviz"

# ---- 最后播放 bag ----
sleep 3

tmux new-window -t "$SESSION" -n "bag播放" \
  "bash -c 'source /opt/ros/humble/setup.bash && \
    echo \"播放: $BAG_DIR\" && \
    ros2 bag play $BAG_DIR --clock; exec bash'"

echo "===== RoboSense Airy 数据集建图 — Point-LIO (会话: $SESSION) ====="
echo "窗口: Point-LIO | robot_desc | lio_if | sensor | pc2laser | slam_toolbox | RViz | bag播放"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "与 FAST-LIO 版的关键差异:"
echo "  1. Point-LIO 发布 /aft_mapped_to_init (Odometry) + /cloud_registered"
echo "  2. lio_interface 用 lio_type:=pointlio 自动匹配话题"
echo "  3. Point-LIO 自带 RViz (point_lio/rviz_cfg/pointlio_robosense.rviz)"
echo ""
echo "保存地图:"
echo "  docker exec lio_nav2 bash -c 'source install/setup.bash && /ws/scripts/save_map.sh'"
echo "  docker exec lio_nav2 bash -c 'source install/setup.bash && /ws/scripts/save_pcd.sh'"

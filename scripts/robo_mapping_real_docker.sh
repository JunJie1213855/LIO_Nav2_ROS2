#!/usr/bin/env bash
# 容器内 RoboSense Airy 数据集建图 (tmux 版)
# 用法:
#   挂载数据集: docker run ... -v /home/ros/dataset:/dataset ...
#   启动建图:   docker exec -it lio_nav2 /ws/scripts/robo_mapping_real_docker.sh [bag路径]
#   查看输出:   docker exec -it lio_nav2 tmux attach -t robo_mapping
#   停止:       docker exec lio_nav2 tmux kill-session -t robo_mapping
#
# 示例:
#   docker exec -it lio_nav2 /ws/scripts/robo_mapping_real_docker.sh /dataset/robosense/mapping

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=robo_mapping
BAG_DIR="${1:-/dataset/robosense/mapping}"

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

# ---- 先启动所有节点（等待 /clock 和传感器数据）----

# ================= FAST-LIO 里程计 + 3D RViz =================
tmux new-session -d -s "$SESSION" -n "FAST-LIO" \
  "bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
    ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py use_sim_time:=true rviz:=true; exec bash'"

# ================= 中间层 =================
new_win "robot_desc"    "ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false use_sim_time:=true"
new_win "lio_if"        "ros2 launch lio_interface lio_interface_launch.py use_sim_time:=true"
new_win "sensor"        "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ================= 3D 转 2D =================
new_win "pc2laser"      "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch_robo.py"

# ================= slam toolbox =================
new_win "slam_toolbox"  "ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

# ================= slam toolbox 可视化 =================
new_win "RViz"          "ros2 run rviz2 rviz2 -d /ws/src/planner/nav2_planner_bringup/rviz/nav2.rviz"

# ---- 最后播放 bag（所有节点就绪后开始推送数据）----
# 前面所有窗口不依赖 bag 数据即可启动，bag 延迟启动可以给各节点充分初始化时间

# new_win "bag播放" \
#   "bash -c 'source /opt/ros/humble/setup.bash && \
#     echo \"播放: $BAG_DIR\" && \
#     ros2 bag play $BAG_DIR --clock; exec bash'"

echo "===== RoboSense Airy 数据集建图 (会话: $SESSION) ====="
echo "窗口: FAST-LIO | robot_desc | lio_if | sensor | pc2laser | slam_toolbox | RViz | bag播放"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "保存地图:"
echo "  docker exec lio_nav2 bash -c 'source install/setup.bash && /ws/scripts/save_map.sh'"
echo "  docker exec lio_nav2 bash -c 'source install/setup.bash && /ws/scripts/save_pcd.sh'"

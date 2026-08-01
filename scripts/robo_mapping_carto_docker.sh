#!/usr/bin/env bash
# 容器内 RoboSense Airy 数据集建图 — Cartographer 版 (tmux)
# 用法:
#   docker exec -it lio_nav2 /ws/scripts/robo_mapping_carto_docker.sh
#   docker exec -it lio_nav2 tmux attach -t robo_mapping_carto
#   docker exec lio_nav2 tmux kill-session -t robo_mapping_carto

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

SESSION=robo_mapping_carto
BAG_DIR="${1:-/dataset/robosense/robosenseAiry-slamtoolbox}"

tmux kill-session -t "$SESSION" 2>/dev/null

new_win() {
  tmux new-window -t "$SESSION" -n "$1" \
    "bash -c 'cd $WORKSPACE_ROOT && source install/setup.bash && $2; exec bash'"
}

tmux new-session -d -s "$SESSION" -n "bag播放" \
  "bash -c 'source /opt/ros/humble/setup.bash && ros2 bag play $BAG_DIR/robosenseAiry-slamtoolbox_0.db3 --clock; exec bash'"

new_win "FAST-LIO" "ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py \
    use_sim_time:=true rviz:=true"

new_win "robot_desc" "ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false"
new_win "lio_if"     "ros2 launch lio_interface lio_interface_launch.py use_sim_time:=true"
new_win "sensor"     "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
new_win "pc2laser"   "ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"

# Cartographer 2D 建图
new_win "Cartographer" "ros2 launch me_nav2_bringup cartographer_mapping_launch.py"

# 建图 RViz（显示 /map + /scan + TF + 点云）
new_win "RViz" "ros2 run rviz2 rviz2 -d /ws/src/me_nav2_bringup/rviz/cartographer_mapping.rviz"

echo "===== RoboSense Airy 数据集建图 — Cartographer (会话: $SESSION) ====="
echo "窗口: bag播放 | FAST-LIO | robot_desc | lio_if | sensor | pc2laser | Cartographer | RViz"
echo "attach:  docker exec -it lio_nav2 tmux attach -t $SESSION"
echo "停止:    docker exec lio_nav2 tmux kill-session -t $SESSION"
echo ""
echo "保存地图:"
echo "  docker exec lio_nav2 bash -c 'source install/setup.bash && \\"
echo "    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \\"
echo "      \"{filename: \\\"/ws/src/me_nav2_bringup/map/airy_map.pbstream\\\", include_unfinished_submaps: true}\" && \\"
echo "    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \"{trajectory_id: 0}\" && \\"
echo "    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \\"
echo "      -pbstream_filename /ws/src/me_nav2_bringup/map/airy_map.pbstream \\"
echo "      -map_filestem /ws/src/me_nav2_bringup/map/airy_map'"

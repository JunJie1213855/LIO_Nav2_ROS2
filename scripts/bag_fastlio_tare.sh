#!/usr/bin/env bash
#
# bag_fastlio_tare.sh — Bag 回放 + FAST-LIO 里程计 + TARE 探索规划
#
# 用法:
#   ./scripts/bag_fastlio_tare.sh /path/to/bag [scenario]
#
#   scenario: 可选，默认 indoor，可选 garage/forest/campus/tunnel/matterport
#
# 数据流:
#   bag(/rslidar_points + /rslidar_imu_data)
#     → FAST-LIO → /cloud_registered + /Odometry
#       → lio_interface → /registered_scan + /registered_odometry
#         → TARE → /way_point

set -eo pipefail

BAG="${1:?用法: $0 <bag_path> [scenario]}"
SCENARIO="${2:-indoor}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

source install/setup.bash

echo "============================================"
echo " Bag + FAST-LIO + TARE Planner"
echo " bag:      $BAG"
echo " scenario: $SCENARIO"
echo "============================================"

# ------- Bag 回放 -------
gnome-terminal --title="Bag 回放" -- bash -c "
source install/setup.bash;
echo '播放 bag: $BAG';
ros2 bag play $BAG --clock"

sleep 2

# ------- FAST-LIO -------
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py use_sim_time:=true rviz:=false"

# ------- 中间层 -------
gnome-terminal --title="中间层(lio+sensor+pc2l)" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup middleware_launch.py use_sim_time:=True"

# ------- 机器人描述（TF 树） -------
gnome-terminal --title="机器人描述" -- bash -c "
source install/setup.bash;
ros2 launch gld_robot_description gld_robot_description_launch.py"

# ------- 静态 TF: map → odom（TARE Fixed Frame 是 map） -------
gnome-terminal --title="静态TF map->odom" -- bash -c '
source install/setup.bash;
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom'

# ------- TARE Planner -------
gnome-terminal --title="TARE 探索规划" -- bash -c "
source install/setup.bash;
export LD_LIBRARY_PATH=$WORKSPACE_ROOT/src/tare_planner/src/tare_planner/or-tools/lib:\$LD_LIBRARY_PATH;
ros2 launch tare_planner explore.launch use_sim_time:=True scenario:=$SCENARIO rviz:=true"

echo ""
echo "===== 全部启动完成 ====="
echo "TARE 自动开始探索 (kAutoStart: true)"
echo "查看规划路径: ros2 topic echo /way_point"
echo "查看探索子空间: ros2 topic echo /tare_visualizer/exploring_subspaces"

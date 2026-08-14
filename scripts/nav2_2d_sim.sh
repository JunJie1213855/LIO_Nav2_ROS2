#!/usr/bin/env bash
# 2D 单线 LiDAR 导航仿真 (Gazebo + Cartographer 2D SLAM + Nav2)
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=nav2_2d
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 世界文件（默认 indoor_2d 封闭室内场景，特征丰富 SLAM 稳定）
WORLD=${1:-indoor_2d}
WORLD_PATH=/ws/src/get_urdf/worlds/${WORLD}.world

killall -9 gzserver gzclient cartographer_node cartographer_occupancy_grid_node \
  planner_server controller_server bt_navigator behavior_server \
  lifecycle_manager_navigation rviz2 robot_state_publisher 2>/dev/null
sleep 2
tmux kill-session -t "$SESS" 2>/dev/null

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# Gazebo (spawn 2D 差分小车 diff_robot_2d)
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py robot:=diff_robot_2d world_path:=$WORLD_PATH rviz:=false; exec bash'"
sleep 8

# Cartographer 2D SLAM (在线建图 + map→odom TF)
W "Carto" "ros2 launch me_nav2_bringup cartographer_2d_launch.py"
sleep 4

# Nav2 在线导航 (无 map_server/AMCL)
W "Nav2" "ros2 launch me_nav2_bringup nav2_online_launch.py"
sleep 3

# RViz 2D 导航视图
W "RViz" "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_2d -p use_sim_time:=true -- -d /ws/src/me_nav2_bringup/rviz/nav2.rviz"

echo "========================================="
echo " 2D 单线 LiDAR 导航 (Cartographer + Nav2)"
echo " 窗口: Gazebo | Carto | Nav2 | RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " 操作: 切到 RViz 窗口, 用 '2D Goal Pose' 点目标"
echo "========================================="

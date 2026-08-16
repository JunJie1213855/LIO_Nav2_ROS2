#!/usr/bin/env bash
# 双目 + IMU VIO（vins_run 侧：VINS-Fusion + RViz）
# Gazebo 在 lio_nav2 容器运行（见 scripts/stereo_vio_sim.sh）
SESS=vins
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export LIBGL_ALWAYS_SOFTWARE=0

killall -9 vins_node rviz2 2>/dev/null
sleep 2
tmux kill-session -t "$SESS" 2>/dev/null

CONFIG=/ws/VINS_ROS2/src/VINS_ROS2/config/gazebo_stereo/gazebo_stereo.yaml
RVZ=/ws/VINS_ROS2/src/VINS_ROS2/config/vins_rviz_config.rviz

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'source /ws/VINS_ROS2/install/setup.bash && $2; exec bash'"; }

# VINS-Fusion 双目 + IMU
tmux new-session -d -s "$SESS" -n "VINS" \
  "bash -c 'source /ws/VINS_ROS2/install/setup.bash && ros2 run vins vins_node $CONFIG; exec bash'"
sleep 6

# RViz 可视化（轨迹 /vins_estimator/path、点云、特征点）
W "RViz" "ros2 run rviz2 rviz2 --ros-args -r __name:=rviz2_vins -p use_sim_time:=true -- -d $RVZ"

echo "========================================="
echo " 双目 + IMU VIO（VINS 侧）"
echo " 窗口: VINS | RViz"
echo " attach: docker exec -it vins_run tmux attach -t $SESS"
echo ""
echo " 提示: VIO 需要小车运动完成初始化，初始化成功后才输出轨迹"
echo "========================================="

#!/usr/bin/env bash
# 双目 + IMU VIO 定位验证（lio_nav2 侧：Gazebo + 键盘遥控）
# VINS 在 vins_run 容器运行（见 scripts/vins_run.sh）
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=stereo_vio
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

killall -9 gzserver gzclient rviz2 teleop_twist_keyboard robot_state_publisher 2>/dev/null
sleep 2
tmux kill-session -t "$SESS" 2>/dev/null

W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# Gazebo (spawn 双目小车 diff_robot_stereo，室内场景)
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py robot:=diff_robot_stereo world_path:=/ws/src/get_urdf/worlds/indoor_2d.world rviz:=false; exec bash'"
sleep 10

# 键盘遥控（切到此窗口后用键盘控制小车运动）
W "Teleop" "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

echo "========================================="
echo " 双目 + IMU VIO（Gazebo 侧）"
echo " 窗口: Gazebo | Teleop"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " 操作: 切到 Teleop 窗口，用键盘控制小车运动"
echo "       （VIO 需要运动才能初始化，建议前进+转向）"
echo "========================================="

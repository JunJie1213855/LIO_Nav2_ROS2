#!/usr/bin/env bash
# ============================================================
# Gazebo + FAST-LIO + FAR Planner 联合仿真
# FAR Planner：可见图（visibility graph）全局规划器
# 管线：Gazebo → FAST-LIO → lio_interface → sensor_scan_generation
#       → FAR Planner → /way_point → far_local_planner(VFH) → /cmd_vel
# ============================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=far_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============== 清理旧进程 / 旧 tmux 会话 =======================
killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  sensor_scan_generation_node far_planner waypoint_follower far_local_planner.py 2>/dev/null

tmux kill-session -t "$SESS" 2>/dev/null

# ============== 辅助函数：tmux 开窗执行命令 =======================
W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# ============== Gazebo 仿真环境 =======================
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"
sleep 6

# ============== FAST-LIO 里程计 =======================
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping_livox.launch.py rviz:=true use_sim_time:=true"
sleep 3

# ============== LIO 接口 + 扫描生成 =======================
W "lio_if"    "ros2 launch lio_interface fastlio_lio_interface_launch.py"
W "sensor"    "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
sleep 2

# ============== FAR Planner + far_local_planner =======================
W "FAR"       "ros2 launch me_nav2_bringup far_planner_lio_launch.py use_sim_time:=true"

# ============== RViz 可视化（FAR 自带 config，含 Goalpoint 插件）=======================
W "RViz"      "rviz2 -d $WS/src/planner/far_planner/src/far_planner/rviz/default.rviz"

# ============== 启动提示 =======================
echo "========================================="
echo " Gazebo + FAST-LIO + FAR Planner"
echo " 窗口: Gazebo | FAST-LIO | lio_if | sensor | FAR | RViz"
echo " attach: tmux attach -t $SESS"
echo ""
echo " 操作: RViz 里点 'Goalpoint' 按钮，再点地图设目标点"
echo "       FAR 会发布 /way_point，far_local_planner(VFH) 转 /cmd_vel"
echo "========================================="

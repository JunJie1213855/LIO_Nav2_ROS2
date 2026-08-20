#!/usr/bin/env bash
# ============================================================
# 真实 Airy bag 回放 + FAST-LIO + TARE Planner 探索测试
# 用法（两个终端）:
#   终端1:  ros2 bag play ~/dataset/robosenseAiry/mapping
#   终端2:  ./scripts/exploration_sim_robo.sh
# FAST-LIO 消费 bag 的 /rslidar_points + /rslidar_imu_data 做里程计，
# lio_interface → sensor_scan_generation 桥接出 /odom + /registered_scan，
# TARE 订阅它们做探索规划。
# ============================================================

# ============== 基础变量 / 环境设置 =======================
# 工作区根目录、tmux 会话名，以及 DDS 中间件（cyclonedds）
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=tare_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============== 清理旧进程 / 旧 tmux 会话 =======================
# 先杀掉上次运行残留的 Gazebo、各 LIO、TARE 等进程，避免端口/话题冲突
killall -9 gzserver gzclient fastlio_mapping pointlio_mapping \
  ign_sim_pointcloud_tool_node cloud_z_filter \
  tare_planner_node waypoint_follower lio_interface_node \
  sensor_scan_generation_node 2>/dev/null

tmux kill-session -t "$SESS" 2>/dev/null




# ============== 辅助函数：在 tmux 里开一个窗口执行命令 =======================
# W <窗口名> <命令>：每个模块放到独立的 tmux 窗口，方便单独查看日志
W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }


tmux new-session -d -s "$SESS"

# tmux new-session -d -s "$SESS" -n "rslidar-skd" \
#   "bash -c 'cd $WS && source install/setup.bash && ros2 launch rslidar_sdk driver_only.launch.py; exec bash'"

# ============== Fast-LIO 里程计 ========================
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py"


# ============== 静态 TF（URDF 链: base_footprint→chassis→livox_frame） ===============
# 注意：不能用 gld_robot_description（robot_state_publisher 需要 /joint_states 消息
# 才发布 TF，bag 回放里没有 joint_states → 实测不出 TF，tf2_echo 报 frame 不存在）。
# 真机能用是因为底盘控制器在发 /joint_states。这里按 URDF 里的 identity 变换手动补。
# lio_interface.cpp 初始化要 lookupTransform(base_footprint→livox_frame)，
# 缺这个 TF 它就不发 /registered_odometry + /registered_scan，整条 odom 链断掉，
# TARE 收不到 /odom → keypose_update=0 → 永远不探索。
W "robo_desc" "ros2 launch gld_robot_description robosense_description_launch.py"


# ============== LIO 接口（TF 桥接） =======================
W "lio_if"   "ros2 launch lio_interface fastlio_lio_interface_launch.py use_sim_time:=False"

# ============== 扫描生成（/registered_scan + /odom） =======================
W "sensor"   "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=False"


# ============== TARE 探索规划 =======================
# 订阅 /registered_scan 和 /odom，规划探索路径，发布 /way_point 目标点
W "TARE"     "ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=False"

# ============== RViz 可视化 =======================
# 查看探索路径(绿)、目标点(红)等可视化结果
W "RViz"     "rviz2 -d $WS/src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz"

# ============== 启动提示 =======================
echo "========================================="
echo " Airy bag 回放 + FAST-LIO + TARE Explorer"
echo " 窗口: FAST-LIO | tf_static | lio_if | sensor | TARE | RViz"
echo " attach: tmux attach -t $SESS"
echo ""
echo " TARE 会自动开始探索 (kAutoStart: true)"
echo " 观察 RViz 中的 exploration_path(绿) 和 way_point(红)"
echo "========================================="

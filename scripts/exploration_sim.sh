#!/usr/bin/env bash
# ============================================================
# Gazebo + Point-LIO + TARE Planner 联合仿真（探索模式）
# 与 gazebo_tare.sh 的区别：LIO 后端由 FAST-LIO 换成 Point-LIO。
# 仿真下 Point-LIO 需要 ign_sim_pointcloud_tool 先把 Gazebo 的 /livox/lidar
# 转成 velodyne 格式（注入 ring/time），再订阅 velodyne_points。
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

# ============== Gazebo 仿真环境 =======================
# 启动 indoor 世界，加载机器人 URDF，发布 /livox/lidar(PointCloud2) 和 /livox/imu
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"


# ============== 点云格式转换器 =======================
# 把 Gazebo 的 /livox/lidar 转成 /velodyne_points，
# 并注入 Point-LIO(Velodyne 模式) 必需的 ring(线号) 和 time(单点时间戳) 字段
# W "convert"  "ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args -p pcd_topic:=/livox/lidar -p n_scan:=50 -p horizon_scan:=360 -p ang_bottom:=7.22 -p ang_res_y:=1.248"

# ============== Point-LIO 里程计 =======================
# 订阅 /velodyne_points + /livox/imu，输出里程计 /aft_mapped_to_init 和点云 /cloud_registered
# W "Point-LIO" "ros2 launch point_lio point_lio.launch.py rviz:=true point_lio_cfg_dir:=$WS/src/localization/point_lio/config/mid360_sim.yaml"

# ============== Fast-LIO 里程计 ========================
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping_livox.launch.py rviz:=true use_sim_time:=true"


# ============== LIO 接口（TF 桥接） =======================
# lio_type:=pointlio 表示订阅 /aft_mapped_to_init，转成标准 odom 坐标系 TF
# W "lio_if"   "ros2 launch lio_interface pointlio_lio_interface_launch.py"

W "lio_if"   "ros2 launch lio_interface fastlio_lio_interface_launch.py"

# ============== 扫描生成（/registered_scan + /odom） =======================
# 发布 /registered_scan（odom 帧点云）和 /odom，供 TARE 规划与 waypoint_follower 使用
W "sensor"   "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"


# ============== TARE 探索规划 =======================
# 订阅 /registered_scan 和 /odom，规划探索路径，发布 /way_point 目标点
# W "TARE"     "ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true"

# ============== RViz 可视化 =======================
# 查看探索路径(绿)、目标点(红)等可视化结果
W "RViz"     "rviz2 -d $WS/src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz"

# ============== 启动提示 =======================
echo "========================================="
echo " Gazebo + Point-LIO + TARE Explorer"
echo " 窗口: Gazebo | convert | Point-LIO | lio_if | sensor | TARE | RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " TARE 会自动开始探索 (kAutoStart: true)"
echo " 观察 RViz 中的 exploration_path(绿) 和 way_point(红)"
echo "========================================="

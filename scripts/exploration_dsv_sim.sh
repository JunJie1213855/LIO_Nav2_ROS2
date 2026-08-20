#!/usr/bin/env bash
# ============================================================
# Gazebo + FAST-LIO + DSV Planner 联合仿真（探索模式）
#
# 管线:
#   Gazebo(simple_car) → ign_sim_pointcloud_tool(/livox/lidar → /velodyne_points, 注入 ring/time)
#     → FAST-LIO(/velodyne_points + /livox/imu, config: mid360_sim.yaml)
#     → lio_interface(odom→base_footprint TF)
#     → sensor_scan_generation(/odom + /registered_scan, odom 帧点云)
#     → topic_tools relay:
#         /odom           → /state_estimation
#         /odom           → /state_estimation_at_scan
#         /registered_scan→ /terrain_map_ext       (地形高程/grid 用，保留地面点)
#         /next_goal      → /way_point
#     → ground_ceiling_filter(/terrain_map_ext → /terrain_map_ext_filtered, 滤除 z<0.25 地面点)
#         ★ 关键: DSV 的 ground-robot RRT 在地面高度(z≈0)规划，若地面点被 octomap 标成 occupied
#           (z≈0.175 单元)，RRT 扩展线被地面挡住 → 只长几个节点 → gainFound()=false
#           → 误判 "Exploration completed"。滤除地面后该层变 free/unknown，RRT 才能生长。
#         ★ /terrain_map_ext 必须保持未滤除: frontier 的地形高程(getZvalue)和 grid 依赖地面点。
#     → dsvp_launch(exploration + dsvplanner_exe + graph_planner + rviz)
#         octomap 订阅 /terrain_map_ext_filtered (octomap.yaml: octo/velodyne_cloud_topic)
#     → waypoint_follower(/way_point → /cmd_vel, 带局部避障)
#
# DSV 需要的输入话题(exploration.yaml):
#   /state_estimation(odom)、/terrain_map_ext(地形点云) —— 由 relay 提供
#   autoExp: true → 收到 odom + 点云后自动开始探索，无需手动发 /start_exploring
# ============================================================

# ============== 基础变量 / 环境设置 =======================
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=dsv_gz
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 世界文件：默认 test_world.world，可用 `WORLD=indoor_office.world ./dsv_exploration_sim.sh` 切换
WORLD="${WORLD:-test_world.world}"

# ============== 清理旧进程 / 旧 tmux 会话 =======================
killall -9 gzserver gzclient fastlio_mapping lio_interface_node \
  sensor_scan_generation_node exploration navigationBoundary \
  dsvplanner_exe graph_planner_exe waypoint_follower.py relay 2>/dev/null

tmux kill-session -t "$SESS" 2>/dev/null

# ============== 辅助函数：在 tmux 里开一个窗口执行命令 =======================
W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# ============== Gazebo 仿真环境 =======================
# 启动世界，加载 simple_car(差速底盘, 订阅 /cmd_vel)，发布 /livox/lidar + /livox/imu
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false world_path:=$WS/src/get_urdf/worlds/$WORLD; exec bash'"

# ============== 点云格式转换 =======================
# Gazebo ray 传感器 /livox/lidar 缺 ring/time 字段，FAST-LIO(Velodyne 模式) 无法处理而发散
# ign_sim_pointcloud_tool 按垂直角注入 ring，按扫描行注入 time(秒)，输出 /velodyne_points
W "convert" "ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args -p pcd_topic:=/livox/lidar -p n_scan:=50 -p horizon_scan:=360 -p ang_bottom:=7.22 -p ang_res_y:=1.248"

# ============== FAST-LIO 里程计 ========================
# 订阅 /velodyne_points + /livox/imu，输出 /Odometry 和 /cloud_registered
# 用源码绝对路径传 config（symlink-install 不会自动同步新文件到 install/share，相对路径会加载失败）
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping_livox.launch.py config_file:=$WS/src/localization/FAST_LIO_ROBOAIRY/config/mid360_sim.yaml rviz:=true use_sim_time:=true"

# ============== LIO 接口（TF 桥接） =======================
# odom → base_footprint TF
W "lio_if"   "ros2 launch lio_interface fastlio_lio_interface_launch.py"

# ============== 扫描生成（/registered_scan + /odom） =======================
# 发布 /registered_scan(odom 帧点云) 和 /odom
W "sensor"   "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ============== map→odom 静态 TF =======================
# DSV 的 grid/graph 在 map 帧下规划，需要 map→odom(与 lio_interface 的 odom→base_footprint 相接)
W "tf_map"   "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"

# ============== DSV 需要的话题中继 =======================
# dsvplanner 订阅 /state_estimation + /terrain_map_ext；发布 /next_goal
# waypoint_follower 消费 /way_point
W "relays" "ros2 run topic_tools relay /odom /state_estimation & \
  ros2 run topic_tools relay /odom /state_estimation_at_scan & \
  ros2 run topic_tools relay /registered_scan /terrain_map_ext & \
  ros2 run topic_tools relay /next_goal /way_point & \
  wait"

# ============== 地面滤除（octomap 专用） =======================
# /terrain_map_ext → 滤除 z<0.25 的地面点 → /terrain_map_ext_filtered
# 只喂给 octomap(octo/velodyne_cloud_topic)，避免地面形成 occupied 层挡住 RRT
# 输入用 body 帧阈值(flat 车 ≈ odom z)，ceil 置 100 表示不滤上界
W "groundfilt" "python3 $WS/scripts/ground_ceiling_filter.py --ros-args \
  -p input_cloud:=/terrain_map_ext -p input_odom:=/odom \
  -p output_cloud:=/terrain_map_ext_filtered \
  -p ground_z_threshold:=0.25 -p ceiling_z_threshold:=100.0"

# ============== waypoint_follower（执行 /way_point → /cmd_vel） =======================
# 复用 tare_planner 的 waypoint_follower.py（带局部避障），参数与 tare_planner_lio_launch.py 一致
W "follower" "ros2 run tare_planner waypoint_follower.py --ros-args \
  -p use_sim_time:=true -p max_linear_vel:=0.5 -p max_angular_vel:=1.0 \
  -p arrival_dist:=0.3 -p kp_linear:=0.8 -p kp_angular:=2.0 \
  -p stop_dist:=0.45 -p slow_dist:=0.8 -p robot_half_width:=0.35 \
  -p check_height_min:=0.05 -p check_height_max:=0.60"

# ============== DSV 探索规划 =======================
# exploration + dsvplanner_exe + graph_planner + rviz(default.rviz)
# autoExp: true → 收到 /state_estimation 和 /terrain_map_ext 后自动开始探索
W "DSV"      "ros2 launch dsvp_launch dsvp.launch"

# ============== 启动提示 =======================
echo "========================================="
echo " Gazebo + FAST-LIO + DSV Planner Explorer"
echo " 窗口: Gazebo | convert | FAST-LIO | lio_if | sensor | tf_map | relays | groundfilt | follower | DSV"
echo " attach: tmux attach -t $SESS"
echo ""
echo " DSV autoExp: true → 自动开始探索"
echo " RViz: 观察 new_tree_path(绿), next_goal, global_frontier 等"
echo " 调试:"
echo "   ros2 topic hz /state_estimation"
echo "   ros2 topic hz /terrain_map_ext"
echo "   ros2 topic echo /way_point"
echo "   ros2 topic echo /cmd_vel"
echo "========================================="

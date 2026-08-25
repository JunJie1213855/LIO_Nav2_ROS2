#!/usr/bin/env bash
# ============================================================
# Gazebo + FAST-LIO + explore_lite (m-explore-ros2) 联合仿真（前沿探索）
# 与 exploration_tare_sim.sh 的区别：探索前端由 TARE 换成 explore_lite。
# explore_lite 是前沿( frontier )探索器：在代价地图里找"已知/未知"交界，
# 用 Nav2 的 NavigateToPose action 逐个送目标点。
#
# 数据流（蓝色是 explore_lite 必需的三条腿）：
#   Gazebo(/livox/lidar) → FAST-LIO(/cloud_registered)
#     → lio_interface(odom→base_footprint TF)
#     → sensor_scan_generation(/registered_scan, /odom)
#     → pointcloud_to_laserscan(/scan)
#     → slam_toolbox(/map + map→odom TF)          ← 实时 2D 地图（含未知区）
#     → Nav2 navigation_launch（无静态地图）→ /navigate_to_pose action
#            全局 costmap 的 static_layer 直接订阅 slam_toolbox 的 /map
#     → explore_lite 订阅 /global_costmap/costmap 找前沿 + 发 NavigateToPose
#
# 关键点：
#  1) 一定不要用 my_nav2_launch.py —— 它带 map_server 加载静态图 test_map__2.yaml，
#     会跟 slam_toolbox 抢 /map 话题；一旦 explore_lite 读到那张"全已知"静态图，
#     没有未知格 → 找不到前沿 → 直接结束探索。
#  2) explore_lite 的 robot_base_frame 必须改成 base_footprint（库默认 base_link）。
#  3) Nav2 的 navigation_launch 从 nav2_bringup 直接起，autostart:=true 自动激活。
# ============================================================

# ============== 基础变量 / 环境设置 =======================
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=explore_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============== 清理旧进程 / 旧 tmux 会话 =======================
# 杀掉上次运行残留的 Gazebo、LIO、SLAM、Nav2、explore 等进程
killall -9 gzserver gzclient fastlio_mapping \
  ign_sim_pointcloud_tool_node \
  lio_interface_node sensor_scan_generation_node \
  pointcloud_to_laserscan_node slam_toolbox \
  explore \
  map_server bt_navigator controller_server planner_server \
  smoother_server behavior_server velocity_smoother waypoint_follower \
  costmap_2d_cloud costmap_2d_markers lifecycle_manager \
  rviz2 2>/dev/null

tmux kill-session -t "$SESS" 2>/dev/null

# ============== 辅助函数：在 tmux 里开一个窗口执行命令 =======================
W() { tmux new-window -t "$SESS" -n "$1" \
  "bash -c 'cd $WS && source install/setup.bash && $2; exec bash'"; }

# ============== Gazebo 仿真环境 =======================
# 启动默认 indoor 世界，加载机器人 URDF，发布 /livox/lidar(PointCloud2) 和 /livox/imu
tmux new-session -d -s "$SESS" -n "Gazebo" \
  "bash -c 'cd $WS && source install/setup.bash && ros2 launch get_urdf get_urdf_launch.py rviz:=false; exec bash'"

# ============== 点云格式转换器 =======================
# Gazebo 的 /livox/lidar 只有 xyz+intensity；FAST-LIO 用 lidar_type:2(Velodyne) 需要
# ring/time 字段 → ign_sim_pointcloud_tool 注入 ring/time，重发到 /velodyne_points
W "convert"  "ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args -p pcd_topic:=/livox/lidar -p n_scan:=50 -p horizon_scan:=360 -p ang_bottom:=7.22 -p ang_res_y:=1.248"

# ============== Fast-LIO 里程计 ========================
# 订阅 /velodyne_points（转换后的 velodyne 格式）+ /livox/imu，输出 /Odometry 和 /cloud_registered。
# 必须用 mid360_sim.yaml（lid_topic:=/velodyne_points, lidar_type:=2），不能用默认 mid360.yaml。
W "FAST-LIO" "ros2 launch fast_lio_robosense mapping_livox.launch.py config_file:=mid360_sim.yaml rviz:=false use_sim_time:=true"

# ============== LIO 接口（TF 桥接） =======================
# 把 FAST-LIO 的 /Odometry 转成标准 odom→base_footprint TF
W "lio_if"   "ros2 launch lio_interface fastlio_lio_interface_launch.py"

# ============== 扫描生成（/registered_scan + /odom） =======================
# 发布 /registered_scan（odom 帧点云）和 /odom，供 3D→2D 切片使用
W "sensor"   "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ============== 3D 点云 → 2D 激光 =======================
# 把 /registered_scan 切片成 /scan（Pointcloud2d_3d.yaml，target_frame:=livox_frame）
W "laserscan" "ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

# ============== SLAM 建图 =======================
# slam_toolbox online_async：吃 /scan，发布实时 /map（含未知区）+ map→odom TF。
# base_frame:=base_footprint、map_frame:=map、odom_frame:=odom
W "slam"     "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WS/src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

# ============== Nav2 导航栈（无静态地图） =======================
# 只起 navigation_launch（controller/planner/bt_navigator/costmaps），提供
# /navigate_to_pose action。全局 costmap 的 static_layer 订阅 slam 的 /map，
# 因此实时地图 + 未知区一路贯通到 explore_lite。
# 注意 use_composition 必须传 Python 布尔大写 False/True：navigation_launch.py:112
# 用 PythonExpression(['not ', use_composition]) 拼字符串再 eval，小写 false 会
# 报 "name 'false' is not defined"。
W "Nav2"     "ros2 launch nav2_bringup navigation_launch.py params_file:=$WS/src/planner/nav2_planner_bringup/config/nav2_params.yaml use_sim_time:=true autostart:=true use_composition:=False"

# ============== explore_lite 前沿探索 =======================
# 订阅 /global_costmap/costmap 找前沿，向 Nav2 的 /navigate_to_pose 送目标点。
# robot_base_frame 必须覆盖为 base_footprint（库默认 base_link）。
# 参数对齐 explore/config/params_costmap.yaml，另开 return_to_init 用后回起点。
W "explore"  "ros2 run explore_lite explore --ros-args \
  -p use_sim_time:=true \
  -p robot_base_frame:=base_footprint \
  -p costmap_topic:=/global_costmap/costmap \
  -p costmap_updates_topic:=/global_costmap/costmap_updates \
  -p return_to_init:=true \
  -p visualize:=true \
  -p planner_frequency:=0.15 \
  -p progress_timeout:=30.0 \
  -p potential_scale:=3.0 \
  -p orientation_scale:=0.0 \
  -p gain_scale:=1.0 \
  -p transform_tolerance:=0.3 \
  -p min_frontier_size:=0.5"

# ============== RViz 可视化 =======================
# 显示 /map、全局/局部代价地图、/odom 等；如需看前沿，手动添加
# MarkerArray 显示，话题填 /explore/frontiers
W "RViz"     "rviz2 -d $WS/src/planner/nav2_planner_bringup/rviz/nav2.rviz"

# ============== 启动提示 =======================
echo "========================================="
echo " Gazebo + FAST-LIO + explore_lite Explorer"
echo " 窗口: Gazebo | convert | FAST-LIO | lio_if | sensor | laserscan | slam | Nav2 | explore | RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " explore_lite 会自动探索 (导航至前沿目标点)"
echo " 控制暂停/恢复:  ros2 topic pub -1 /explore/resume std_msgs/Bool \"{data: false}\""
echo " 状态话题:        /explore/status"
echo " 前沿可视化:      RViz 添加 MarkerArray → /explore/frontiers"
echo " 注意: 不要用 my_nav2_launch.py(带静态地图)，会抢 /map 导致探索立即结束"
echo "========================================="

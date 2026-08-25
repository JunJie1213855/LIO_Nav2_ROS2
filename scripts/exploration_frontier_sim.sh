#!/usr/bin/env bash
# ============================================================
# Gazebo + FAST-LIO + frontier_exploration 联合仿真（前沿+质心探索）
# 与 exploration_explore_lite_sim.sh 的区别：探索前端由 explore_lite 换成
# frontier_exploration（src/planner/frontier-occ-exploration，ROS1→ROS2 移植版）。
# frontier_planner 也是前沿探索器，但走"裸 /map + 自身膨胀"路线：
#   订阅 /map(slam_toolbox) → InflateMap(膨胀0.3m) → 前沿单元 → 分组 → 算质心
#   → 代价函数(距离×碰撞惩罚)选目标 → 直连 Nav2 /navigate_to_pose 逐个送达
#   → 到位原地旋转 Rotation() → 无前沿 → 返航退出。
#
# 与 explore_lite 的三点不同：
#   · frontier_planner 读裸 /map 自己做膨胀，不依赖 /global_costmap/costmap
#   · 无 /explore/resume 暂停话题，停止 = Ctrl+C（阻塞式主循环）
#   · 启动即阻塞等待 Nav2 的 /navigate_to_pose 动作服务器（wait_for_action_server）
#
# 数据流：
#   Gazebo(/livox/lidar) → FAST-LIO(/cloud_registered)
#     → lio_interface(odom→base_footprint TF)
#     → sensor_scan_generation(/registered_scan, /odom)
#     → pointcloud_to_laserscan(/scan)
#     → slam_toolbox(/map + map→odom TF)          ← 实时 2D 地图（含未知区）
#     → Nav2 navigation_launch（无静态地图）→ /navigate_to_pose action
#     → frontier_planner 订阅 /map 找前沿+质心 → 发 NavigateToPose
#
# 关键点：
#  1) 一定不要用 my_nav2_launch.py —— 它带 map_server 加载静态图 test_map__2.yaml，
#     会跟 slam_toolbox 抢 /map；frontier_planner 读到全已知图 → 无未知格 → 无前沿。
#  2) frontier_planner 的 robot_base_frame 必须覆盖为 base_footprint（默认 base_link）。
#  3) Nav2 的 navigation_launch 从 nav2_bringup 直接起，autostart:=true 自动激活；
#     use_composition 必须传 Python 布尔大写 False/True（navigation_launch.py:112
#     用 PythonExpression 拼字符串再 eval，小写 false 会报 "name 'false' is not defined"）。
#  4) Rotation() 会直接发布 /cmd_vel，只在启动/刚到达目标（Nav2 空闲）时执行，
#     与 DWB 的控制输出偶尔交错但无碍探索。
# ============================================================

# ============== 基础变量 / 环境设置 =======================
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname -- "$SCRIPT_DIR")"
SESS=frontier_gz
export LIBGL_ALWAYS_SOFTWARE=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============== 清理旧进程 / 旧 tmux 会话 =======================
# 杀掉上次运行残留的 Gazebo、LIO、SLAM、Nav2、frontier 等进程
killall -9 gzserver gzclient fastlio_mapping \
  ign_sim_pointcloud_tool_node \
  lio_interface_node sensor_scan_generation_node \
  pointcloud_to_laserscan_node slam_toolbox \
  frontier_planner \
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
# 订阅 /velodyne_points + /livox/imu，输出 /Odometry 和 /cloud_registered。
# 必须用 mid360_sim.yaml（lid_topic:=/velodyne_points, lidar_type:=2）。
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
W "slam"     "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WS/src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

# ============== Nav2 导航栈（无静态地图） =======================
# 只起 navigation_launch（controller/planner/bt_navigator/costmaps），提供
# /navigate_to_pose action。全局 costmap 的 static_layer 订阅 slam 的 /map。
W "Nav2"     "ros2 launch nav2_bringup navigation_launch.py params_file:=$WS/src/planner/nav2_planner_bringup/config/nav2_params.yaml use_sim_time:=true autostart:=true use_composition:=False"

# ============== frontier_planner 前沿探索 =======================
# 订阅 /map（slam_toolbox）→ 自身膨胀 → 前沿/质心 → 代价选点 → /navigate_to_pose。
# robot_base_frame 必须覆盖为 base_footprint（库默认 base_link）。
# 参数对齐 docs/USAGE.md §5：obstacle_inflation / goal_tolerance / obstacle_tolerance / rotate_speed
# 先等 Nav2 的 bt_navigator 进入 active 再启动 frontier_planner。
# 原因：frontier 启动比 Nav2 激活早约 1s，此时发的第一个 /navigate_to_pose 目标会被
# 未激活的 bt_navigator 静默拒绝 → goal_handle 为 null → GetGoalStatus() 永远 UNKNOWN
# → 主循环死等 180s 才换目标（表现为"重启后前 3 分钟机器人不动"）。
W "frontier" "for i in \$(seq 1 60); do \
    [[ \"\$(ros2 lifecycle get /bt_navigator 2>/dev/null)\" == active* ]] && break; sleep 1; \
  done; \
  ros2 run frontier_exploration frontier_planner --ros-args \
  -p use_sim_time:=true \
  -p robot_base_frame:=base_footprint \
  -p obstacle_inflation:=0.3 \
  -p map_revolution:=0.1 \
  -p cmd_topic:=cmd_vel \
  -p goal_tolerance:=0.3 \
  -p obstacle_tolerance:=0.5 \
  -p rotate_speed:=0.5"

# ============== RViz 可视化 =======================
# 自带 frontier_exploration.rviz 已配好 /map /inflated_map /frontier_vis(蓝)
# /centroid_vis(红) /goal_vis(粉球) /home_vis(绿球) 显示
W "RViz"     "rviz2 -d $WS/src/planner/frontier-occ-exploration/frontier_exploration_occupancygrid/config/rviz/frontier_exploration.rviz"

# ============== 启动提示 =======================
echo "========================================="
echo " Gazebo + FAST-LIO + frontier_exploration Explorer"
echo " 窗口: Gazebo | convert | FAST-LIO | lio_if | sensor | laserscan | slam | Nav2 | frontier | RViz"
echo " attach: docker exec -it lio_nav2 tmux attach -t $SESS"
echo ""
echo " frontier_planner 会自动探索 (导航至前沿质心)"
echo " 话题: /frontier(前沿点) /centroids(质心) /inflated_map(膨胀图)"
echo " 可视化: /frontier_vis(蓝) /centroid_vis(红) /goal_vis(粉) /home_vis(绿)"
echo " 服务:  ros2 service call /get_centroids frontier_exploration/srv/GetCentroids \"{}\""
echo " 日志:  'Reached the goal!' = 到达一个前沿目标; 探索完自动返航"
echo " 注意: 不要用 my_nav2_launch.py(带静态地图)，会抢 /map 导致探索立即结束"
echo "========================================="

#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# ╔══════════════════════════════════════════════════════════════════════╗
# ║  Airy 实机建图 (Cartographer 后端)                                    ║
# ║                                                                      ║
# ║  数据流：                                                            ║
# ║  FAST-LIO → /cloud_registered → lio_interface → /registered_scan     ║
# ║    → sensor_scan_generation → odom→base_footprint TF + /odom         ║
# ║      → pointcloud_to_laserscan → /scan (livox_frame)                 ║
# ║        → Cartographer 2D 建图 (/map)                                 ║
# ║                                                                      ║
# ║  TF 树：                                                            ║
# ║    map→odom (cartographer 发布) + odom→base_footprint (LIO)          ║
# ║    + base_footprint→chassis→livox_frame (URDF)                       ║
# ║                                                                      ║
# ║  Pointcloud2d_3d.yaml 切片：min_height=0.3, max_height=2.0           ║
# ╚══════════════════════════════════════════════════════════════════════╝

# gnome-terminal --title="robosense lidar SDK" -- bash -c "
# source install/setup.bash;
# ros2 launch rslidar_sdk driver_only.launch.py"

# ================ fast-lio ================
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py \
  use_sim_time:=True"

# ================ lio_interface ================
gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface fastlio_lio_interface_launch.py \
  use_sim_time:=True"

# ================ 中间一些必要的节点 ================
gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch gld_robot_description robosense_description_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=True"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch_zlim.py use_sim_time:=True"

# =================== cartographer 建图 ===================
gnome-terminal --title="cartographer 建图" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup cartographer_2d_launch.py use_sim_time:=True"

# ================ cartographer 建图可视化 ================
gnome-terminal --title="cartographer 建图可视化" -- bash -c "
source install/setup.bash;
rviz2 -d src/gld_robot_description/rviz/nav2.rviz"

# gnome-terminal --title="Nav2 导航" -- bash -c "
# source install/setup.bash;
# ros2 launch nav2_planner_bringup my_nav2_launch.py"

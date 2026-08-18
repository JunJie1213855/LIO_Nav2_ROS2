#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# ╔══════════════════════════════════════════════════════════════════════╗
# ║  Airy 实机建图（方案一：airy_unflip.py 修正 Z 轴翻转）               ║
# ║                                                                      ║
# ║  数据流：                                                            ║
# ║  FAST-LIO → /cloud_registered (Z 翻转)                               ║
# ║    → airy_unflip.py → /cloud_registered_unflipped (Z 正常朝上)       ║
# ║      → lio_interface → /registered_scan                              ║
# ║        → sensor_scan_generation → pointcloud_to_laserscan            ║
# ║          → /scan → SLAM Toolbox                                      ║
# ║                                                                      ║
# ║  Pointcloud2d_3d.yaml 使用标准正值：min_height=0.2, max_height=1.0  ║
# ╚══════════════════════════════════════════════════════════════════════╝


# ================ rslidar 驱动 ================
# gnome-terminal --title="robosense lidar SDK" -- bash -c "
# source install/setup.bash;
# ros2 launch rslidar_sdk driver_only.launch.py"

# ================ 建图定位算法 1：fast-lio2 ================
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py"

# =============== 建图定位算法 2：super-lio =================
# gnome-terminal --title="Super-LIO 里程计" -- bash -c "
# source install/setup.bash;
# ros2 launch super_lio robosense_airy.py"

# ================ robosense airy 描述 ================
gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch gld_robot_description robosense_description_launch.py"

# ================ 中间层 2 : fast-lio2 ================
gnome-terminal --title="中间层(lio + sensor + pc2l)" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner middleware_launch.py use_sim_time:=False"


# ================ 中间层 2 : super lio ================
# gnome-terminal --title="中间层(lio + sensor + pc2l)" -- bash -c "
# source install/setup.bash;
# ros2 launch nav2_planner middleware_launch_superlio.py use_sim_time:=False"

# ================ slam toolbox 建图 ================
# gnome-terminal --title="slam_toolbox 建图" -- bash -c "
# source install/setup.bash;
# ros2 launch slam_toolbox online_async_launch.py \
#     slam_params_file:=src/nav2_planner/config/slam_toolbox_params.yaml"

gnome-terminal --title="slam_toolbox 建图" -- bash -c "
source install/setup.bash;
ros2 launch slam_toolbox online_async_launch.py"

# ================ slam toolbox 建图可视化 ================
gnome-terminal --title="slam_toolbox 建图可视化" -- bash -c "
source install/setup.bash;
rviz2 -d src/gld_robot_description/rviz/nav2.rviz"

# ================ Nav2 导航 ================
# gnome-terminal --title="Nav2 导航" -- bash -c "
# source install/setup.bash;
# ros2 launch nav2_planner my_nav2_launch.py"

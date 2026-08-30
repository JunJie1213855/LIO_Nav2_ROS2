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

# point_lio
# gnome-terminal --title="Livox Point-LIO 驱动" -- bash -c "
# source install/setup.bash;
# ros2 launch livox_ros_driver2 point_lio_msg_MID360_launch.py"

# gnome-terminal --title="Point-LIO 里程计" -- bash -c "
# source install/setup.bash;
# ros2 launch point_lio point_lio.launch.py \
#   point_lio_cfg_dir:=/home/pio/Nav2_3D_ws/src/localization/point_lio/config/mid360_real.yaml"

# gnome-terminal --title="Point-LIO lio_interface" -- bash -c "
# source install/setup.bash;
# ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio"


# fast_lio_robosense
# gnome-terminal --title="Livox Fast-LIO 驱动" -- bash -c "
# source install/setup.bash;
# ros2 launch livox_ros_driver2 fast_lio_msg_MID360_launch.py"

gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py"

# Z 轴翻转修正：施加 extrinsic_R 逆旋转，恢复 Z 轴朝上
gnome-terminal --title="Airy Z轴翻转修正" -- bash -c "
source install/setup.bash;
/usr/bin/python3 $WORKSPACE_ROOT/scripts/airy_unflip.py \
  --ros-args -p use_sim_time:=False"

# lio_interface 订阅修正后的点云（Z 轴已恢复朝上）
gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py \
  cloud_topic:=/cloud_registered_unflipped use_sim_time:=False"

# ---------


gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch gld_robot_description gld_robot_description_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch.py"

# gnome-terminal --title="slam_toolbox 建图" -- bash -c "
# source install/setup.bash;
# ros2 launch slam_toolbox online_async_launch.py"

# gnome-terminal --title="slam_toolbox 建图" -- bash -c "
# source install/setup.bash;
# ros2 launch slam_toolbox online_async_launch.py \
#     slam_params_file:=src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

gnome-terminal --title="slam_toolbox 建图" -- bash -c "
source install/setup.bash;
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner_bringup/config/slam_toolbox_params.yaml"

gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup my_nav2_launch.py"

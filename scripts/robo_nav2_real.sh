#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1


# fast_lio_robosense
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py"

gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

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
ros2 launch nav2_planner_bringup pointcloud_to_laserscan_launch_robo.py"


gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
source install/setup.bash;
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch_robo.py"

gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup my_nav2_launch.py"

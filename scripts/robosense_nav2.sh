#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 实机导航启动脚本

# ========== robosense lidar SDK ==========
# gnome-terminal --title="robosense lidar SDK" -- bash -c "
# source install/setup.bash;
# ros2 launch rslidar_sdk driver_only.launch.py"

# sleep 3

# ========== 建图 fast lio ==========
# fast_lio
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py"

# 中间层（三个节点合并为一个 launch，省 ~72MB）
gnome-terminal --title="中间层(lio+sensor+pc2l)" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup middleware_launch.py use_sim_time:=False"

# 机器人描述
gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch gld_robot_description robosense_description_launch.py"

# ============== 静态变换 ==============
gnome-terminal --title="tf_correction"  -- bash -c "
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0.7071 --qy -0.7071 --qz 0 --qw 0 \
--frame-id base_footprint --child-frame-id base_footprint_nav"

# ========== 重定位 ==========
# gnome-terminal --title="small_gicp 重定位" -- bash -c "
# source install/setup.bash;
# ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py"

gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
source install/setup.bash;
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch_real.py"


# ========== nav2 导航 ==========
gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner_bringup my_nav2_launch.py use_sim_time:=False"
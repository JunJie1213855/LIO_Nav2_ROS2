#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# ╔══════════════════════════════════════════════════════════════════════╗
# ║  Airy 实机建图（Point-LIO）                                         ║
# ║                                                                      ║
# ║  数据流：                                                            ║
# ║  rslidar 驱动 → /rslidar_points + /rslidar_imu_data                  ║
# ║    → Point-LIO → /cloud_registered + /aft_mapped_to_init (Z 朝上)    ║
# ║      → lio_interface → /registered_scan + /registered_odometry       ║
# ║        → sensor_scan_generation → odom→base_footprint TF + /odom     ║
# ║          → pointcloud_to_laserscan（负值切片 -2.0~-0.3）→ /scan     ║
# ║            → SLAM Toolbox → /map                                     ║
# ║                                                                      ║
# ║  注意：Point-LIO 重力对齐后 /cloud_registered 是 Z 朝上，但 3D→2D    ║
# ║  切片发生在 base_footprint（=Airy 机体/IMU 帧，Z 朝下安装），        ║
# ║  所以切片高度仍用负值（墙在负 Z），与 FAST-LIO 的 zlim 相同。       ║
# ╚══════════════════════════════════════════════════════════════════════╝


# ================ rslidar 驱动 ================
# gnome-terminal --title="robosense lidar SDK" -- bash -c "
# source install/setup.bash;
# ros2 launch rslidar_sdk driver_only.launch.py"

# ================ Point-LIO 里程计 ================
# rviz:=False 关闭 Point-LIO 自带的 3D 可视化，统一用下方 nav2.rviz 看 2D 建图结果
# 注意，因为重力对齐的原因，我把重力设置为世界坐标系的上面，可以看看 robosenseAiry.yaml 文件夹中的 gravity 配置
gnome-terminal --title="Point-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch point_lio point_lio_robosenseAiry.launch.py rviz:=True"

# ================ robosense airy 描述 ================
gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch gld_robot_description robosense_description_launch.py"

# ================ 中间层 ================
# lio_interface 订阅 Point-LIO 里程计 /aft_mapped_to_init
# 切片在 base_footprint（Airy 机体帧 Z 朝下）中进行，用负值（zlim）
gnome-terminal --title="中间层(Point-LIO + sensor + pc2l)" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner middleware_launch.py \
    use_sim_time:=False \
    odometry_sub:=/aft_mapped_to_init \
    pc2l_config:=Pointcloud2d_3d_zlim.yaml"


# ================ slam toolbox 建图 ================
gnome-terminal --title="slam_toolbox 建图" -- bash -c "
source install/setup.bash;
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=src/planner/nav2_planner/config/slam_toolbox_params.yaml"


# ================ slam toolbox 建图可视化 ================
gnome-terminal --title="slam_toolbox 建图可视化" -- bash -c "
source install/setup.bash;
rviz2 -d src/gld_robot_description/rviz/nav2.rviz"

# ================ Nav2 导航 ================
# 建图阶段不需要导航，建图完成后再用 robosense_nav2_real_zlim.sh 启动导航
# gnome-terminal --title="Nav2 导航" -- bash -c "
# source install/setup.bash;
# ros2 launch nav2_planner my_nav2_launch.py"

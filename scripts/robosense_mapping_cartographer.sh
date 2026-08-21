#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════════════╗
# ║  Airy bag 回放建图 (Cartographer) — 全链路 use_sim_time=true          ║
# ║                                                                      ║
# ║  为什么必须用模拟时间：                                              ║
# ║  这个 bag (dataset/robosenseAiry/mapping) 录制于 use_lidar_clock=true║
# ║  时代,消息时间戳 ~11865s(雷达坏时钟),与墙钟/录制墙钟都不一致。      ║
# ║  节点若用墙钟,pc2l 的 tf2 buffer 会把 bag 时间戳的 TF 判成           ║
# ║  "过去的数据"(TF_OLD_DATA) 丢弃 → 永远产不出 /scan。                 ║
# ║  所以必须：所有节点 use_sim_time=true,并让 /clock 跟随 bag 消息      ║
# ║  时间戳(bag_clock_bridge.py), 全链路时间基准统一到 ~11865。          ║
# ║                                                                      ║
# ║  注意: 不要用 `ros2 bag play --clock`, 它发的是录制起始墙钟          ║
# ║  (1785481507), 和消息戳差 1.7e9 秒, 依旧会 TF_OLD_DATA。             ║
# ║                                                                      ║
# ║  用法: ./scripts/robosense_mapping_bag.sh [bag路径]                  ║
# ║    默认 bag: /home/ros/dataset/robosenseAiry/mapping                 ║
# ║                                                                      ║
# ║  每次重放前务必先 ./scripts/kill_all.sh -f 清场再跑本脚本,           ║
# ║  否则 tf2 缓存里上一次的旧数据会再次触发 TF_OLD_DATA。               ║
# ║  rviz 打开后,请在 Time 面板勾选 Use Sim Time。                       ║
# ╚══════════════════════════════════════════════════════════════════════╝

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

BAG="${1:-/home/ros/dataset/robosenseAiry/mapping}"

echo "→ 回放 bag: $BAG"

# ================ bag 回放 (不带 --clock) ================
# --clock 发的是录制起始墙钟, 与消息戳(11865)不一致; /clock 由
# bag_clock_bridge 提供 (跟随消息时间戳)。
gnome-terminal --title="bag 回放" -- bash -c "
source install/setup.bash;
ros2 bag play $BAG"

# ================ /clock 桥 (跟随 bag 消息时间戳 ~11865) ================
gnome-terminal --title="/clock 桥" -- bash -c "
source install/setup.bash;
python3 scripts/bag_clock_bridge.py"

# ================ fast-lio (sim time, 不开自带 rviz) ================
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py \
  use_sim_time:=true rviz:=true"

# ================ lio_interface (launch 硬编码 use_sim_time=True) ================
gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface fastlio_lio_interface_launch.py"

# ================ 机器人描述 ================
gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient 2>/dev/null;
source install/setup.bash;
ros2 launch gld_robot_description robosense_description_launch.py"

# ================ sensor_scan_generation (launch 硬编码 use_sim_time=True) ====
gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# ================ 3d点云转2d (zlim; 显式 use_sim_time:=true) ======
gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner pointcloud_to_laserscan_launch_zlim.py \
  use_sim_time:=true"

# =================== cartographer 建图 (sim time) ===================
gnome-terminal --title="cartographer 建图" -- bash -c "
source install/setup.bash;
ros2 launch nav2_planner cartographer_2d_launch.py use_sim_time:=True"

# ================ cartographer 建图可视化 (sim-time rviz) ================
gnome-terminal --title="cartographer 建图可视化" -- bash -c "
source install/setup.bash;
rviz2 -d src/gld_robot_description/rviz/nav2_sim.rviz"

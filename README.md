# LIO_Nav

[English Documentation](./README_EN.md)

基于 ROS 2 的 3D LiDAR 自主导航系统

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22313F?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](https://releases.ubuntu.com/22.04/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](./LICENSE)

<p>
  <img src="docs/KISS%20show.gif" alt="KISS demo 1" width="48%" style="margin-right: 25px;">
  <img src="docs/KISS%20show_2.gif" alt="KISS demo 2" width="48%">
</p>

**LIO_Nav2_ROS2** 是一个面向四轮滑移转向机器人的 ROS 2 Humble 导航工作空间。系统以 Livox MID-360 / RoboSense Airy 3D LiDAR 和 IMU 为核心传感器，集成 LiDAR-Inertial Odometry (LIO) 里程计、2D SLAM 建图、3D 点云重定位和 Nav2 导航框架。支持 **Gazebo 仿真**、**实机部署**和**数据集回放**。

核心特性：

- **多种 LIO 后端** — FAST-LIO2、Point-LIO、**Super-LIO** 可灵活切换
- **双 SLAM 后端** — SLAM Toolbox 与 **Cartographer** 可选，Cartographer 自带回环检测和纯定位
- **3D 重定位** — KISS-Matcher + small_gicp 全局重定位，或 Cartographer 纯定位
- **传感器兼容** — 同时支持 Livox MID-360 和 **RoboSense Airy**
- **仿真-实机一致性** — 同一套导航栈，仅传感器驱动和 URDF 不同
- **本机运行** — 所有启动、建图、导航脚本均为本机一键运行（tmux 多窗口组织）
- **完整工具链** — 构建、建图、保存地图、导航全流程脚本化
- **2D 单线 LiDAR 导航** — 标准差分小车（`diff_robot_2d`）+ 单线 LaserScan + Cartographer 2D SLAM + Nav2 在线导航
- **SCAN-Planner 局部避障** — B-spline 轨迹优化 + A* 绕障的局部反应式规划器（替代 Nav2 局部规划）
- **自主探索** — TARE / FAR Planner 大范围自主探索规划器，配合 LIO 在未知室内环境自动覆盖建图

本仓库的启动、建图、导航、保存地图等所有操作均已封装为 `scripts/` 目录下的一键脚本（使用 tmux 多窗口组织各节点，便于查看日志）。所有脚本均在**工程根目录**下运行。

---

## 1. 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS 2**：Humble Hawksbill
- **Gazebo**：Fortress
- **Livox-SDK2**：实机模式需要，需自行编译
- **推荐**：本机编译运行

## 2. 构建

```bash
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
```

`build.sh` 等价于：

```bash
MAKEFLAGS="-j12" colcon build --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DUSE_SYSTEM_TBB=ON
```

如果卡死了，可以采用更少的线程数编译，如下编译命令
```bash
MAKEFLAGS="-j4" colcon build --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DUSE_SYSTEM_TBB=ON
```

由于`registeration`文件夹下的 `kiss-matcher` 和 `small_gicp` 在编译时，会从 github 下安装源代码，所以需要提前打开 VPN，否则会编译报错。每次修改源代码后需重新构建，并且启动任何节点前确保已执行 `source install/setup.bash`。

## 3. 脚本总览

所有脚本位于 `scripts/` 目录，按功能分组如下：

| 类别 | 脚本 | 说明 |
|------|------|------|
| **工具** | `build.sh` | 编译整个工作空间 |
| | `kill_all.sh` | 一键关闭所有节点 |
| | `docker_shutdown.sh` | 关闭 Docker 容器内节点（仅 Docker 模式） |
| | `show_tf_tree.sh` | 生成 TF 树 |
| | `save_map.sh` | 保存 2D 栅格地图 |
| | `save_pcd.sh` | 保存 3D 点云地图 |
| | `pgm_to_image.py` | PGM 地图转 PNG/JPG |
| | `view_pcd.py` | PCD 点云查看器 |
| **仿真 · 建图** | `mapping_sim.sh` | Gazebo + LIO + SLAM Toolbox |
| | `mapping_sim_carto.sh` | Gazebo + FAST-LIO + Cartographer |
| | `mapping_sim_carto_single.sh` | 精简建图：Gazebo + Cartographer（无 LIO） |
| **仿真 · 导航** | `nav2_sim.sh` | Gazebo + FAST-LIO + KISS-Matcher + Nav2 |
| | `nav2_sim_carto.sh` | Gazebo + FAST-LIO + Cartographer 纯定位 + Nav2 |
| | `nav2_sim_carto_single.sh` | 精简导航：Gazebo + Cartographer 纯定位 + Nav2（无 LIO） |
| **仿真 · 2D** | `nav2_2d_sim.sh` | 2D 单线 LiDAR + Cartographer + Nav2 |
| **仿真 · SCAN-Planner** | `nav2_scan_sim.sh` | Gazebo + FAST-LIO + SCAN-Planner 局部避障 |
| | `map_then_scan.sh` | 先建图保存 PCD → SCAN-Planner 加载 PCD 全局规划 |
| **仿真 · 探索** | `exploration_tare_sim.sh` | Gazebo + FAST-LIO + TARE 自主探索 |
| | `exploration_far_sim.sh` | Gazebo + FAST-LIO + FAR Planner 自主探索 |
| **RoboSense Airy 数据集** | `robo_mapping.sh` | FAST-LIO + SLAM Toolbox 建图 |
| | `robo_mapping_carto.sh` | FAST-LIO + Cartographer 建图 |
| | `robo_mapping_superlio.sh` | Super-LIO + SLAM Toolbox 建图 |
| | `robo_mapping_pointlio.sh` | Point-LIO + SLAM Toolbox 建图 |
| | `robo_nav2.sh` | FAST-LIO + KISS-Matcher + Nav2 导航 |
| | `robo_scan_planner.sh` | SCAN-Planner + LIO 一体化 |
| | `exploration_tare_real.sh` | rosbag + FAST-LIO + TARE |

> 仿真类脚本使用 tmux 多窗口替代 gnome-terminal，在工程根目录下直接运行即可（见各脚本头部注释）。

## 4. 工具脚本

### 4.1 编译 `build.sh`

见[第 2 节](#2-构建)。

### 4.2 关闭节点 `kill_all.sh`

统一关闭仿真 / 实机 / 建图 / 导航的所有节点，自动识别 ROS 2 节点并按进程名清理，同时清理本项目创建的 tmux 会话（不影响其他会话）。

```bash
./scripts/kill_all.sh              # 自动检测并清理
./scripts/kill_all.sh --dry-run    # 只列出，不实际执行
./scripts/kill_all.sh -f           # 强制模式：跳过 ros2 检测，直接按进程名杀
```

### 4.3 关闭容器 `docker_shutdown.sh`（仅 Docker 模式）

关闭 Docker 容器内所有 ROS 节点 / tmux 会话，并可选停止或删除项目容器。本机运行无需使用。

```bash
./scripts/docker_shutdown.sh            # 只清理容器内节点，容器保持运行
./scripts/docker_shutdown.sh --stop     # 清理节点后停止容器
./scripts/docker_shutdown.sh --rm       # 清理节点后删除容器
./scripts/docker_shutdown.sh --dry-run  # 只列出，不实际执行
```

### 4.4 TF 树 `show_tf_tree.sh`

生成 TF 树 PDF（`tf2_tools view_frames`），用于排查 TF 断开 / 代价地图空白等问题。

```bash
cd scripts && ./show_tf_tree.sh
```

### 4.5 保存 2D 地图 `save_map.sh`

用 `map_saver_cli` 保存 2D 栅格地图（`.pgm` + `.yaml`），并软链到 `install/` 供 launch 引用。

```bash
./scripts/save_map.sh [输出路径]   # 默认 src/planner/nav2_planner_bringup/map/robo_map
```

### 4.6 保存 3D 点云 `save_pcd.sh`

触发 FAST-LIO 的 `/map_save` 服务，保存 ikdtree 地图（`robo_map.pcd`）与稠密累积点云（`dense_map.pcd`，KISS-Matcher 重定位用），并软链到 `install/`。

```bash
./scripts/save_pcd.sh
```

> 需 FAST-LIO 正在运行；稠密点云需在 FAST-LIO 配置中开启 `pcd_save.pcd_save_en: true`。

### 4.7 PGM 转图 `pgm_to_image.py`

将 ROS 地图 PGM 转成 PNG/JPG 便于查看。

```bash
python3 scripts/pgm_to_image.py <pgm文件> [-o 输出路径] [--format png|jpg]
```

### 4.8 PCD 查看器 `view_pcd.py`

PCD 点云查看器，支持 Z 轴过滤、体素降采样、尺寸统计。

```bash
python3 scripts/view_pcd.py <pcd文件> [--zmin 下界] [--zmax 上界] [--voxel 体素] [--no-view]
```

## 5. 仿真 · 建图

### 5.1 `mapping_sim.sh` — Gazebo + LIO + SLAM Toolbox

默认建图流程，可选 LIO 算法（第一个参数）：

```bash
./scripts/mapping_sim.sh [0|1|2]
# 0 = fast-lio（默认）  1 = point-lio  2 = super-lio
```

启动窗口：`GUI控制 | Gazebo | LIO | lio_interface | sensor_scan | pc2laser | RViz | slam_toolbox`。

查看输出：

```bash
tmux attach -t mapping_sim   # Ctrl-b n/p 切窗口, Ctrl-b d 退出
```

### 5.2 `mapping_sim_carto.sh` — Gazebo + FAST-LIO + Cartographer

用 Cartographer 替代 SLAM Toolbox，自带回环检测，发布 `/map` + `map→odom` TF。

```bash
./scripts/mapping_sim_carto.sh
```

建图完成后保存地图（脚本尾部有提示）：

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename .../map.pbstream -map_filestem .../my_map
```

### 5.3 `mapping_sim_carto_single.sh` — 精简建图

只用 Gazebo + Cartographer，去掉 FAST-LIO / lio_interface / sensor_scan_generation，纯 scan matching + IMU 建图。

```bash
./scripts/mapping_sim_carto_single.sh
```

## 6. 仿真 · 导航

### 6.1 `nav2_sim.sh` — KISS-Matcher + Nav2

基于先验 PCD 地图的 3D 重定位导航。

```bash
./scripts/nav2_sim.sh
```

> 前置：已建好 `.pgm`/`.yaml` 地图（`my_nav2_launch.py` 中 `map_yaml_file` 指向它）和 `.pcd` 点云（`global_kiss_matcher_relocalization` 中 `prior_pcd_file` 指向它）。在 RViz 中用 "2D Pose Estimate" 给初始位姿，再用 "Nav2 Goal" 发目标。

### 6.2 `nav2_sim_carto.sh` — Cartographer 纯定位 + Nav2

用 Cartographer 纯定位替代 KISS-Matcher，加载 `.pbstream` 地图发布 `map→odom`，配合 map_server 加载 `.pgm` 静态地图。

```bash
./scripts/nav2_sim_carto.sh
```

> 前置：`map.pbstream` 已准备好，且 Nav2 `map_server` 加载的 `.pgm` 与 pbstream 对应。

### 6.3 `nav2_sim_carto_single.sh` — 精简导航

只用 Gazebo + Cartographer 纯定位 + Nav2，去掉 LIO / KISS-Matcher。可指定 pbstream 路径：

```bash
./scripts/nav2_sim_carto_single.sh [pbstream路径]
```

## 7. 仿真 · 2D 单线 LiDAR

### `nav2_2d_sim.sh`

标准差分小车（`diff_robot_2d`）+ 单线 LaserScan + Cartographer 2D SLAM + Nav2 在线导航，无需预建图。

```bash
./scripts/nav2_2d_sim.sh [world]
# world 默认 indoor_2d（src/get_urdf/worlds/indoor_2d.world）
```

在 RViz 中用 "2D Goal Pose" 发目标，机器人实时建图并自主导航。

## 8. 仿真 · SCAN-Planner

SCAN-Planner 是 B-spline 轨迹优化 + A* 绕障的局部反应式规划器，用于需要比 Nav2 更激进的局部避障场景。

### 8.1 `nav2_scan_sim.sh` — FAST-LIO + SCAN-Planner 联合仿真

```bash
./scripts/nav2_scan_sim.sh
```

启动窗口：`Gazebo | FAST-LIO | GroundSeg | lio_if | sensor | SCAN | SP-RViz`。切到 SP-RViz 窗口用 "2D Goal Pose" 点目标开始规划。

### 8.2 `map_then_scan.sh` — 建图后加载 PCD 全局规划

两步流程：先用 GUI 遥控建图并保存 PCD，再用 SCAN-Planner 加载完整 PCD 地图做全局规划（`use_pcd_map:=true`）。

```bash
./scripts/map_then_scan.sh
```

保存 PCD：

```bash
source install/setup.bash && ros2 service call /map_save std_srvs/srv/Trigger
# 然后把 PCD 拷到 src/planner/nav2_planner_bringup/pcd/test_world.pcd，重新运行本脚本
```

## 9. 仿真 · 自主探索

### 9.1 `exploration_tare_sim.sh` — TARE 自主探索

TARE Planner 大范围自主探索，自动在未知室内环境覆盖建图。

```bash
./scripts/exploration_tare_sim.sh
```

启动约 15 秒后自动触发探索。

### 9.2 `exploration_far_sim.sh` — FAR Planner 自主探索

FAR Planner 可见图（visibility graph）全局规划器。

```bash
./scripts/exploration_far_sim.sh
```

> 依赖 `GAZEBO_MODEL_PATH` 指向数据集目录里的模型（脚本已自动设置）。操作：RViz 里点 "Goalpoint" 按钮，再点地图设目标点。

## 10. RoboSense Airy 数据集（实机 / 回放）

以下脚本针对 RoboSense Airy 实机 / rosbag 数据集回放，通过脚本第一个参数指定本机数据集路径（原 Docker 挂载的 `/dataset` 对应本机 `/home/ros/dataset`）。

### 10.1 建图

| 脚本 | LIO 后端 | SLAM 后端 |
|------|----------|-----------|
| `robo_mapping.sh [bag]` | FAST-LIO | SLAM Toolbox |
| `robo_mapping_carto.sh` | FAST-LIO | Cartographer |
| `robo_mapping_superlio.sh [bag]` | Super-LIO | SLAM Toolbox |
| `robo_mapping_pointlio.sh [bag]` | Point-LIO | SLAM Toolbox |

示例：

```bash
./scripts/robo_mapping.sh /home/ros/dataset/robosense/mapping
tmux attach -t robo_mapping   # 查看输出
```

建图完成后保存地图（见脚本尾部提示）：

```bash
./scripts/save_map.sh   # 2D 栅格地图
./scripts/save_pcd.sh   # 3D 点云地图
```

Cartographer 版保存方式不同（`WriteState` + `FinishTrajectory` + `pbstream_to_ros_map`），见 `robo_mapping_carto.sh` 脚本尾部。

### 10.2 导航

`robo_nav2.sh [bag]` — FAST-LIO + KISS-Matcher + Nav2。

```bash
./scripts/robo_nav2.sh /home/ros/dataset/robosense/nav1
```

> 前置：已建好 `.pgm`/`.yaml` 地图（`my_nav2_launch.py`）和 `dense_map.pcd`（KISS-Matcher launch）。导航时先在 RViz 给 "2D Pose Estimate"，再给 "Nav2 Goal"。

### 10.3 SCAN-Planner 一体化

`robo_scan_planner.sh` — bag → FAST-LIO → lio_interface → SCAN-Planner（ESDF 建图 + 局部规划）。

```bash
./scripts/robo_scan_planner.sh
```

切到 SP-RViz 窗口（Fixed Frame: odom）用 "2D Goal Pose" 点目标。

### 10.4 TARE 回放

`exploration_tare_real.sh` — rosbag + FAST-LIO + TARE 自主探索（无 GUI 遥控）。

```bash
./scripts/exploration_tare_real.sh
```

## 11. 常见工作流

**仿真从建图到导航：**

```bash
# 1. 建图（任选其一）
./scripts/mapping_sim.sh            # SLAM Toolbox
./scripts/mapping_sim_carto.sh      # Cartographer（带回环）

# 2. 保存地图
./scripts/save_map.sh
./scripts/save_pcd.sh

# 3. 导航（任选其一）
./scripts/nav2_sim.sh               # KISS-Matcher + Nav2
./scripts/nav2_sim_carto.sh         # Cartographer 纯定位 + Nav2
```

**SCAN-Planner 局部避障：**

```bash
./scripts/nav2_scan_sim.sh       # 在线
./scripts/map_then_scan.sh       # 先建图后加载 PCD
```

**清理：**

```bash
./scripts/kill_all.sh                 # 关闭所有节点
```

## 12. 常见问题

**Gazebo 无法启动** — 残留进程阻止新实例启动，手动终止：

```bash
killall -9 gzserver gzclient
# 或 ./scripts/kill_all.sh -f
```

**LIO 里程计发散** — 检查 IMU 和 LiDAR 话题是否有数据（`ros2 topic echo`），确认 `lidar_type` 与传感器匹配，检查 `use_sim_time` 设置。

**TF 断开 / 代价地图空白** — 进入 `scripts/` 后使用 `./show_tf_tree.sh` 检查 TF 树，确认 `/scan` 正在发布，检查 `pointcloud_to_laserscan` 的目标坐标系是否与 LiDAR 坐标系一致。

**重定位失败** — 确认 PCD 文件存在且非空，在 RViz 中使用 "2D Pose Estimate" 给出大致初始位姿，或尝试 KISS-Matcher 全局重定位方案。

**KISS-Matcher 全局重定位一直失败** — 检查 `/registered_scan` 是否有数据，确认 `prior_pcd_file` 指向当前环境的 PCD；确保 `base_footprint` &rarr; `livox_frame` TF 可查询；让机器人原地旋转或移动一小段距离以增加累计点云重叠。

**TF 抖动或 Nav2 位姿跳变** — 检查是否同时运行了 `small_gicp_relocalization` 和 `global_relocalization_kiss_matcher`。同一时间只能有一个节点发布 `map` &rarr; `odom`。

**实机 LiDAR 无数据** — 检查网线连接，确认 `MID360_config.json` 中的 IP 地址，确认 Livox-SDK2 已安装。

**构建失败** — 清理后重新构建：

```bash
rm -rf build/ install/ log/
cd scripts
./build.sh
```

## 13. 致谢

本项目基于以下开源项目构建：

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — 紧耦合 LiDAR-IMU 里程计
- [Point-LIO](https://github.com/hku-mars/Point-LIO) — 高带宽 LiDAR-IMU 里程计
- [Super-LIO](https://github.com/hku-mars/Super-LIO) — 紧凑地图 LiDAR-IMU 里程计
- [Cartographer](https://github.com/cartographer-project/cartographer) — 2D/3D 图优化 SLAM
- [Nav2](https://github.com/ros-planning/navigation2) — ROS 2 导航框架
- [small_gicp](https://github.com/koide3/small_gicp) — 高效并行化 GICP 配准
- [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) — 快速全局点云配准 (ICRA 2025)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — 2D 位姿图 SLAM
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) — Livox LiDAR SDK
- [Sophus](https://github.com/strasdat/Sophus) — 李群 C++ 库
- [SCAN-Planner](https://github.com/HKUST-Aerial-Robotics/SCAN-Planner) — 局部反应式避障规划器（B-spline + A*）
- [TARE Planner](https://github.com/caochao39/tare_planner) — 大范围自主探索规划器
- [FAR Planner](https://github.com/HKUST-Aerial-Robotics/FAR_Planner) — 可见图全局规划器

## 14. 许可证
本项目依据 [MIT License](./LICENSE) 开源。

# Real → Sim 全流程：RoboSense Airy 实机建图 + 仿真导航

用 FAST-LIO + RoboSense Airy 做实机建图，将真实环境转化为 Gazebo 仿真世界进行导航测试。

## 整体流程

```
Phase 1: 实机建图                     Phase 2: Real→Sim 转换          Phase 3: 仿真导航
┌──────────────────────┐    ┌──────────────────────────┐    ┌─────────────────────┐
│ RoboSense Airy       │    │ 3D PCD → 2D 栅格地图     │    │ Gazebo 加载仿真世界  │
│ + FAST-LIO           │    │ 3D PCD → 障碍物提取      │    │ Nav2 加载实机 2D 地图│
│ + SLAM Toolbox       │    │ 手工/自动创建 .world    │    │ KISS-Matcher 重定位  │
│ → airy_map.pcd       │    │ → airy_world.world      │    │ → 路径规划 & 导航    │
│ → airy_map.yaml/.pgm │    │ → airy_map.yaml/.pgm    │    │ → 验证 & 调优        │
└──────────────────────┘    └──────────────────────────┘    └─────────────────────┘
```

---

## Phase 1: 实机建图

### 1.1 硬件准备

```
RoboSense Airy
    │ 以太网
    ▼
工控机 (Ubuntu 22.04 + ROS 2 Humble)
    │
    ├── FAST-LIO (LiDAR-IMU 里程计)
    ├── lio_interface (TF 桥接)
    ├── sensor_scan_generation (点云组装)
    └── SLAM Toolbox (2D 建图)
```

> **前提**：FAST-LIO 和基础 ROS 2 环境已搭好，项目正常编译。

### 1.2 已有适配代码

本项目已有现成的 FAST-LIO for RoboSense Airy 适配代码，位于：

```
/home/ros/rosws/fast_lio2_ros2/FAST_LIO_ROBOAIRY/
```

**无需从零修改 FAST-LIO**，直接使用该适配版本即可。

```bash
# 编译 RoboSense Airy 专用的 FAST-LIO
cd ~/rosws/fast_lio2_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fast_lio_robosense \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

### 1.3 适配关键点（代码中已实现）

参照 `FAST_LIO_ROBOAIRY/config/robosenseAiry.yaml`，与 Livox MID-360 的核心差异：

| 参数 | Livox MID-360 | RoboSense Airy |
|------|---------------|----------------|
| `lidar_type` | `1` | `5` |
| `scan_line` | `50` | `96` |
| LiDAR 话题 | `/livox/lidar` | `/rslidar_points` |
| IMU 话题 | `/livox/imu` | `/rslidar_imu_data` |
| `blind` | `0.5` | `1.0` |
| `fov_degree` | `360.0` | `120.0` |
| `timestamp_unit` | `1` (ms) | `0` (s) |
| IMU-LiDAR 外参旋转 | 单位阵 | `[[0,-1,0],[-1,0,0],[0,0,-1]]` |

### 1.4 安装 RoboSense 驱动

RoboSense Airy 使用 `rs_driver` + `rslidar_sdk`：

```bash
cd ~/rosws/3d_nav_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git -b ros2
cd rslidar_sdk && git submodule update --init --recursive

cd ~/rosws/3d_nav_ws
colcon build --symlink-install --packages-select rslidar_sdk \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

在 `rslidar_sdk/config/config.yaml` 中配置：

```yaml
lidar:
  - driver:
      lidar_type: RS AIRY
      frame_id: rslidar
      msop_port: 6699
      difop_port: 7788
    ros:
      ros_frame_id: rslidar
      ros_send_point_cloud_topic: /rslidar_points   # Airy 专用话题
```

### 1.5 URDF 适配

修改 `gld_robot_description` 的 URDF，添加 Airy 的坐标系链：

```xml
<!-- gld_robot_description.urdf -->
<joint name="rslidar_joint" type="fixed">
  <parent link="chassis"/>
  <child link="rslidar"/>
  <origin xyz="0.15 0 0.52" rpy="0 0 0"/>  <!-- Airy 安装位置 -->
</joint>
<link name="rslidar"/>
```

同时在 URDF 中设置静态 TF `chassis → livox_frame` 用于兼容原有 launch 文件：

```xml
<joint name="lidar_alias_joint" type="fixed">
  <parent link="rslidar"/>
  <child link="livox_frame"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
<link name="livox_frame"/>
```

### 1.6 创建实机建图脚本

```bash
#!/usr/bin/env bash
# scripts/mapping_airy.sh — RoboSense Airy 实机建图
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

SESSION="mapping_airy"
tmux kill-session -t "$SESSION" 2>/dev/null
source install/setup.bash

tmux new-session -d -s "$SESSION" -n "sensors"

# Pane 0: RoboSense Airy 驱动
tmux send-keys -t "$SESSION:sensors.0" \
  "source install/setup.bash && ros2 launch rslidar_sdk start.py" C-m
tmux select-pane -t "$SESSION:sensors.0" -T "Airy"

# Pane 1: FAST-LIO (使用 fast_lio_robosense 适配版)
tmux split-window -h -t "$SESSION:sensors"
tmux send-keys -t "$SESSION:sensors.1" \
  "source ~/rosws/fast_lio2_ros2/install/setup.bash && \
   ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py \
   use_sim_time:=False" C-m
tmux select-pane -t "$SESSION:sensors.1" -T "FAST-LIO(Airy)"

# Pane 2: lio_interface
tmux split-window -v -t "$SESSION:sensors.0"
tmux send-keys -t "$SESSION:sensors.2" \
  "source install/setup.bash && ros2 launch lio_interface lio_interface_launch.py" C-m
tmux select-pane -t "$SESSION:sensors.2" -T "lio_if"

# Pane 3: sensor_scan_generation
tmux split-window -v -t "$SESSION:sensors.1"
tmux send-keys -t "$SESSION:sensors.3" \
  "source install/setup.bash && ros2 launch sensor_scan_generation \
   sensor_scan_generation_launch.py use_sim_time:=False" C-m
tmux select-pane -t "$SESSION:sensors.3" -T "scan_gen"

# Window 2: SLAM + 切片
tmux new-window -t "$SESSION" -n "slam"
tmux send-keys -t "$SESSION:slam.0" \
  "source install/setup.bash && ros2 launch nav2_planner \
   pointcloud_to_laserscan_launch.py" C-m
tmux select-pane -t "$SESSION:slam.0" -T "3d→2d"

tmux split-window -h -t "$SESSION:slam"
tmux send-keys -t "$SESSION:slam.1" \
  "source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py \
   slam_params_file:=src/nav2_planner/config/slam_toolbox_params.yaml \
   use_sim_time:=False" C-m
tmux select-pane -t "$SESSION:slam.1" -T "SLAM"

tmux attach -t "$SESSION"
```

### 1.7 建图 & 保存

```bash
# 1. 检查硬件连接
ping 192.168.1.200   # Airy 默认 IP
ros2 topic list | grep livox

# 2. 启动建图
./scripts/mapping_airy.sh

# 3. 驾驶机器人遍历环境
#    - 保持 0.5 m/s 以下的低速
#    - 经过所有角落和走廊
#    - 确保回环

# 4. 保存地图（在另一个终端）
source install/setup.bash
./scripts/save_map.sh    # → src/nav2_planner/map/airy_map.yaml + .pgm
./scripts/save_pcd.sh    # → 手动 cp 到 src/nav2_planner/pcd/airy_map.pcd
```

### 1.8 建图质量检查

```bash
# 检查 PCD 点云
pcl_viewer src/nav2_planner/pcd/airy_map.pcd

# 查看 2D 地图
eog src/nav2_planner/map/airy_map.pgm

# 确认关键文件
ls -lh src/nav2_planner/map/airy_map.*
ls -lh src/nav2_planner/pcd/airy_map.pcd
```

---

## Phase 2: Real → Sim 转换

### 2.1 关键原则

> **Gazebo world 的几何结构必须与 2D 地图的障碍物分布一致。** 二者来自同一实机数据，Nav2 才能正确规划。

```
实机数据
├── 3D PCD (airy_map.pcd)  ──► Gazebo World (airy_world.world)
└── 2D Map (airy_map.yaml) ──► Nav2 导航 (map_server 加载)
                                     ↑
                              必须匹配
                                     │
Gazebo World 中的障碍物 ←────────────┘
```

### 2.2 方式 A：手工参照建图（快速，推荐首选）

**Step 1**：打开 2D 地图查看障碍物：

```bash
# 查看地图分辨率
head -5 src/nav2_planner/map/airy_map.yaml
# 输出: resolution: 0.05, origin: [-10.0, -10.0, 0.0]
```

**Step 2**：量测障碍物位置。使用图像查看器打开 `.pgm`，根据分辨率计算：

```
pixel_x * 0.05 + origin_x = 世界坐标 x
pixel_y * 0.05 + origin_y = 世界坐标 y
```

**Step 3**：创建 world 文件：

```xml
<!-- src/get_urdf/worlds/airy_world.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="airy_world">

    <!-- 基础 -->
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- ===== 根据实机地图放置障碍物 ===== -->

    <!-- 东墙壁 (x=3.5, 从 y=-4 到 y=4) -->
    <model name="wall_east">
      <static>true</static>
      <link name="link">
        <pose>3.5 0 0.5 0 0 0</pose>
        <visual name="visual">
          <geometry><box><size>0.2 8 1</size></box></geometry>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.2 8 1</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- 西墙壁 (x=-3.5) -->
    <model name="wall_west">
      <static>true</static>
      <link name="link">
        <pose>-3.5 0 0.5 0 0 0</pose>
        <visual name="visual">
          <geometry><box><size>0.2 8 1</size></box></geometry>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.2 8 1</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- 圆柱障碍 (x=0, y=2) -->
    <model name="pillar_1">
      <static>true</static>
      <link name="link">
        <pose>0 2 0.5 0 0 0</pose>
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
        </collision>
      </link>
    </model>

    <!-- 更多障碍物... -->

  </world>
</sdf>
```

### 2.3 方式 B：PCD → Mesh（高精度）

```bash
# 1. 地面滤波 (保留高度 0.1m 以上的点)
pcl_pass_through_filter -filter_field_name z \
  -filter_min 0.1 -filter_max 10.0 \
  airy_map.pcd airy_no_ground.pcd

# 2. 降采样 (减少面数)
pcl_voxel_grid -leaf 0.05 0.05 0.05 \
  airy_no_ground.pcd airy_downsampled.pcd

# 3. 法线估计 + 表面重建
pcl_normal_estimation -radius 0.1 airy_downsampled.pcd airy_normals.pcd
pcl_poisson_reconstruction airy_normals.pcd airy_mesh.ply

# 4. 格式转换 (PLY → DAE)
# 用 MeshLab 或 assimp 工具
meshlab airy_mesh.ply
# Filters → Cleaning → Remove Duplicate Faces
# File → Export Mesh As → airy_mesh.dae
```

在 world 中引用：

```xml
<model name="real_env">
  <static>true</static>
  <link name="link">
    <pose>0 0 0 0 0 0</pose>
    <visual name="visual">
      <geometry><mesh><uri>model://airy_mesh.dae</uri></mesh></geometry>
    </visual>
    <collision name="collision">
      <geometry><mesh><uri>model://airy_mesh.dae</uri></mesh></geometry>
    </collision>
  </link>
</model>
```

> **注意**：Mesh 碰撞检测比盒/柱慢很多。如果环境简单，优先手工方式 A。

### 2.4 配置导航参数

建图完成后，更新以下文件：

**my_nav2_launch.py** — 指向实机 2D 地图：

```python
map_yaml_file = os.path.join(me_share_path, 'map', 'airy_map.yaml')
```

**global_kiss_matcher_relocalization_launch.py** — 指向实机 3D 点云：

```python
pcd_path = os.path.join(
    get_package_share_directory("nav2_planner"),
    "pcd", "airy_map.pcd"
)
# 在 parameters 中使用:
"prior_pcd_file": pcd_path,
```

**get_urdf_launch.py** — 指向新 world：

```python
world_file_path = os.path.join(pkg_share_path, 'worlds', 'airy_world.world')
```

---

## Phase 3: 仿真导航

### 3.1 部署

```bash
cd ~/rosws/3d_nav_ws
source /opt/ros/humble/setup.bash

# 确认所有文件就位
ls src/nav2_planner/map/airy_map.yaml
ls src/nav2_planner/pcd/airy_map.pcd
ls src/get_urdf/worlds/airy_world.world

# 重新部署
colcon build --symlink-install --packages-select get_urdf nav2_planner \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source install/setup.bash
```

### 3.2 启动

```bash
./scripts/nav2_sim_tmux.sh
```

### 3.3 验证流程

```bash
# 1. TF 树完整性
ros2 run tf2_ros tf2_echo map odom
# 应有连续输出

# 2. 地图加载确认
ros2 topic echo /map --once | head -3

# 3. KISS-Matcher 重定位
# 在 KISS+GICP pane 中看到 "KISSMatcher initialization succeeded"
# 如果一直 failed → 让小车原地旋转几圈

# 4. 发送测试目标 (小距离)
ros2 run nav2_planner send_goal.py --ros-args -p x:=2.0 -p y:=0.0 -p yaw:=0.0

# 5. 验证规划路径
ros2 topic echo /plan
```

### 3.4 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 小车不显示 | spawn 时序问题 | `ros2 run gazebo_ros spawn_entity.py -entity simple_car -topic robot_description` |
| map→odom 不存在 | KISS-Matcher 未初始化 | 让小车原地旋转; 确认 `prior_pcd_file` 正确 |
| 路径规划失败 | 2D 地图与 world 不一致 | 复查 Gazebo world 中障碍物坐标 |
| costmap 显示错误 | TF 时间戳不一致 | 所有节点 `use_sim_time:=True` |

---

## 差异对照表：实机 vs 仿真

| | 实机 (Phase 1) | 仿真 (Phase 3) |
|--|---------------|----------------|
| LiDAR 数据源 | RoboSense Airy 硬件 | Gazebo LiDAR 插件 |
| FAST-LIO | `fast_lio_robosense` (`lidar_type=5`) | `fast_lio` (`lidar_type=1`) |
| IMU | Airy 内建 (`/rslidar_imu_data`) | Gazebo IMU 插件 |
| LiDAR 话题 | `/rslidar_points` | `/livox/lidar` |
| 时钟 | 系统时钟 | `/clock` (Gazebo) |
| `use_sim_time` | `False` | `True` |
| URDF | `gld_robot_description` | `get_urdf` |
| 地图 | 实时 SLAM 构建 | 加载静态 `.yaml` |
| 重定位 | 无需 | KISS-Matcher + 先验 PCD |

## 快速命令速查

```bash
# Phase 1: 实机建图
source ~/rosws/fast_lio2_ros2/install/setup.bash  # 加载 Airy 适配版
./scripts/mapping_airy.sh     # 启动建图
./scripts/save_map.sh          # 保存 2D 地图
./scripts/save_pcd.sh          # 保存 3D 点云

# Phase 2: 转换
vim src/get_urdf/worlds/airy_world.world   # 创建仿真世界

# Phase 3: 仿真导航
source ~/rosws/3d_nav_ws/install/setup.bash   # 切回原工作空间
colcon build --packages-select get_urdf nav2_planner
./scripts/nav2_sim_tmux.sh
ros2 run nav2_planner send_goal.py --ros-args -p x:=2.0 -p y:=1.0 -p yaw:=0.0
```

## 参考代码

RoboSense Airy 的 FAST-LIO 适配代码位于：

```
/home/ros/rosws/fast_lio2_ros2/FAST_LIO_ROBOAIRY/
├── config/robosenseAiry.yaml    ← Airy 专用参数 (lidar_type=5, scan_line=96)
├── launch/mapping_robosense_airy.launch.py  ← Airy 专用启动文件
└── src/                         ← 适配后的预处理代码
```

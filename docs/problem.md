# 实机建图与重定位问题记录

> 关联文档: [airy_z_flip.md](airy_z_flip.md) — Airy Z 轴翻转基础问题
> 关联文档: [arch.md](arch.md) — 系统架构

---

## 问题 1：Z 轴翻转修正后 XY 轴互换

### 现象

使用 `airy_unflip.py` 修正 Z 轴翻转后，**机器人前进方向与地图坐标轴不对齐**：物理世界沿 X 轴前进，SLAM Toolbox 地图上显示为 Y 轴移动。

### 根因

旧版 `airy_unflip.py` 直接使用 FAST-LIO 配置中的 `extrinsic_R` 的转置作为修正矩阵：

```
R_ext = [[ 0, -1,  0],
         [-1,  0,  0],
         [ 0,  0, -1]]

R_inv = R_ext^T = R_ext  (此矩阵对称)
```

`R_ext` 是 LiDAR→IMU 的外参旋转，它不仅翻转 Z 轴，还**交换了 X 和 Y 轴**：

```
X → -Y    (前进 → 左移且反向)
Y → -X    (左移 → 后退且反向)
Z → -Z    (上 → 下)
```

用作修正矩阵时，虽然 Z 轴恢复朝上，但 XY 被交换。

### 解决方案

改用**绕 X 轴 180° 旋转**作为修正矩阵：

```
M = [[ 1,  0,  0],
     [ 0, -1,  0],
     [ 0,  0, -1]]

效果: X →  X  (前进方向不变 ✓)
      Y → -Y  (左右镜像，翻转 Z 时数学上不可避免)
      Z → -Z  (Z 轴翻转向下→向上 ✓)
```

Y 轴的左右镜像是 SO(3) 中翻转 Z 轴的必然副作用——任何保持右手系的旋转，翻转 Z 时必须同时翻转 XY 平面中的某个方向。绕 X 轴翻转（保留 X 方向）是保持前进方向的最优选择。

### 代码位置

- `scripts/airy_unflip.py`: `extrin_r` 默认值改为 `1.0,0.0,0.0, 0.0,-1.0,0.0, 0.0,0.0,-1.0`

---

## 问题 2：Z 轴翻转时位姿未修正

### 现象

点云和 SLAM 轨迹不一致，地图出现撕裂或重影。

### 根因

旧版 `airy_unflip.py` **只修正点云**（`/cloud_registered` → `/cloud_registered_unflipped`），**不处理里程计位姿**（`/Odometry`）。

FAST-LIO 发布的 `/Odometry` 位姿仍在翻转后的坐标空间中，而修正后的点云在新坐标空间中。`lio_interface` 和 `sensor_scan_generation` 使用未修正的位姿计算 `odom→base_footprint` TF，导致坐标系不一致：

| 数据 | 坐标系 | 状态 |
|------|--------|------|
| `/cloud_registered_unflipped` (点云) | 修正后 (Z 朝上) | ✓ |
| `/Odometry` (位姿) | 原始翻转 (Z 朝下) | ✗ 不一致 |
| `odom→base_footprint` TF | 由未修正位姿计算 | ✗ 不一致 |

### 位姿修正推导

世界帧施加修正矩阵 M 后：`v_world' = M @ v_world`

原始位姿矩阵 R (world→body)：
```
v_body = R @ v_world = R @ M^T @ v_world'   (v_world = M^T @ v_world')
```

因此：
- **位置**: `t' = M @ t`
- **方向**: `R' = R @ M^T`

### 解决方案

在 `airy_unflip.py` 中新增 odometry 订阅和修正：

- 订阅 `/Odometry`，发布 `/Odometry_unflipped`
- 对位置施加 `M @ t`，对方向施加 `R @ M^T`（旋转矩阵乘法）
- 同步修正协方差矩阵（位置和姿态部分各自变换）
- Twist（机体帧速度）不变，直接复制

下游 `lio_interface` 需订阅修正后的 odometry：
```bash
ros2 launch lio_interface fastlio_lio_interface_launch.py \
  cloud_topic:=/cloud_registered_unflipped \
  odometry_sub:=/Odometry_unflipped
```

### 代码位置

- `scripts/airy_unflip.py`: `odom_cb()` 位姿修正回调，`rotmat_to_quat()` / `quat_to_rotmat()` 工具函数
- `scripts/robosense_mapping_real_zflip.sh`: lio_interface 启动命令 (`odometry_sub:=/Odometry_unflipped`)

---

## 问题 3：重定位地图 PCD 点云太稀疏

> **现状更新**：当前 FAST-LIO 保存 PCD 使用 `pcl_wait_pub`（`publish_map` 累积的 world 点云），由 `save_to_pcd()` 直接写入 `map_file_path`（`robo_map.pcd`），该点云**已满足重定位需求**。下方方案 B 的稠密累积方案（`pcl_wait_save` → `dense_map.pcd`）在 `laserMapping.cpp` 中已注释停用，**不再使用**。

### 现象

KISS-Matcher + small_gicp 全局重定位持续失败：

```
[WARN] KISSMatcher initialization failed. Inliers: 2. Retrying...
[WARN] # final inliers: 0 < 3
[WARN] Overlapness: 26.42% < 80.00%
```

加载的地图 `robo_map.pcd` 只有 ~1300 个点（60KB），远不足以支撑 FPFH 特征匹配。

### 根因

FAST-LIO 的 `save_to_pcd()` 保存的是 **ikd-tree 的稀疏降采样地图**，经过两层降采样：

| 降采样层 | 参数 | 旧值 | 效果 |
|----------|------|------|------|
| ikd-tree 体素 | `filter_size_map` | 0.5m | 每个 0.5m 体素只保留 1 个点 |
| PCD 保存体素 | `save_voxel_size` | 0.2m | 保存时再次降采样 |

两层叠加后，大面积建图结果只有几千个点。而 KISS-Matcher 的核心算法需要：

- **FPFH** (Fast Point Feature Histograms): 对每个点计算局部邻域几何特征描述子（旋转不变），需要足够密度的点才能形成有区分力的特征
- **TEASER++**: 从 FPFH 描述子匹配中通过 GNC (Graduated Non-Convexity) 迭代剔除 outlier，估计 6-DOF 位姿，需要 ≥3 个正确对应 (inliers)

地图只有 1300 个点时，FPFH 描述子区分力不足，TEASER++ 无法找到足够正确对应 → 重定位失败。

### KISS-Matcher + GICP 重定位原理

```
累积扫描 (source, ~90000 pts)  ──→  KISSMatcher 全局粗配准
地图 PCD  (target,   ~1300 pts) ──┘  (FPFH + ROBIN + TEASER++)
                                          │
                                          ▼
                                    粗位姿 (6-DOF)
                                          │
                                          ▼
                                    VGICP 精配准 (验证+细化)
                                          │
                                    overlap > 80%?
                                          │
                                    ┌─────┴─────┐
                                    │ YES        │ NO
                                    ▼            ▼
                              发布 map→odom    失败重试
```

**状态机** (`performRegistration()`):

```
KISS_GLOBAL_INIT ──成功──→ GICP_TRACKING ──失败 N 次──→ GLOBAL_RECOVERY
      ↓                         ↓                           ↓
 KISSMatcher               GICP 连续跟踪              KISSMatcher 恢复
 全局匹配                  以上一帧位姿为初值         重新全局匹配
      ↓                         ↓                           ↓
 发布 TF                   发布 TF                   成功→回 GICP_TRACKING
```

关键：KISSMatcher 虽然不需要初始位姿（FPFH 旋转不变），但需要足够的点来提取区分力强的 FPFH 描述子。

### 解决方案

采用 pcl_wait_pub 点云进行保存。

### 代码位置

- `src/localization/FAST_LIO_ROBOAIRY/config/robosenseAiry.yaml`: `filter_size_map`, `mapping.save_voxel_size`, `publish.dense_publish_en`
- `src/localization/FAST_LIO_ROBOAIRY/src/laserMapping.cpp`: `save_to_pcd()` 使用 `pcl_wait_pub` 写入 `map_file_path`（约 724 行）
- `src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py`: PCD 路径
- `src/registration/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py`: PCD 路径

---

## 问题 4：Bag 回放建图 cartographer 只看到第一帧

### 现象

重放 `/home/ros/dataset/robosenseAiry/mapping` bag 时，cartographer 只能看到第一帧建图（启动时建的初始空 submap），之后无任何建图/位姿。pc2l 终端持续报：

```
Warning: TF_OLD_DATA ignoring data from the past for frame base_footprint at time 11865.299657 ...
```

### 根因

这个 bag 录制于 `use_lidar_clock=true` 时代，消息时间戳 ≈ **11865s**（雷达坏时钟），与任何墙钟都不一致。pc2l 的 tf2 buffer 判旧逻辑 `now() - stamp > cache_time(10s)` 把每帧 `odom→base_footprint` TF 当作"过去的数据"丢弃 → 永远发不出 `/scan` → cartographer 只有初始空 submap，看不到后续建图。

| 场景 | now() | 消息戳 | now() - stamp |
|------|-------|--------|----------------|
| 之前（墙钟） | 1.7e9 或 3.3h | 11865 | ≫ 10s → **TF_OLD_DATA** |
| 现在（桥式模拟时钟） | 11865+ | 11865 | ≈ 0 → **TF 有效** |

### 为什么 `ros2 bag play --clock` 不行

- Humble 里 `--clock` 带**可选** `[Hz]` 参数，`--clock <bag>` 会把 bag 路径当 Hz 报错；正确写法 `ros2 bag play <bag> --clock`。
- 但即使写对，`--clock` 发布的是**录制起始墙钟**（1785481507），与消息戳差 1.7e9 秒 → 依旧 TF_OLD_DATA。**对这个 bag 本质无效。**

### 解决方案

`scripts/bag_clock_bridge.py` 订阅 bag 内 topic，把最新消息戳以 50Hz RELIABLE 发布为 `/clock`。全链路 `use_sim_time=true` 后各节点 `now() == 消息戳 (~11865)`，TF 不再被判旧 → `/scan` 恢复 → cartographer 正常建图。

附带修复了 FAST-LIO 的隐藏坑：其处理由 `create_timer(get_clock())` 驱动（`laserMapping.cpp:1085`），无推进的 /clock 时 `now()=0`、timer 永不触发，连里程计都不出；桥提供 /clock 后 timer 才工作。

**验证结果（tmux 全链路实测）：** `/registered_scan` ~3.5Hz、`/scan` 360° 正常、cartographer 持续 10Hz 处理 scan 并 `Inserted submap`、`map→odom` TF 正常、pc2l **零 TF_OLD_DATA 错误**。

### 代码位置

- `scripts/robosense_mapping_bag.sh` — bag 回放建图脚本（bag 不带 `--clock` + 桥 + FAST-LIO + pc2l zlim + cartographer）
- `scripts/bag_clock_bridge.py` — `/clock` 桥（跟随 bag 消息时间戳）
- `src/planner/nav2_planner/launch/pointcloud_to_laserscan_launch_zlim.py` — 显式 `use_sim_time` 参数
- `src/gld_robot_description/rviz/nav2_sim.rviz` — 内置 Use Sim Time 的 rviz
- `docs/bag_replay_sim_time.md` — 详细技术文档（根因 / QoS 要点 / 用法）

---

## 当前状态

| 问题 | 状态 |
|------|------|
| Z 轴翻转 → SLAM Toolbox 无数据 | ✅ 已解决 (zflip / zlim) |
| 修正矩阵 XY 互换 → 前进方向不对 | ✅ 已修复 (绕 X 轴 180°) |
| 位姿未修正 → 点云和 odom 不一致 | ✅ 已修复 (odometry 修正) |
| 重定位 PCD 太稀疏 → KISS-Matcher 失败 | ✅ 已解决 (pcl_wait_pub 保存即可满足重定位；稠密 dense_map.pcd 已停用) |
| Bag 回放 cartographer 只看到第一帧 → TF_OLD_DATA | ✅ 已修复 (bag_clock_bridge 模拟时钟) |

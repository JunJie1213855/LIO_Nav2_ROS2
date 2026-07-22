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

#### A. 减小 ikd-tree 体素尺寸

`robosenseAiry.yaml`:

| 参数 | 旧值 | 新值 | 效果 |
|------|------|------|------|
| `filter_size_map` | 0.5 | **0.15** | ikd-tree 密度提升 ~37× |
| `filter_size_surf` | 0.5 | **0.2** | surf 特征点更密 |
| `mapping.save_voxel_size` | — (默认 0.2) | **0.1** | 保存时不再过度降采样 |

权衡：ikd-tree 内存占用增加 ~37×，大面积建图时需注意内存。

#### B. 额外保存稠密累积点云（推荐）

在 `laserMapping.cpp` 的 `SigHandle()` 中，新增保存 `dense_map.pcd`：

```
                          方案A: ikd-tree            方案B: dense_map
                          ──────────────            ────────────────
feats_undistort ──→ ikd-tree(0.15m体素)          feats_undistort ──→ pcl_wait_save 累积
                         │                                              │
                         ▼                                              ▼
                    robo_map.pcd             Ctrl+C → 0.05m体素滤波 → dense_map.pcd
                   (~5万点，稀疏)                                  (~10-50万点，稠密)
```

方案 B 的优势：
- 不改 ikd-tree 参数（不影响 SLAM 实时性能）
- 累积所有历史帧的 `laserCloudWorld`，包含多次扫描的冗余信息
- 密度远超方案 A，FPFH 特征区分力更强
- 可用更小的体素（0.05m）滤波，细节保留更好

#### C. 重定位 launch 文件适配

两个 launch 文件均改为**优先加载稠密地图，不存在则 fallback**：

```python
dense_pcd = os.path.join(..., "PCD", "dense_map.pcd")
legacy_pcd = os.path.join(..., "pcd", "robo_map.pcd")
pcd_path = dense_pcd if os.path.exists(dense_pcd) else legacy_pcd
```

### 代码位置

- `src/localization/FAST_LIO_ROBOAIRY/config/robosenseAiry.yaml`: `filter_size_map`, `mapping.save_voxel_size`
- `src/localization/FAST_LIO_ROBOAIRY/src/laserMapping.cpp`: `SigHandle()` 稠密地图保存
- `src/registration/global_relocalization_kiss_matcher/launch/global_kiss_matcher_relocalization_launch.py`: PCD 路径
- `src/registration/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py`: PCD 路径

---

## 当前状态

| 问题 | 状态 |
|------|------|
| Z 轴翻转 → SLAM Toolbox 无数据 | ✅ 已解决 (zflip / zlim) |
| 修正矩阵 XY 互换 → 前进方向不对 | ✅ 已修复 (绕 X 轴 180°) |
| 位姿未修正 → 点云和 odom 不一致 | ✅ 已修复 (odometry 修正) |
| 重定位 PCD 太稀疏 → KISS-Matcher 失败 | ✅ 已修复 (dense_map.pcd + 参数调优) |

# RoboSense Airy Z 轴翻转问题与解决方案

## 问题现象

使用 RoboSense Airy 实机建图时，SLAM Toolbox **无法构建 2D 占用栅格地图**。FAST-LIO 的三维点云建图正常，但 3D→2D 转换后 `/scan` 数据不足以支撑 SLAM Toolbox 的 scan matching。

## 根因分析

### 1. Airy 的外参矩阵

RoboSense Airy 的 LiDAR-IMU 外参矩阵（`robosenseAiry.yaml`）将 Z 轴翻转了：

```yaml
# src/localization/FAST_LIO_ROBOAIRY/config/robosenseAiry.yaml
mapping:
  extrinsic_R: [0.0, -1.0, 0.0,
                -1.0,  0.0, 0.0,
                 0.0,  0.0, -1.0]
```

这个 3×3 旋转矩阵的变换为：

```
[ 0  -1   0 ]   [x_lidar]     [-y_lidar]    =  x_imu
[ -1   0   0 ] × [y_lidar]  =  [-x_lidar]    =  y_imu
[ 0   0  -1 ]   [z_lidar]     [-z_lidar]    =  z_imu
```

**关键**：第三行 `[0, 0, -1]` 导致 `Z_imu = -Z_lidar`，即 **Z 轴被翻转**。

### 2. 翻转后的坐标系含义

| 原始 LiDAR 帧 | 变换后 IMU/Body 帧 |
|:---:|:---:|
| `+Z_lidar = 朝上` | `+Z_imu = 朝下` |
| `+X_lidar = 朝前` | `+X_imu = 朝左` |
| `+Y_lidar = 朝左` | `+Y_imu = 朝后` |

经过 FAST-LIO 处理后，`/cloud_registered` 发布在 `camera_init` 系下，**保留了 Z 轴翻转的特性**。

### 3. 为什么 FAST-LIO 建图正常但 SLAM Toolbox 不行

| 模块 | 是否依赖 Z 轴 | 详情 |
|:---|:---:|:---|
| FAST-LIO | 不影响 | 使用点云的几何特征（平面/边缘）做 ICP 配准，只关心**相对几何形状**，不依赖 Z 轴的绝对方向 |
| `pointcloud_to_laserscan` | **受影响** | 按 `min_height ~ max_height` 做高度切片。如果切片范围在正常 Z 区域（如 `0.2~1.0m`），但实际点云在翻转后的负 Z 区域（`-1.0~-0.3m`），则**切不到任何点** |
| SLAM Toolbox | **受影响** | 依赖 2D scan 做 scan matching，没有 scan 数据就无法建图 |

### 4. 数据流详解

```
Airy LiDAR (raw)
  │  Z_lidar 朝上，墙壁在 +0.5m
  ├─ extrinsic_R: Z 翻转
  ▼
IMU Body 帧
  │  Z_imu = -Z_lidar（朝下），墙壁变为 -0.5m
  ├─ FAST-LIO 运动补偿 + 里程计
  ▼
/cloud_registered (camera_init 帧)
  │  点云 Z 值分布：
  │    天花板  → Z ≈ -2.5m
  │    桌椅    → Z ≈ -1.5m ~ -0.5m
  │    墙壁    → Z ≈ -0.5m ~ -1.5m
  │    地面    → Z ≈ -0.1m ~ 0m
  ├─ lio_interface
  ▼
/registered_scan (odom 帧)
  ├─ pointcloud_to_laserscan
  │     min_height= 0.2, max_height= 1.0  ← ❌ 切不到任何点！
  │     因为点云全部在负 Z 区域
  ▼
/scan → SLAM Toolbox → ❌ 无有效数据，建图失败
```

## 解决方案

### 方案一（推荐）：`airy_unflip.py` — 施加 extrinsic_R 逆旋转恢复 Z 轴

通过一个独立节点，在 body 帧对点云施加 `R_ext` 的逆旋转，
**彻底消除 Z 翻转**，使修正后的点云 Z 轴恢复朝上。
下游所有节点（`pointcloud_to_laserscan`、`ground_ceiling_filter` 等）
均可使用**正常正值**高度参数，无需任何特殊配置。

```
FAST-LIO → /cloud_registered (Z 翻转)
            │
            ▼
        airy_unflip.py     ←  world→body→R_inv→body→world
            │
            ▼
        /cloud_registered_unflipped (Z 朝上，正常坐标系)
            │
            ▼
        lio_interface / sensor_scan_generation / ...
```

**用法**：

```bash
# 在 robosense_mapping_real.sh 中，FAST-LIO 之后启动
/usr/bin/python3 scripts/airy_unflip.py --ros-args -p use_sim_time:=False

# 然后 lio_interface 订阅修正后的点云
ros2 launch lio_interface fastlio_lio_interface_launch.py \
  cloud_topic:=/cloud_registered_unflipped
```

**原理**：

```python
# 对每个点（body 帧）：
pt_corrected_b = R_inv @ pt_b    # R_inv = R_ext^T
# Airy 的 R_inv:
#   [ 0, -1,  0]   [x]     [-y]
#   [-1,  0,  0] × [y]  =  [-x]
#   [ 0,  0, -1]   [z]     [-z]  ← Z 翻转被消除
# 结果：Z_corrected = -Z_flipped（恢复朝上）
```

配置 `Pointcloud2d_3d.yaml` 为标准正值即可：

```yaml
min_height: 0.2
max_height: 1.0
```

### 方案二：`Pointcloud2d_3d.yaml` 中将高度切片范围也翻转

```yaml
# src/me_nav2_bringup/config/Pointcloud2d_3d.yaml
Pointcloud2d_3d:
  ros__parameters:
    target_frame: "livox_frame"
    transform_tolerance: 0.01

    # Airy 适配：由于 extrinsic_R 将 Z 轴翻转
    #   • 正常雷达 Z 朝上，墙壁在 +0.2~+1.0m → min_height=0.2, max_height=1.0
    #   • Airy 雷达 Z 朝下，墙壁在 -1.0~-0.3m → min_height=-2.0, max_height=-0.3
    min_height: -2.0    # 切片下界（翻转后对应"天花板侧"）
    max_height: -0.3    # 切片上界（翻转后对应"地面侧"，滤除机器人附近地面点）

    angle_min: -3.14159
    angle_max: 3.14159
    angle_increment: 0.0087
    scan_time: 0.1
    range_min: 0.05
    range_max: 70.0
    use_sim_time: true
```

### 原理示意图

```
翻转后的点云 Z 值分布（camera_init/odom 帧下）：

    +Z（朝下）
    ↑
     0.0m  ──────────── 地面 / 机器人安装高度
    -0.3m  ── max_height ── 切片上界（保留下方边界，滤除地面杂点）
           ████████████████
    -1.0m  ██ 墙壁/障碍物 ██  ← 3D→2D 切片有效区域
           ████████████████
    -2.0m  ── min_height ── 切片下界（保留上方边界）
    -2.5m  ──────────── 天花板
    
通过 min_height=-2.0, max_height=-0.3，
恰好捕捉翻转后"机器人高度附近"的点云，生成有效的 2D scan。
```

## 两种方案对比

| | 方案一：`airy_unflip.py` | 方案二：翻转高度阈值 |
|:---|:---|:---|
| 原理 | body 帧施加 R_ext^{-1}，彻底消除 Z 翻转 | 把 `min_height/max_height` 设为负值 |
| 修正后 Z 轴 | 朝上（正常） | 仍朝下 |
| 下游节点配置 | 全部用标准正值 | 所有 Z 相关参数需用负值 |
| 对其他节点的影响 | 无（`ground_ceiling_filter` 等正常用） | 所有依赖 Z 的节点均需适配 |
| 适用场景 | Airy 专用，一劳永逸 | 快速验证 / 不想加额外节点 |
| 额外延迟 | ~1ms（逐点矩阵乘法） | 0 |

## 正常雷达 vs Airy 修复后

| | Livox MID-360 | Airy（方案一 unflip 后） | Airy（方案二 翻阈值） |
|:---|:---|:---|:---|
| `extrinsic_R` (第三行) | `[0, 0, 1]` | `R_inv` 已消除 | `[0, 0, -1]` |
| 墙壁在点云中的 Z 值 | `+0.2 ~ +1.0` | `+0.2 ~ +1.0` | `-1.0 ~ -0.3` |
| `min_height` | `0.2` | `0.2` | `-2.0` |
| `max_height` | `1.0` | `1.0` | `-0.3` |
| SLAM Toolbox | ✅ | ✅ | ✅ |


## 方案一部署指南

### 数据流

```
Airy LiDAR (Z_lidar 朝上)
  |  rslidar_sdk
  v
/rslidar_points + /rslidar_imu_data
  |  FAST-LIO (extrinsic_R Z 翻转)
  v
/cloud_registered (camera_init 帧, Z 朝下)
  |  airy_unflip.py (R_inv = R_ext^T)
  v
/cloud_registered_unflipped (camera_init 帧, Z 恢复朝上)
  |  lio_interface (cloud_topic:=/cloud_registered_unflipped)
  v
/registered_scan (odom 帧, Z 朝上)
  |  sensor_scan_generation
  +-- /odom + odom->base_footprint TF
  v
pointcloud_to_laserscan
  |  min_height=0.2, max_height=1.0  <-- 标准正常值
  v
/scan -> SLAM Toolbox [OK]
```

### 方式 A：使用集成脚本（推荐）

```bash
# 直接运行，已包含 airy_unflip.py + lio_interface(cloud_topic)
./scripts/robosense_mapping_real_new.sh
```

### 方式 B：手动命令行

```bash
# 终端 1: FAST-LIO
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py

# 终端 2: Z 轴修正
/usr/bin/python3 scripts/airy_unflip.py --ros-args -p use_sim_time:=False

# 终端 3: lio_interface（订阅修正后点云）
ros2 launch lio_interface fastlio_lio_interface_launch.py \
  cloud_topic:=/cloud_registered_unflipped
```

### 前提：lio_interface 支持 cloud_topic

`fastlio_lio_interface_launch.py` 支持 `cloud_topic` 参数重映射硬编码的 `/cloud_registered`：

```python
# key change in fastlio_lio_interface_launch.py
remappings=[
    ('/cloud_registered', LaunchConfiguration('cloud_topic')),
],
```

不传 `cloud_topic` 时默认仍为 `/cloud_registered`，向后兼容。

### 验证方法

```bash
# 1. 确认 unflip 输出正常
ros2 topic hz /cloud_registered_unflipped

# 2. 确认 2D scan 有数据
ros2 topic echo /scan --once | grep ranges

# 3. SLAM Toolbox 建图
rviz2  # 查看 /map topic
```

### airy_unflip.py 参数

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `input_cloud` | `/cloud_registered` | 输入点云（Z 翻转） |
| `input_odom` | `/Odometry` | FAST-LIO 里程计 |
| `output_cloud` | `/cloud_registered_unflipped` | 输出点云（Z 恢复朝上） |
| `extrin_r` | Airy 默认外参 | `R_lidar->imu`，9 个浮点数逗号分隔 |

### 变换步骤

```
对 /cloud_registered 中的每个点 pt_w (world 帧):

  1) world -> body
     pt_b = R_w2b @ (pt_w - T)

  2) body 帧施加 R_inv（消除 Z 翻转 & X/Y 交换）
     pt_corrected_b = R_inv @ pt_b
     其中 R_inv = R_ext^T

  3) body -> world
     pt_corrected_w = R_b2w @ pt_corrected_b + T

结果: 同一 world 帧，Z 轴恢复朝上
```

## 注意事项

1. **方案一依赖**：`fastlio_lio_interface_launch.py` 需要 `cloud_topic` 参数支持（已实现）。
2. **方案二局限**：若同时使用 `ground_ceiling_filter.py` 等依赖 Z 的节点，负 Z 会使其逻辑出错，必须用方案一。
3. 两种方案都只影响输出点云，不影响 FAST-LIO 三维建图质量。
4. Airy 本身 FOV 为 120°，`angle_min/max` 建议保持 `-pi ~ +pi`。
5. 如果将来重新标定 `extrinsic_R` 使 Z 轴不翻转（第三行改为 `[0, 0, 1]`），两种方案都无需使用。
6. 统一关闭：`./scripts/kill_all.sh` 自动识别并关闭所有节点（含 `airy_unflip.py`）。

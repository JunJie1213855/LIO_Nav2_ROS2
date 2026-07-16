# sensor_scan_generation

LIO 点云与里程计的**桥接节点**，负责将 FAST-LIO / Point-LIO 的内部输出转换为标准 ROS 2 接口，是数据管线的关键枢纽。

## 作用

`fast_lio` / `point_lio` 只发布原始的 `/cloud_registered`（点云）和非标准 `/Odometry`（里程计），不会发布标准 TF `odom → base_footprint` 和标准 `/odom` 话题。

`sensor_scan_generation` 负责：

1. **转发 `/registered_scan`** — 从 LIO 接收点云，重新发布供重定位和 3D→2D 切片使用
2. **发布标准 `/odom`** — 将 LIO 里程计转换为 `nav_msgs/Odometry`，供 Nav2 使用
3. **发布 TF `odom → base_footprint`** — 连接 TF 树的核心环节，无此链路 Nav2 无法定位

## 数据流

```
FAST-LIO / Point-LIO
    │
    ├── /cloud_registered ──► sensor_scan_generation
    └── /Odometry        ──► sensor_scan_generation
                                    │
                                    ├── /registered_scan ──► KISS-Matcher 重定位
                                    │                        pointcloud_to_laserscan
                                    │
                                    ├── /odom ─────────────► Nav2 bt_navigator
                                    │
                                    └── TF: odom→base_footprint ──► TF 树
```

## 订阅与发布

| 方向 | 话题/类型 | 用途 |
|------|-----------|------|
| 订阅 | `/registered_scan` (PointCloud2) | LIO 输出点云 |
| 订阅 | `/registered_odometry` (Odometry) | LIO 里程计原始数据 |
| 发布 | `/registered_scan` (PointCloud2) | 转发给下游的重定位和切片节点 |
| 发布 | `/odom` (Odometry) | 标准里程计，Nav2 使用 |
| 发布 | `odom → base_footprint` (TF) | 里程计位姿变换 |

## 启动

```bash
# 仿真模式 (use_sim_time:=True)
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py

# 实机模式 (use_sim_time:=False)
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=False
```

## 依赖

- **上游**：FAST-LIO (`/cloud_registered`, `/Odometry`) 或 Point-LIO (`/aft_mapped_to_init`)
- **下游**：`global_relocalization_kiss_matcher`、`pointcloud_to_laserscan`、Nav2

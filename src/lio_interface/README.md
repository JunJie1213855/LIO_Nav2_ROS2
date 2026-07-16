# lio_interface

**TF 桥接节点**，将 LIO 内部里程计转换为标准 `odom → base_footprint` TF。

## 作用

FAST-LIO (`/Odometry`) 和 Point-LIO (`/aft_mapped_to_init`) 不会直接发布标准 TF。`lio_interface` 订阅这些话题并发布 `odom → base_footprint`。

## 两种模式

| 模式 | 启动文件 | 订阅 |
|------|---------|------|
| FAST-LIO | `fastlio_lio_interface_launch.py` | `/Odometry` |
| Point-LIO | `pointlio_lio_interface_launch.py` | `/aft_mapped_to_init` |

## 启动

```bash
# FAST-LIO 模式（默认）
ros2 launch lio_interface lio_interface_launch.py

# Point-LIO 模式
ros2 launch lio_interface pointlio_lio_interface_launch.py
```

## 注意

与 `sensor_scan_generation` 同步发布 `odom → base_footprint`，两者必须一致。

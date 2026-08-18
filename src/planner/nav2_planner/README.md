# nav2_planner

**Nav2 导航集成中心**，包含启动文件、参数配置、地图、PCD 和 RViz 界面。

## 核心文件

| 文件 | 用途 |
|------|------|
| `launch/my_nav2_launch.py` | Nav2 主启动：map_server + navigation_launch + RViz |
| `launch/pointcloud_to_laserscan_launch.py` | 3D→2D 激光扫描切片 |
| `config/nav2_params.yaml` | DWB 局部规划器 + Navfn 全局规划器 + costmap 参数 |
| `config/slam_toolbox_params.yaml` | SLAM Toolbox 在线建图参数 |
| `config/Pointcloud2d_3d.yaml` | 3D→2D 切片高度和角度分辨率 |
| `map/` | 2D OccupancyGrid 地图 (`.yaml` + `.pgm`) |
| `pcd/` | 3D 先验点云地图 (`.pcd`) |
| `rviz/nav2.rviz` | RViz 可视化配置 |

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 最大线速度 | `0.26 m/s` | DWB `max_vel_x` |
| 最大角速度 | `1.0 rad/s` | DWB `max_vel_theta` |
| 目标容差 XY | `0.035 m` | `xy_goal_tolerance` |
| 目标容差 Yaw | `10°` | `yaw_goal_tolerance` |
| 膨胀半径 | `0.55 m` | 障碍物安全距离 |
| 机器人轮廓 | `0.42×0.39 m` | 矩形 footprint |

## 启动

```bash
ros2 launch nav2_planner my_nav2_launch.py
```

## 依赖

- 上游：`/scan`、`map → odom` TF、`/map`
- 输出：`/cmd_vel` → 底盘

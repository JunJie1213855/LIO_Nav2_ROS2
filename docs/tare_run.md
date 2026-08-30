# TARE Planner 编译与运行指南（tare_run.md）

> TARE（Technically-Aware Robotic Exploration）自主探索规划器。
> 在 Gazebo + FAST-LIO 仿真环境中，让机器人自动探索未知室内环境、规划覆盖路径并局部避障。
> 环境：Ubuntu 22.04 + ROS 2 Humble + Docker（容器名 `lio_nav2`，工作空间挂载到 `/ws`）。

---

## 1. 管线架构

```
Gazebo (get_urdf)
   └─> FAST-LIO (里程计 + /cloud_registered)
          ├─> lio_interface          (转发/生成 /odom 与 /registered_scan)
          │       └─> sensor_scan_generation (odom 帧点云 + TF)
          │
          └─> TARE Planner (tare_planner_node)
                 ├─ 订阅 /odom (里程计)
                 ├─ 订阅 /registered_scan (odom 帧点云)
                 ├─ 构建 rolling occupancy grid + 视点图 + TSP 全局/局部规划
                 └─ 发布 /way_point (探索目标点)
                        └─> waypoint_follower (带局部避障) → /cmd_vel
```

| 节点 | 包 | 作用 |
|------|-----|------|
| `get_urdf` | `get_urdf` | Gazebo 仿真世界与机器人模型 |
| `fast_lio_robosense` | `fast_lio_robosense` | LiDAR 惯性里程计 + 点云配准 |
| `lio_interface` | `lio_interface` | 里程计话题转发与格式转换 |
| `sensor_scan_generation` | `sensor_scan_generation` | 生成 odom 帧点云 |
| `tare_planner_node` | `tare_planner` | 探索规划核心 |
| `waypoint_follower` | `tare_planner` | 航点跟随 + 局部避障 → `/cmd_vel` |

---

## 2. 构建

### 2.1 源码位置与分支

TARE 官方仓库 `caochao39/tare_planner` 已有 ROS 2 迁移分支 `humble-jazzy`，直接使用：

```bash
# 本仓库源码已放在 src/planner/tare_planner，无需重新 clone
cd /ws/src/planner/tare_planner
git branch          # 确认当前在 humble-jazzy
git checkout humble-jazzy   # 若不在该分支则切换
```

### 2.2 编译命令

```bash
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ws &&
  colcon build --packages-select tare_planner --symlink-install
"
```

> 首次编译约 2 分钟（or-tools 第三方头文件会输出大量 warning，属正常）。
> 若 CMake 报源码路径不匹配，先清理旧产物：
> ```bash
> docker exec lio_nav2 bash -c "rm -rf /ws/build/tare_planner /ws/install/tare_planner"
> ```

### 2.3 依赖说明

`humble-jazzy` 分支的 CMakeLists 已迁移到 `ament_cmake`，但有两处 Humble 兼容性问题需要处理（本仓库已修复）：

| 问题 | 修复 |
|------|------|
| `explore.launch` 使用 `SetParameter`（Jazzy 新增，Humble 无此 API） | 删除 `SetParameter` 导入与使用 |
| CMakeLists 残留 `${catkin_LIBRARIES}` | 删除 catkin 引用 |

---

## 3. 运行

### 3.1 一键启动（推荐）

```bash
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/gazebo_tare.sh"
```

脚本自动完成：
1. 杀掉残留的 Gazebo/LIO/TARE 进程、关闭旧 tmux 会话
2. 启动 Gazebo 仿真世界
3. 启动 FAST-LIO 里程计
4. 启动 `lio_interface`、`sensor_scan_generation`
5. 启动 TARE 探索规划 + waypoint_follower
6. 启动 RViz

### 3.2 tmux 窗口

```bash
docker exec -it lio_nav2 tmux attach -t tare_gz
```

| 窗口 | 内容 |
|------|------|
| `Gazebo` | Gazebo 仿真世界 |
| `FAST-LIO` | LiDAR 惯性里程计 |
| `lio_if` | 里程计接口 |
| `sensor` | 点云生成 |
| `TARE` | TARE 探索规划器 + waypoint_follower |
| `RViz` | 可视化 |

切换窗口：`Ctrl+B` 后按对应数字键。关闭整个会话：`docker exec lio_nav2 tmux kill-server`。

### 3.3 手动启动（分步调试用）

```bash
docker exec -it lio_nav2 bash
cd /ws && source install/setup.bash

# 1. Gazebo
ros2 launch get_urdf get_urdf_launch.py rviz:=false

# 2. FAST-LIO
ros2 launch fast_lio_robosense mapping.launch.py rviz:=false

# 3. 里程计接口 + 点云生成
ros2 launch lio_interface lio_interface_launch.py
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py

# 4. TARE
ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true

# 5. RViz
rviz2 -d /ws/src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz
```

---

## 4. TARE 配置（config/gazebo_indoor.yaml）

配置位于 `src/planner/tare_planner/src/tare_planner/config/gazebo_indoor.yaml`，关键参数：

### 4.1 话题映射

| 参数 | 值 | 说明 |
|------|-----|------|
| `sub_state_estimation_topic_` | `/odom` | 里程计输入 |
| `sub_registered_scan_topic_` | `/registered_scan` | odom 帧配准点云 |
| `pub_waypoint_topic_` | `/way_point` | 探索目标点输出 |
| `kAutoStart` | `true` | 启动后自动开始探索 |

### 4.2 探索行为

| 参数 | 值 | 说明 |
|------|-----|------|
| `kLookAheadDistance` | 5.0 | 前瞻点距离 |
| `kExtendWayPoint` | `false` | **关闭航点延长**（延长点无碰撞检测，会穿墙） |
| `kUseLineOfSightLookAheadPoint` | `true` | 前瞻点做视线检测 |
| `kNoExplorationReturnHome` | `true` | 探索完成后返回起点 |
| `kRushHome` | `true` | 快速返航 |
| `kSensorRange` | 7.5 | 传感器探测范围 |

### 4.3 性能参数（已针对仿真调优）

| 参数 | 值 | 说明 |
|------|-----|------|
| `kPointCloudCellSize` | 10.0 | 点云管理器单元尺寸（默认 18） |
| `kPointCloudManagerNeighborCellNum` | 3 | 相邻单元数（默认 5） |
| `rolling_occupancy_grid/resolution` | 0.3 | 滚动占据栅格分辨率（默认 0.3） |

> Rolling Occupancy Grid 范围 = `kPointCloudCellSize × kPointCloudManagerNeighborCellNum`
> 默认 18×5=90m 在 0.2m 分辨率下产生 910 万网格，RayTrace 极慢导致主线程阻塞。
> 调小后范围 30m，网格降到约 15 万，速度提升 60 倍。

### 4.4 waypoint_follower 避障参数（launch 中配置）

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_linear_vel` | 0.5 | 最大线速度 |
| `max_angular_vel` | 1.0 | 最大角速度 |
| `arrival_dist` | 0.3 | 到达判定距离 |
| `stop_dist` | 0.45 | 前方障碍物停车距离 |
| `slow_dist` | 0.8 | 开始减速距离 |
| `robot_half_width` | 0.35 | 机器人半宽 + 余量 |
| `check_height_min` | 0.05 | 障碍物检查高度下限（**排除地面点**） |
| `check_height_max` | 0.60 | 障碍物检查高度上限 |

---

## 5. RViz 可视化

RViz 配置：`src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz`（Fixed Frame 已改为 `odom`）。

| 显示项 | 话题 | 含义 |
|--------|------|------|
| OverallMap | `/overall_map` | 整体地图点云 |
| ExploredAreas | `/explored_areas` | 已探索区域 |
| ExploringSubspaces | `/tare_visualizer/exploring_subspaces` | 探索中的子空间 |
| SelectedViewPoints | `/selected_viewpoint_vis_cloud` | 选中的候选视点 |
| GlobalPath | `/global_path` | 全局探索路径（青色） |
| LocalPath | `/local_path` | 局部路径（蓝色） |
| Waypoint | `/way_point` | 当前目标航点（紫色大球） |
| ObjectSurfacesToCover | `/uncovered_cloud` | 待覆盖物体表面 |
| FrontierSurfacesToCover | `/uncovered_frontier_cloud` | 待覆盖前沿表面 |
| LocalPlanningHorizon | `/tare_visualizer/local_planning_horizon` | 局部规划范围框 |

---

## 6. 常见问题排查

### 6.1 机器人撞障碍物

1. 确认 `kExtendWayPoint: false`（航点延长无碰撞检测）
2. 确认 `waypoint_follower` 订阅到 `/registered_scan`（`ros2 topic info /registered_scan -v`）
3. 调整 `stop_dist` / `robot_half_width` 适配机器人尺寸

### 6.2 机器人停滞不动

多半是地面点被误判为障碍物。确认 `check_height_min ≥ 0.05`（排除地面）。

### 6.3 无可视化数据

1. 确认所有数据帧为 `odom`（TARE 源码 `kWorldFrameID` 已改为 `odom`）
2. 确认 RViz Fixed Frame 为 `odom`
3. 检查话题频率：`ros2 topic hz /overall_map /way_point`

### 6.4 首轮规划慢

TARE 首次完整规划周期（`UpdateGlobalRepresentation` + TSP）约需 5 秒，期间主线程阻塞，属正常现象，后续迭代更快。

### 6.5 查看调试输出

`execute()` 前 5 次调用带 `[DEBUG]` 日志，可通过 TARE 窗口观察初始化与 keypose 更新状态。

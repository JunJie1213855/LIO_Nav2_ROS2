# TARE Planner 运行指南（tare_run.md）

> TARE（Technically-Aware Robotic Exploration）自主探索规划器。
> 在 Gazebo 仿真环境中，让机器人自动探索未知室内环境、规划覆盖路径并局部避障。
> 环境：Ubuntu 22.04 + ROS 2 Humble + Docker（容器名 `lio_nav2`，工作空间挂载到 `/ws`）。
>
> 本工作空间支持两种 LIO 里程计后端，通过不同启动脚本切换：
> - **FAST-LIO**：`scripts/gazebo_tare.sh`
> - **Point-LIO**：`scripts/gazebo_tare_pointlio.sh`

---

## 1. 管线架构

### 1.1 FAST-LIO 作为里程计（`gazebo_tare.sh`）

```
Gazebo (get_urdf)
   └─> FAST-LIO (fast_lio mapping.launch.py)
          ├─ 订阅 /livox/lidar + /livox/imu
          └─ 发布 /Odometry + /cloud_registered
                 └─> lio_interface (默认 fastlio)
                        └─> sensor_scan_generation → /registered_scan + /odom
                                └─> TARE (tare_planner_node) → /way_point
                                        └─> waypoint_follower → /cmd_vel
```

### 1.2 Point-LIO 作为里程计（`gazebo_tare_pointlio.sh`）

```
Gazebo (get_urdf)
   └─ 发布 /livox/lidar (PointCloud2, 无 ring/time) + /livox/imu
   └─> ign_sim_pointcloud_tool（点云格式转换器）
          └─ /livox/lidar → /velodyne_points（注入 ring + time 字段）
                 └─> Point-LIO (point_lio, mid360_sim.yaml)
                        ├─ 订阅 /velodyne_points + /livox/imu
                        └─ 发布 /aft_mapped_to_init + /cloud_registered
                               └─> lio_interface (lio_type:=pointlio)
                                      └─> sensor_scan_generation → /registered_scan + /odom
                                              └─> TARE → /way_point
                                                      └─> waypoint_follower → /cmd_vel
```

> **两种 LIO 的关键差异**：FAST-LIO 里程计话题是 `/Odometry`；Point-LIO 是 `/aft_mapped_to_init`。
> `lio_interface` 通过 `lio_type` 参数自动匹配：`fastlio` / `pointlio` / `superlio`。

| 节点 | 包 | 作用 |
|------|-----|------|
| `get_urdf` | `get_urdf` | Gazebo 仿真世界与机器人模型 |
| `fast_lio` | `fast_lio` | FAST-LIO 里程计（`/Odometry`） |
| `ign_sim_pointcloud_tool` | `ign_sim_pointcloud_tool` | Point-LIO 仿真专用：点云转 velodyne 格式并注入 ring/time |
| `point_lio` | `point_lio` | Point-LIO 里程计（`/aft_mapped_to_init`） |
| `lio_interface` | `lio_interface` | 里程计话题转发与 TF 桥接 |
| `sensor_scan_generation` | `sensor_scan_generation` | 生成 `/registered_scan` + `/odom` |
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

### 2.3 本仓库已修复的编译/运行问题（重要）

| 问题 | 现象 | 修复 |
|------|------|------|
| or-tools 缺 `absl/log` 头文件 | 编译报 `fatal error: absl/log/check.h: 没有那个文件或目录` | 补齐 `src/planner/tare_planner/src/tare_planner/or-tools/include/absl/log/`（36 个文件）；根 `.gitignore` 的 `log/` 改为 `/log/`（原规则误忽略了 `include/absl/log/`） |
| OR-Tools ABI 不匹配 | 运行时段错误（exit -11），堆栈在 `IteratedLocalSearchParameters::~...` | 编译用 vendored OR-Tools 9.8.3296 头文件，但运行时加载了 ROS 自带 `ortools_vendor` 的 `libortools.so.9`（v9.9.9999）。在 `tare_planner/CMakeLists.txt` 给 `tsp_solver` 加 `target_link_options(tsp_solver INTERFACE "-Wl,--disable-new-dtags")`，让 RPATH 优先于 `LD_LIBRARY_PATH` |
| `Start time is zero` 退出 | `use_sim_time` 下 `/clock` 未就绪就 `exit(1)` | `sensor_coverage_planner_ground.cpp` 改为等待重试，不再退出 |
| waypoint_follower 朝障碍物方向转 | 机器人卡在障碍物前静止/打转 | 见 §4.4 |

### 2.4 依赖说明

`humble-jazzy` 分支的 CMakeLists 已迁移到 `ament_cmake`，但有两处 Humble 兼容性问题需要处理（本仓库已修复）：

| 问题 | 修复 |
|------|------|
| `explore.launch` 使用 `SetParameter`（Jazzy 新增，Humble 无此 API） | 删除 `SetParameter` 导入与使用 |
| CMakeLists 残留 `${catkin_LIBRARIES}` | 删除 catkin 引用 |

---

## 3. 运行

### 3.1 FAST-LIO 作为里程计：一键启动

```bash
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/gazebo_tare.sh"
```

脚本自动完成：杀残留进程 → 启动 Gazebo → FAST-LIO → `lio_interface` + `sensor_scan_generation` → TARE + waypoint_follower → RViz。

### 3.2 Point-LIO 作为里程计：一键启动

```bash
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/gazebo_tare_pointlio.sh"
```

相比 FAST-LIO 版本多了一个 **点云格式转换器**（`ign_sim_pointcloud_tool`），并把 `lio_interface` 的 `lio_type` 设为 `pointlio`。

脚本关键模块（`gazebo_tare_pointlio.sh` 内已用 `# ============== xxx =======================` 标注）：

| 模块 | 命令要点 | 说明 |
|------|----------|------|
| Gazebo | `ros2 launch get_urdf get_urdf_launch.py rviz:=false` | 发布 `/livox/lidar` + `/livox/imu` |
| convert | `ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args -p pcd_topic:=/livox/lidar -p n_scan:=50 -p horizon_scan:=360 -p ang_bottom:=7.22 -p ang_res_y:=1.248` | `/livox/lidar` → `/velodyne_points`，注入 ring/time |
| Point-LIO | `ros2 launch point_lio point_lio.launch.py rviz:=false point_lio_cfg_dir:=$WS/src/localization/point_lio/config/mid360_sim.yaml` | 订阅 `/velodyne_points`，输出 `/aft_mapped_to_init` |
| lio_if | `ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio` | 把 `/aft_mapped_to_init` 转成 odom TF |
| sensor | `ros2 launch sensor_scan_generation sensor_scan_generation_launch.py` | 发布 `/registered_scan` + `/odom` |
| TARE | `ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true` | 探索规划，发布 `/way_point` |

### 3.3 tmux 窗口

```bash
docker exec -it lio_nav2 tmux attach -t tare_gz
```

| 窗口（FAST-LIO 版） | 窗口（Point-LIO 版） | 内容 |
|------|------|------|
| `Gazebo` | `Gazebo` | Gazebo 仿真世界 |
| `FAST-LIO` | `convert` | Point-LIO 版此处是点云格式转换器 |
| — | `Point-LIO` | Point-LIO 里程计 |
| `lio_if` | `lio_if` | 里程计接口 |
| `sensor` | `sensor` | 点云生成 |
| `TARE` | `TARE` | TARE 探索规划器 + waypoint_follower |
| `RViz` | `RViz` | 可视化 |

切换窗口：`Ctrl+B` 后按对应数字键。关闭整个会话：`docker exec lio_nav2 tmux kill-server`。

### 3.4 手动启动（FAST-LIO 版，分步调试用）

```bash
docker exec -it lio_nav2 bash
cd /ws && source install/setup.bash

# 1. Gazebo
ros2 launch get_urdf get_urdf_launch.py rviz:=false

# 2. FAST-LIO
ros2 launch fast_lio mapping.launch.py rviz:=false

# 3. 里程计接口 + 点云生成
ros2 launch lio_interface lio_interface_launch.py
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py

# 4. TARE
ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true

# 5. RViz
rviz2 -d /ws/src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz
```

### 3.5 手动启动（Point-LIO 版，分步调试用）

```bash
docker exec -it lio_nav2 bash
cd /ws && source install/setup.bash

# 1. Gazebo
ros2 launch get_urdf get_urdf_launch.py rviz:=false

# 2. 点云格式转换器（/livox/lidar → /velodyne_points，注入 ring/time）
ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args \
  -p pcd_topic:=/livox/lidar \
  -p n_scan:=50 -p horizon_scan:=360 \
  -p ang_bottom:=7.22 -p ang_res_y:=1.248

# 3. Point-LIO（仿真配置 mid360_sim.yaml，订阅 velodyne_points）
ros2 launch point_lio point_lio.launch.py rviz:=false \
  point_lio_cfg_dir:=/ws/src/localization/point_lio/config/mid360_sim.yaml

# 4. 里程计接口 + 点云生成
ros2 launch lio_interface lio_interface_launch.py lio_type:=pointlio
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py

# 5. TARE
ros2 launch tare_planner tare_planner_lio_launch.py use_sim_time:=true

# 6. RViz
rviz2 -d /ws/src/planner/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz
```

---

## 4. 配置

### 4.1 TARE 配置（config/gazebo_indoor.yaml）

配置位于 `src/planner/tare_planner/src/tare_planner/config/gazebo_indoor.yaml`，关键参数：

#### 话题映射

| 参数 | 值 | 说明 |
|------|-----|------|
| `sub_state_estimation_topic_` | `/odom` | 里程计输入 |
| `sub_registered_scan_topic_` | `/registered_scan` | odom 帧配准点云 |
| `pub_waypoint_topic_` | `/way_point` | 探索目标点输出 |
| `kAutoStart` | `true` | 启动后自动开始探索 |

#### 探索行为

| 参数 | 值 | 说明 |
|------|-----|------|
| `kLookAheadDistance` | 5.0 | 前瞻点距离 |
| `kExtendWayPoint` | `false` | **关闭航点延长**（延长点无碰撞检测，会穿墙） |
| `kUseLineOfSightLookAheadPoint` | `true` | 前瞻点做视线检测 |
| `kNoExplorationReturnHome` | `true` | 探索完成后返回起点 |
| `kRushHome` | `true` | 快速返航 |
| `kSensorRange` | 7.5 | 传感器探测范围 |

#### 性能参数（已针对仿真调优）

| 参数 | 值 | 说明 |
|------|-----|------|
| `kPointCloudCellSize` | 10.0 | 点云管理器单元尺寸（默认 18） |
| `kPointCloudManagerNeighborCellNum` | 3 | 相邻单元数（默认 5） |
| `rolling_occupancy_grid/resolution` | 0.3 | 滚动占据栅格分辨率（默认 0.3） |

> Rolling Occupancy Grid 范围 = `kPointCloudCellSize × kPointCloudManagerNeighborCellNum`
> 默认 18×5=90m 在 0.2m 分辨率下产生 910 万网格，RayTrace 极慢导致主线程阻塞。
> 调小后范围 30m，网格降到约 15 万，速度提升 60 倍。

### 4.2 Point-LIO 仿真配置（config/mid360_sim.yaml）

配置位于 `src/localization/point_lio/config/mid360_sim.yaml`。与实机配置 `mid360.yaml` 的主要差异：

| 参数 | `mid360_sim.yaml`（仿真） | `mid360.yaml`（实机） | 说明 |
|------|------|------|------|
| `common.lid_topic` | `velodyne_points` | `livox/lidar` | 仿真订阅转换器输出，实机订阅 Livox 驱动 |
| `preprocess.lidar_type` | 2（Velodyne） | 2（Velodyne） | 订阅 PointCloud2，需要 ring/time 字段 |
| `preprocess.scan_line` | 50 | 4 | 仿真垂直扫描线数 |
| `preprocess.timestamp_unit` | 0（秒） | 3（纳秒） | time 字段单位 |
| `mapping.acc_norm` | 9.81 | — | 仿真 Gazebo IMU 单位为 m/s² |
| `mapping.gravity` | Z 轴取反 | — | 仿真重力方向与实机相反 |

> **`lidar_type` 枚举值在 FAST-LIO 和 Point-LIO 中定义不同，配置文件不可混用。**

### 4.3 waypoint_follower 避障参数（launch 中配置）

参数在 `tare_planner_lio_launch.py` 中传给 `waypoint_follower.py`：

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

### 4.4 waypoint_follower 避障修复（已改）

`src/planner/tare_planner/src/tare_planner/scripts/waypoint_follower.py` 有两处修复：

1. **左右扇区赋值反了**：点云转 body 帧后 `ly > 0` 是机器人**左侧**，但原代码把 `ly<0` 记成 `obs_left`、`ly>0` 记成 `obs_right`，导致「朝开阔侧转」的方向反了，机器人在障碍物前会朝障碍物那边转、卡死。已交换赋值（`obs_left`=左、`obs_right`=右）。

2. **绕障逻辑改为「绕行 + 前进」**：正前方被挡（`obs_center < stop_dist`）时朝开阔侧硬转，转开后再前进；近处有障碍（`nearest < slow_dist`）时，目标在开阔侧就朝目标走，否则朝开阔侧绕行（沿墙滑行），避免在原地来回振荡。

> 修改后需重装脚本（或 `colcon build --packages-select tare_planner`）再重启 `waypoint_follower`。

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

### 6.2 机器人停滞不动 / 在障碍物前打转

1. 确认 `waypoint_follower.py` 已是修复后的版本（§4.4），并已重装脚本
2. 确认地面点被排除：`check_height_min ≥ 0.05`

### 6.3 Point-LIO 报 `Failed to find match for field 'time'/'ring'`

说明 Point-LIO（Velodyne 模式）收到的点云缺少 `ring` 和 `time` 字段。常见原因：

1. **没开转换器** `ign_sim_pointcloud_tool`（`/livox/lidar` 原始点云只有 x/y/z/intensity）
2. **用了默认配置** `mid360.yaml`（`lid_topic: livox/lidar`）而不是 `mid360_sim.yaml`（`lid_topic: velodyne_points`）

验证：

```bash
ros2 topic info /velodyne_points   # 应看到 laserMapping 订阅
ros2 topic info /livox/lidar       # 只应有 point_cloud_converter 订阅，不应有 laserMapping
```

### 6.4 Point-LIO 里程计发散 !!!!

1. 确认小车**初始化**完成（启动后先静止几秒钟，等 LIO 收敛，再发探索目标 TARE 程序）
2. 检查 `/velodyne_points` 与 `/livox/imu` 有数据：`ros2 topic hz /velodyne_points /livox/imu`
3. 确认用的是 `mid360_sim.yaml`（仿真）而非 `mid360.yaml`（实机）
4. 设置配置文件 `Point-LIO` 中的降采样参数 `filter_size_surf` 和 `filter_size_map` 为 `0.5`

### 6.5 无可视化数据

1. 确认所有数据帧为 `odom`（TARE 源码 `kWorldFrameID` 已改为 `odom`）
2. 确认 RViz Fixed Frame 为 `odom`
3. 检查话题频率：`ros2 topic hz /overall_map /way_point`

### 6.6 首轮规划慢

TARE 首次完整规划周期（`UpdateGlobalRepresentation` + TSP）约需 5 秒，期间主线程阻塞，属正常现象，后续迭代更快。

### 6.7 查看调试输出

`execute()` 前 5 次调用带 `[DEBUG]` 日志，可通过 TARE 窗口观察初始化与 keypose 更新状态。

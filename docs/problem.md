# 全部问题与解决方案记录（problem.md）

> 记录从编译到仿真导航全流程遇到的问题、根因分析和修复方案。
> 环境：Ubuntu 22.04 + ROS 2 Humble + Docker (`osrf/ros:humble-desktop-full`)。
> 相关文件：`Dockerfile`、`scripts/build.sh`、`scripts/*_docker.sh`、`docs/docker_build.md`。

---

## 一、编译阶段

### P1：`colcon build` 报 `ModuleNotFoundError: No module named 'ament_package'`

**现象**
```text
from ament_package.templates import get_environment_hook_template_path
ModuleNotFoundError: No module named 'ament_package'
Failed   <<< gld_robot_description [0.35s, exited with code 1]
```

**原因**：没 `source /opt/ros/humble/setup.bash`

**解决**：所有 `colcon build` 前必须 `source /opt/ros/humble/setup.bash`

---

### P2：`kiss_matcher_ros` FetchContent 从 GitHub 拉取 small_gicp 失败

**现象**：`GnuTLS recv error (-110): The TLS connection was non-properly terminated`

**原因**：CMakeLists 无条件 `FetchContent` 从 GitHub 克隆 small_gicp，国内网络不稳

**解决**：修改 `src/registration/KISS-Matcher/ros/CMakeLists.txt`，优先 `find_package` 用系统安装版（镜像里已装），找不到才回退 FetchContent

---

### P3：`kiss_matcher_ros` FetchContent 从 GitHub 拉取 oneTBB 失败

**原因**：KISS-Matcher 核心的 `USE_SYSTEM_TBB` 默认 OFF，又去 GitHub 下载。镜像 `make cppinstall` 已装好 oneTBB

**解决**：`colcon build` 加 `-DUSE_SYSTEM_TBB=ON`，已写入 `scripts/build.sh`

---

### P4：`global_relocalization` FetchContent 拉取 small_gicp 失败

**原因**：同 P2，CMakeLists 无条件 FetchContent

**解决**：修改 `src/registration/global_relocalization/CMakeLists.txt`，同 P2 逻辑

---

### P5：`small_gicp_relocalization` / `global_small_gicp_relocalization` 的 `find_package(small_gicp)` 静默失败

**原因**：系统 small_gicp 的 CMake config 有 `find_dependency(OpenMP)`，但镜像里没装 `libomp-dev`，导致 `find_package` 失败后回退到 FetchContent

**解决**：装 `libomp-dev`（已固化进 Dockerfile）

---

### P6：编译占满全部线程致宿主机卡死

**原因**：`colcon build` 默认并行编包 + 每包 `-j$(nproc)`，12 线程全占满

**解决**：三层限速

| 层级 | 手段 |
| --- | --- |
| 容器 | `docker run --cpus 4` |
| colcon | `--executor sequential` |
| make | `MAKEFLAGS='-j4'` |

---

### P7：宿主机删不掉 `build/` 产物

**原因**：容器以 root 运行，产物属 `root:root`

**解决**：借容器删 (`docker exec ... rm -rf`) 或编译后 `sudo chown -R $USER:$USER build install log`

---

### P8：`rtabmap_ros` / `3d_bbs` 依赖缺失

**原因**：前者需同版本 librtabmap（源码 0.22.1），后者需 CUDA + Iridescence。and 启动脚本不依赖它们

**解决**：放置 `COLCON_IGNORE` 跳过编译

---

## 二、运行阶段

### P9：`gnome-terminal` 不存在

**原因**：容器无桌面环境

**解决**：写 docker 版脚本（`mapping_sim_docker.sh` / `nav2_sim_docker.sh`），用 tmux 窗口替代 gnome-terminal

---

### P10：容器没有 X11，Gazebo/RViz 打不开

**解决**：运行容器加 `-e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri:/dev/dri`，宿主机 `xhost +local:docker`

---

### P11：256 个僵尸进程致 DDS 通信瘫痪

**现象**：重跑多次脚本后 ROS 2 节点发现失效，`ros2 node list` 看不到实际进程

**原因**：每次跑 `mapping_sim_docker.sh` 启动新 tmux 窗口和新节点但不先停旧的，进程累积。旧会话产生的子进程不被 PID 1 (`sleep infinity`) 回收 → 僵尸

**解决**：
- 容器加 `docker run --init`（tini 作为 PID 1 自动回收）
- 每次重跑前 `docker exec lio_nav2 tmux kill-server`
- 使用 `scripts/docker_shutdown.sh` 统一关闭

---

### P12：Point-LIO 在仿真下不工作

**现象**：数据输入正常（`ptr->size=2804`），但 `buffer=0`，不发布 `/cloud_registered` 和 `camera_init` TF，RViz 空白

**原因**：
- `lidar_type: 2` (VELO16) 的 `N_SCANS=6` 与仿真 50 线不匹配，85% 点被过滤
- IMU 初始化后 reset 清空队列，`sync_packages()` 找不到数据不处理
- RViz `Fixed Frame: camera_init` 只在 LIO 初始化成功后才发布

**解决**：仿真用 FAST-LIO（从 Gazebo 直接订阅 PointCloud2），Point-LIO 保留为实机用

---

### P13：`save_pcd.sh` / `save_map.sh` 不工作

**现象**：找不到 PCD 文件

**原因**：
- `source /opt/ros/humble/setup.sh`（旧文件名）
- 节点名 `/fastlio_mapping` 实际是 `/laser_mapping`
- `ros2 param set map_file_path` 运行时无效（FAST-LIO 只启动时读一次）
- 保存的 PCD 只写入 `src/`，launch 文件在 `install/` 下找

**解决**：
- `source install/setup.bash`
- 不再依赖 `ros2 param set`，改为调用 service 后把默认路径的 PCD `mv` 到目标
- 保存后自动 `ln -sf` 到 `install/me_nav2_bringup/share/...` 目录

---

### P14：`map_server` 僵死致 `/map` 话题无数据

**现象**：Nav2 终端显示加载了地图文件，但 RViz 不显示

**原因**：
1. `my_nav2_launch.py` 把几百行的 `nav2_params.yaml` 传给了 `map_server`，YAML 中有冲突参数导致 lifecycle configure 失败 → 进程变成 `<defunct>` 僵尸
2. Python launch 文件改后没重编 `me_nav2_bringup`，`install/` 里的副本还是旧的（`--symlink-install` 只对编译产物做链接）

**解决**：
- `my_nav2_launch.py` 中 map_server 的 `parameters` 去掉 `params_file`，只传 `[{yaml_filename}, {use_sim_time}]`
- 改 Python 文件后重编：`colcon build --packages-select me_nav2_bringup`

---

### P15：Nav2 GoalTool 插件加载失败

**现象**：`Failed to load library /opt/ros/humble/lib/libnav2_rviz_plugins.so: libdiagnostic_updater.so: cannot open shared object file`

**原因**：缺 `ros-humble-diagnostic-updater`

**解决**：`apt install ros-humble-diagnostic-updater`（已固化进 Dockerfile）

---

### P16：有绿线路径但小车不动，`/cmd_vel` 全是 0

**现象**：
- 全局规划器生成了绿线路径
- `controller_server` 每秒打 log `Passing new path to controller`
- `ros2 topic echo /cmd_vel` 全是 0
- 最终 `Goal failed`

**原因**：DWB 控制器默认 `trans_stopped_velocity: 0.25`。当前车速为 0（低于 0.25），DWB 认为机器人"已停止"，trajectory generator 采样不到有效速度。配合 `max_vel_x: 0.26`（几乎等于 stopped 阈值）和 `failure_tolerance: 0.3`（300ms 超时），控制器永远发不出速度指令。

**解决**：在 `nav2_params.yaml` 的 `FollowPath` 段显式设低停止阈值：

```yaml
trans_stopped_velocity: 0.01
rot_stopped_velocity: 0.01
min_vel_x: 0.0        # 滑移转向不快倒车
```

---

### P17：PCD 文件只在 `src/` 不在 `install/`

**原因**：`save_pcd.sh` 保存到 `src/`，但 KISS 重定位的 launch 用 `get_package_share_directory()` 找 `install/` 下的路径

**解决**：`save_pcd.sh` / `save_map.sh` 保存后自动 `ln -sf` 到 `install/` 目录

---

### P18：`airy_unflip.py` 修正后点云与位姿 XY 方向不匹配

**现象**：Airy 实机运行方案一（`airy_unflip.py`）后，2D SLAM 建图和导航的 costmap
中障碍物位置偏移，与机器人实际朝向不一致。

**原因**：Airy 的 `extrinsic_R` 同时做了 **Z 翻转** 和 **X/Y 交换**：

```yaml
extrinsic_R: [0, -1, 0,
              -1, 0, 0,
              0, 0, -1]
```

`airy_unflip.py` 对整个矩阵取了逆（`R_inv = R_ext^T`），连 X/Y 一起改了：

```python
pt_corrected_b = R_inv @ pt_b    # ← 当前实现：XY 也被旋转了
```

以 LiDAR 正前方一点 `[1, 0, 0]` 为例：

| 阶段 | world 帧坐标 | 指向 |
| --- | --- | --- |
| 原始（Z 翻转后） | `R_imu→world × [0, -1, 0] + T` | 左侧 |
| unflip 后 | `R_imu→world × [1, 0, 0] + T` | **前方** |

同一个物理墙壁点，unflip 前后在 world 帧下**差了 90°**。但 odometry 姿态 `R_imu→world`
**没有被修正**，机器人朝向（TF 链 `odom → base_footprint`）仍是旧的。
结果是点云中障碍物的方向与机器人的方向在世界帧下不一致。

**后果链**：
```
点云 XY 旋转了   ≠   TF 链的机器人朝向
        ↓
  /scan 数据角度偏移
        ↓
  costmap 障碍物与机器人真实环境错位
        ↓
  Nav2 路径规划错误 / 碰撞
```

**正确修复方案**：只翻转 Z 轴，不动 XY——对外参矩阵取第一行和第三行的反，第二行保持不变：

```python
# 正确：只修正 Z，不动 XY
pt_corrected_b = pt_b.copy()
pt_corrected_b[2] = -pt_b[2]

# 当前错误实现（XY 也被改了）：
# pt_corrected_b = R_ext.T @ pt_b
```

**为什么方案二（翻转高度阈值）不受此影响**：方案二不改点云本身，只在 `pointcloud_to_laserscan`
中把 `min_height/max_height` 设为负值。点云和位姿始终在同一坐标系下，XY 天然一致。

---

## 三、编译/运行命令速查

```bash
# ====== 构建镜像 ======
docker build -t lio_nav2:humble .

# ====== 编译 ======
docker run -d --name lio_nav2_build --cpus 4 --network host -v $PWD:/ws lio_nav2:humble sleep infinity
docker exec lio_nav2_build bash -c "rosdep update && cd /ws && rosdep install --from-paths src --ignore-src -r -y"
docker exec lio_nav2_build bash -c "
  source /opt/ros/humble/setup.bash && cd /ws &&
  MAKEFLAGS='-j4' colcon build --symlink-install --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_TBB=ON
"

# ====== 运行容器 ======
xhost +local:docker
docker run -d --name lio_nav2 --init --network host --ipc host --cpus 8 \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri:/dev/dri \
  -v $PWD:/ws lio_nav2:humble-dev sleep infinity

# ====== 仿真建图 ======
docker exec -it lio_nav2 /ws/scripts/mapping_sim_docker.sh

# ====== 保存地图 ======
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && /ws/scripts/save_map.sh"
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && /ws/scripts/save_pcd.sh"

# ====== 仿真导航 ======
docker exec lio_nav2 tmux kill-server 2>/dev/null    # 先停建图
docker exec -it lio_nav2 /ws/scripts/nav2_sim_docker.sh

# ====== 关闭 ======
./scripts/docker_shutdown.sh          # 清理节点，容器保留
./scripts/docker_shutdown.sh --stop   # 停止容器
```

## 四、改进文件清单

| 文件 | 改动 |
| --- | --- |
| `Dockerfile` | 新增，固化编译/运行依赖 |
| `.dockerignore` | 新增，白名单模式缩小 build context |
| `scripts/build.sh` | 加 `-DUSE_SYSTEM_TBB=ON`，注释说明 |
| `scripts/mapping_sim_docker.sh` | 新增，tmux 版建图脚本 |
| `scripts/nav2_sim_docker.sh` | 新增，tmux 版导航脚本 |
| `scripts/docker_shutdown.sh` | 新增，容器内节点/容器统一关闭 |
| `scripts/save_pcd.sh` | 修节点名、source 路径、PCD 同步到 install |
| `scripts/save_map.sh` | 修 source 路径、地图同步到 install |
| `src/me_nav2_bringup/launch/my_nav2_launch.py` | map_server 去 params_file，加独立 RViz |
| `src/me_nav2_bringup/config/nav2_params.yaml` | DWB `trans_stopped_velocity` 等修复 |
| `src/registration/KISS-Matcher/ros/CMakeLists.txt` | small_gicp 优先系统版 |
| `src/registration/global_relocalization/CMakeLists.txt` | 同上 |
| `src/localization/rtabmap_ros/COLCON_IGNORE` | 跳过 |
| `src/registration/3d_bbs/COLCON_IGNORE` | 跳过 |
| `docs/docker_build.md` | Docker 编译运行详细指南 |

## 五、已知待解决问题

1. **Point-LIO 仿真适配**：需修改源码中 `N_SCANS` 匹配 50 线仿真 LiDAR，或改用 `lidar_type` 适配
2. **NVIDIA GPU 加速**：当前走 Mesa 软件渲染，装 nvidia-container-toolkit 后可 `--gpus all`
3. **`navigation_launch.py` 的 lifecycle_manager 未自动激活**：需排查 nav2_bringup 中 `autostart` 参数传递
4. **`airy_unflip.py` XY 方向偏移**（P18）：`R_ext.T @ pt` 连 X/Y 一起旋转了，应改为只翻转 Z 轴

---

## 七、TARE Planner ROS 2 编译与 Gazebo 仿真管线（P20）

### 背景

TARE (Technically-Aware Robotic Exploration) 是一个自主探索规划器，让机器人在未知环境中自主决定探索路径。

需要将 TARE 从 ROS 1 (melodic-noetic) 迁移到 ROS 2 Humble 并适配 Gazebo + FAST-LIO。

### 编译

官方仓库 `caochao39/tare_planner` 已有 `humble-jazzy` 分支（已迁移到 ROS 2），直接使用：

```bash
docker exec lio_nav2 bash -c "
  cd /ws/src/planner/tare_planner &&
  git checkout humble-jazzy &&
  cd /ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select tare_planner --symlink-install
"
```

### Humble 兼容性修复

Jazzy 分支中 `explore.launch` 使用了 `SetParameter`（Jazzy 新增），Humble 不支持：

```python
# 删除这行
from launch.actions import SetParameter
# 删除这行
SetParameter(name='use_sim_time', value=use_sim_time),
```

### 仿真管线

**架构**：Gazebo → FAST-LIO (里程计+点云) → lio_interface → sensor_scan_generation → cloud_z_filter → **TARE Planner** → waypoint_follower → `/cmd_vel`

**TARE 话题适配**：

| TARE 输入 | 来源 | 话题 |
|-----------|------|------|
| 里程计 | lio_interface | `/odom` |
| 点云 | cloud_z_filter | `/registered_scan_filtered` |
| 起始信号 | auto (kAutoStart=true) | `/start_exploration` (不发布) |

**TARE 输出**：

| 话题 | 类型 | 含义 |
|------|------|------|
| `/way_point` | geometry_msgs/PointStamped | 下一个探索目标点 |
| exploration_finish | std_msgs/Bool | 探索完成标志 |

**Waypoint Follower**：新增 `scripts/waypoint_follower.py`，订阅 `/way_point` → PID 控制器 → `/cmd_vel`。

### 启动

```bash
docker exec -it lio_nav2 bash
cd /ws && bash scripts/gazebo_tare.sh
```

tmux 窗口：`Gazebo | FAST-LIO | lio_if | sensor | TARE | RViz`

### 新增文件

| 文件 | 说明 |
|------|------|
| `src/planner/tare_planner/src/tare_planner/config/gazebo_indoor.yaml` | Gazebo 仿真参数 |
| `src/planner/tare_planner/src/tare_planner/launch/tare_planner_lio_launch.py` | ROS 2 Launch 文件 |
| `src/planner/tare_planner/src/tare_planner/scripts/waypoint_follower.py` | PID 航点跟随器 |
| `scripts/gazebo_tare.sh` | 一键启动脚本 |

### 调试过程中的关键问题与修复

#### 1. QoS 不匹配（已修复）

`cloud_z_filter.py` 用 `qos_profile_sensor_data`（BEST_EFFORT）发布，但 TARE subscriber 默认 RELIABLE，导致点云无法传递。

#### 2. 点云帧错误（关键根因）

`cloud_z_filter` 对 `camera_init` 帧（LiDAR 帧）的 `/cloud_registered` 做 Z 轴过滤，`camera_init` 帧的 Z 轴与世界 Z 轴不一致，导致所有点被滤除（输出 width=0）。

**修复**：TARE 直接订阅 `sensor_scan_generation` 发布的 `/registered_scan`（已转换到 `odom` 帧），去掉 `cloud_z_filter` 节点。

#### 3. Rolling Occupancy Grid 过大（性能瓶颈）

`rolling_occupancy_grid` 范围 = `kPointCloudCellSize × kPointCloudManagerNeighborCellNum`。默认 18×5=90m，在 0.2m 分辨率下是 450×450×45 = 910 万网格，3D RayTrace 极慢，主线程被点云回调占满，`execute()` 定时器长时间延迟。

**修复**：
```yaml
kPointCloudCellSize: 18.0 → 10.0
kPointCloudManagerNeighborCellNum: 5 → 3
rolling_occupancy_grid/resolution: 0.2 → 0.3
```
网格从 910 万降到 15 万，速度提升 60 倍。

#### 4. 坐标系帧不匹配（可视化空白根因）

TARE 所有可视化数据（点云、路径、标记）都发布在 `map` 帧（`kWorldFrameID = "map"`），但 FAST-LIO 管线的 TF 树只有 `world → odom`，没有 `map` 帧。RViz 用 `odom` 作 Fixed Frame，看不到 `map` 帧数据。

**修复**：
- `sensor_coverage_planner_ground.h`: `kWorldFrameID = "map"` → `"odom"`
- `tare_visualizer.h`: 同上
- `planning_env.h`: 默认帧参数 `"map"` → `"odom"`
- `sensor_coverage_planner_ground.cpp`: 所有 `header.frame_id = "map"` → `"odom"`
- RViz 配置 Fixed Frame: `map` → `odom`

#### 5. 首轮全局规划耗时 5 秒

TARE 首次完整规划周期（`UpdateGlobalRepresentation` + TSP）约需 5 秒，期间主线程阻塞，后续定时器延迟。这是正常现象，后续迭代更快。

#### 6. 机器人撞障碍物（waypoint 延长 + follower 无避障）

**现象**：探索时机器人直线撞上障碍物。

**根因 1**：`PublishWaypoint()` 中当 lookahead 点在视线内时，waypoint 被延长到 `kExtendWayPointDistanceBig`（默认 6~8m），但延长后的点**没有碰撞检测**，可能穿过障碍物：

```cpp
// sensor_coverage_planner_ground.cpp - PublishWaypoint()
if (r < extend_dist && kExtendWayPoint) {
    dx = dx / r * extend_dist;  // 直接延长, 无碰撞检查
    dy = dy / r * extend_dist;
}
```

**根因 2**：`waypoint_follower.py` 是纯 PID 直线跟随，无任何避障。

**修复**：
1. `kExtendWayPoint: true → false`（waypoint 停在经过 LOS 检测的 lookahead 点）
2. 重写 `waypoint_follower.py`，订阅 `/registered_scan` 点云做前方局部避障：
   - 将点云变换到 body 帧，分左/中/右三扇区检测最小障碍距离
   - 正前方 < 0.45m：原地转向远离障碍物
   - 前方 < 0.8m：减速 + 转向偏往开阔侧

**根因 3（地面点误判）**：避障检测时地面点（z≈0）落在高度检查范围 [-0.15, 0.6] 内，被误判为障碍物，导致机器人停滞。

**修复**：`check_height_min: -0.15 → 0.05`（排除地面点，只检测机器人底盘以上的障碍物）。

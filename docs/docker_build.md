# Docker 编译与运行指南（docker_build.md）

> 使用 Docker（`osrf/ros:humble-desktop-full` 基础镜像）编译和运行本工作空间。
> 环境：Ubuntu 22.04 宿主机（12 核 / 16G 内存），Docker 29.x，工作空间挂载到容器 `/ws`。
> 相关文件：仓库根目录 `Dockerfile`、`.dockerignore`、`scripts/build.sh`。

---

## 1. 构建镜像

```bash
cd <工作空间根目录>
docker build -t lio_nav2:humble .
```

镜像包含：ROS 2 Humble + Gazebo Fortress + Nav2 + SLAM Toolbox + Cartographer + small_gicp + KISS-Matcher C++ 库（含 oneTBB/ROBIN）+ rosdep 依赖。

**预装的 ROS 包**（`Dockerfile` 中 `apt-get install`）：

| 类别 | 包名 |
|------|------|
| 导航 | `ros-humble-navigation2` `ros-humble-nav2-bringup` |
| SLAM（传统） | `ros-humble-slam-toolbox` |
| **SLAM（Cartographer）** | **`ros-humble-cartographer` `ros-humble-cartographer-ros`** |
| OctoMap | `ros-humble-octomap-server` `ros-humble-octomap-rviz-plugins` |
| 传感器/点云 | `ros-humble-pointcloud-to-laserscan` `ros-humble-pcl-ros` `ros-humble-pcl-conversions` |
| Gazebo | `ros-humble-gazebo-ros-pkgs` |
| 通信 | `ros-humble-rmw-cyclonedds-cpp` |
| 诊断 | `ros-humble-diagnostic-updater` |
| 数学/优化 | `ros-humble-gtsam` `ros-humble-tf-transformations` `libeigen3-dev` `libglm-dev` |
| 工具 | `ros-humble-tf2-tools` `ros-humble-ament-cmake-clang-format` |
| 容器内 | `tmux` `psmisc` |

---

## 2. 编译工作空间

### 2.1 启动编译容器（限 4 核）

```bash
docker run -d --name lio_nav2_build --cpus 4 --network host \
  -v $PWD:/ws lio_nav2:humble sleep infinity
```

### 2.2 rosdep 兜底安装（正常几秒）

```bash
docker exec lio_nav2_build bash -c \
  "rosdep update && cd /ws && rosdep install --from-paths src --ignore-src --rosdistro humble -r -y"
```

### 2.3 限速编译（单包串行 + 每包 4 线程）

```bash
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ws &&
  MAKEFLAGS='-j4' colcon build --symlink-install --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DUSE_SYSTEM_TBB=ON
"
```

> **`source /opt/ros/humble/setup.bash` 是必须的**——不 source 的话 Python 找不到
> `ament_package` 等 ROS 2 模块，CMake configure 阶段会报
> `ModuleNotFoundError: No module named 'ament_package'`。
> 已编译过的包会自动跳过（增量续编），只重新编译修改过的文件。

最终结果：**20 个包全部编译通过**（`Summary: 20 packages finished [11min 44s]`）。

---

## 3. 运行

### 3.1 启动运行容器（X11 + GPU）

```bash
xhost +local:docker        # 每次开机执行一次

docker run -d --name lio_nav2 \
  --network host --ipc host --cpus 8 \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri:/dev/dri \
  -v $PWD:/ws \
  lio_nav2:humble-dev sleep infinity
```

### 3.2 仿真建图

项目支持两种 SLAM 后端：**SLAM Toolbox**（原有）和 **Cartographer**（新增）。

#### 3.2.1 SLAM Toolbox 建图（原有方案）

```bash
docker exec -it lio_nav2 /ws/scripts/mapping_sim_docker.sh   # 启动全部节点
docker exec -it lio_nav2 tmux attach -t mapping_sim          # 查看输出
docker exec lio_nav2 tmux kill-session -t mapping_sim        # 停止
```

tmux 窗口：`GUI控制` `FAST-LIO` `lio_interface` `Gazebo` `sensor_scan` `pc2laser` `slam_toolbox`

#### 3.2.2 Cartographer 建图（新方案，推荐）

Cartographer 相比 SLAM Toolbox 的优势：
- 自带回环检测（图优化），大范围建图漂移更小
- 建图和定位使用同一框架，保存的 `.pbstream` 可直接用于纯定位模式
- 发布 `/map`（OccupancyGrid）+ `map→odom` TF，与 Nav2 无缝对接

```bash
docker exec -it lio_nav2 /ws/scripts/mapping_sim_carto_docker.sh   # 启动全部节点
docker exec -it lio_nav2 tmux attach -t mapping_carto              # 查看输出
docker exec lio_nav2 tmux kill-session -t mapping_carto            # 停止
```

tmux 窗口：`GUI控制` `FAST-LIO` `lio_interface` `Gazebo` `sensor_scan` `pc2laser` `Cartographer` `RViz`

Cartographer 节点数据流：

```
FAST-LIO → /Odometry → lio_interface (TF 桥接)
         → /cloud_registered → sensor_scan_generation → /registered_scan
              → pointcloud_to_laserscan → /scan ──────────────┐
/livox/imu ────────────────────────────────────────────────────┤
/odom (sensor_scan_generation) ─────────────────────────────────┤
                                                                  ↓
                                                       cartographer_node
                                                     (Cartographer 2D SLAM)
                                                                  ↓
                                                    /map + map→odom TF
```

Cartographer 配置文件：
- `src/me_nav2_bringup/config/cartographer_mapping.lua` — 建图模式参数
- `src/me_nav2_bringup/launch/cartographer_mapping_launch.py` — 启动文件
- `src/me_nav2_bringup/rviz/cartographer_mapping.rviz` — RViz 可视化配置

**RViz 可视化**：Cartographer 建图时 RViz 窗口会自动弹出（tmux 中的 `RViz` 窗口），显示：
| 显示项 | 话题 | 说明 |
|--------|------|------|
| CartographerMap | `/map` | 实时 OccupancyGrid 建图结果 |
| LaserScan | `/scan` | 2D 激光扫描点（红色） |
| RegisteredScan | `/registered_scan` | 3D 注册点云（彩色） |
| TF | 坐标树 | `map→odom→base_footprint→chassis→livox_frame` |
| RobotModel | `/robot_description` | 机器人 3D 模型 |

如果 Gazebo/RViz 报 GLX 错误，容器内改用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

#### 3.2.3 Cartographer 地图保存

建图完成后（控制小车在环境中完整走一圈），执行以下步骤保存地图：

```bash
# 步骤 1: 结束 Cartographer 轨迹（停止接收新数据，完成最后的全局优化）
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \
    '{trajectory_id: 0}'"

# 步骤 2: 导出 .pbstream → .pgm + .yaml
#   .pbstream: Cartographer 内部格式（含子图 + 位姿图），纯定位时需要
#   .pgm + .yaml: 静态占据栅格地图，Nav2 map_server 需要
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename /ws/src/me_nav2_bringup/map/map.pbstream \
    -map_filestem /ws/src/me_nav2_bringup/map/my_map"
```

输出文件：
| 文件 | 用途 |
|------|------|
| `map.pbstream` | Cartographer 纯定位模式加载 |
| `my_map.pgm` | Nav2 map_server 加载 |
| `my_map.yaml` | PGM 元数据（分辨率、原点） |

> 也可以用 `ros2 run nav2_map_server map_saver_cli -f <地图名>` 直接保存 `/map` 话题，
> 但这样只能得到 `.pgm`，丢失了 Cartographer 的子图信息，纯定位效果会打折。

### 3.3 仿真导航

#### 3.3.1 SLAM Toolbox + KISS-Matcher 导航（原有方案）

```bash
docker exec -it lio_nav2 /ws/scripts/nav2_sim_docker.sh      # 启动全部节点
docker exec -it lio_nav2 tmux attach -t nav2_sim              # 查看输出
docker exec lio_nav2 tmux kill-session -t nav2_sim            # 停止
```

tmux 窗口：`GUI控制` `FAST-LIO` `lio_interface` `Gazebo` `sensor_scan` `pc2laser` `KISS+GICP` `Nav2`

#### 3.3.2 Cartographer 纯定位导航（新方案）

Cartographer 纯定位模式加载已建好的 `.pbstream` 地图，实时匹配当前 `/scan` 数据，
发布 `map→odom` TF，替代原来的 KISS-Matcher 重定位。

**前提**：已完成 Cartographer 建图并保存了 `.pbstream` 和 `.pgm` 文件。

```bash
docker exec -it lio_nav2 /ws/scripts/nav2_sim_carto_docker.sh   # 启动全部节点
docker exec -it lio_nav2 tmux attach -t nav2_carto               # 查看输出
docker exec lio_nav2 tmux kill-session -t nav2_carto             # 停止
```

tmux 窗口：`GUI控制` `FAST-LIO` `lio_interface` `Gazebo` `sensor_scan` `pc2laser` `Carto定位` `Nav2`

架构对比：

```
原有方案:  KISS-Matcher (加载 .pcd 先验地图) ──→ map→odom TF ──→ Nav2
Cartographer: Cartographer 纯定位 (加载 .pbstream) ──→ map→odom TF ──→ Nav2
```

> **注意**：`map→odom` TF 同一时间只能有一个发布者。用了 Cartographer 纯定位就不需要 KISS-Matcher。
> `nav2_sim_carto_docker.sh` 中没有 KISS-Matcher 窗口。

**使用步骤**：

1. 等 `Carto定位` 窗口打印定位成功后
2. 在 RViz 中用 **"2D Pose Estimate"** 工具（工具栏绿色箭头）在机器人实际位置点一下，拖动方向
3. Cartographer 收到初始位姿后开始实时匹配，`map→odom` TF 开始发布
4. 用 **"Nav2 Goal"** 工具发送导航目标

配置文件：
- `src/me_nav2_bringup/config/cartographer_localization.lua` — 纯定位模式参数
  - `TRAJECTORY_BUILDER.pure_localization = true` — 不建图，只匹配
  - 搜索窗口比建图模式更大（`linear_search_window = 0.2`），提升鲁棒性
- `src/me_nav2_bringup/launch/cartographer_localization_launch.py` — 启动文件

### 3.4 关闭 / 清理

```bash
./scripts/docker_shutdown.sh            # 清理容器内所有节点，容器保留
./scripts/docker_shutdown.sh --stop     # 清理节点 + 停止容器
./scripts/docker_shutdown.sh --rm       # 清理节点 + 删除容器
./scripts/docker_shutdown.sh --dry-run  # 预览，不实际操作
```

> `docker_shutdown.sh` 已配置本项目所有 tmux 会话：`mapping_sim` `nav2_sim` `mapping_real` `nav2_real` `mapping_carto` `nav2_carto` `octomap_sim`

### 3.5 进入容器交互式操作

```bash
docker exec -it lio_nav2 bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 然后可以 ros2 topic list、ros2 launch 等
```

---

## 4. 关键配置说明

### 4.1 三层限速（避免拖垮宿主机）

| 层级 | 手段 | 作用 |
| --- | --- | --- |
| 容器 | `docker run --cpus 4` | 硬上限：容器最多用 4 个核 |
| colcon | `--executor sequential` | 同一时间只编 1 个包 |
| make | `MAKEFLAGS='-j4'` | 单个包内部最多 4 线程 |

> 运行容器用 `--cpus 8`（仿真需要更多算力），编译容器用 `--cpus 4`。

### 4.2 资源消耗参考

| 场景 | CPU 限制 | 内存占用 | 说明 |
| --- | --- | --- | --- |
| 编译 | `--cpus 4`, sequential | ~1-2 GB | 全量编译约 12 分钟 |
| 仿真建图 (SLAM Toolbox) | `--cpus 8` | ~3-5 GB | Gazebo + FAST-LIO + SLAM + Nav2 |
| 仿真建图 (Cartographer) | `--cpus 8` | ~4-6 GB | Gazebo + FAST-LIO + Cartographer（图优化更耗内存） |
| 仿真导航 (Cartographer 纯定位) | `--cpus 8` | ~3-5 GB | 纯定位比建图更省资源 |

### 4.3 GPU/渲染

本机 NVIDIA 独显但未装 nvidia-container-toolkit，通过 `--device /dev/dri`（mesa）渲染。
如果 Gazebo/RViz 报 GLX 错误，容器内改用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

安装 nvidia-container-toolkit 后可加 `--gpus all` 启用硬件加速。

### 4.4 编译产物文件属主

容器以 root 运行，生成的 `build/ install/ log/` 在宿主机上属 `root:root`：

```bash
# 清理时借容器内 root 来删
docker exec lio_nav2 rm -rf /ws/build/<pkg>

# 或编译完成后把产物归还当前用户
sudo chown -R $USER:$USER build install log
```

### 4.5 镜像标签说明

| 标签 | 用途 |
| --- | --- |
| `lio_nav2:humble` | Dockerfile 构建的纯编译镜像 |
| `lio_nav2:humble-dev` | 在编译容器上 `docker commit`，额外含 rosdep 补装依赖和 tmux |

两者内容等价；`humble-dev` 可立即用于运行仿真。

---

## 5. 脚本速查表

| 脚本 | tmux 会话 | 模式 | 说明 |
|------|-----------|------|------|
| `mapping_sim_docker.sh` | `mapping_sim` | SLAM Toolbox 建图 | 原有方案 |
| **`mapping_sim_carto_docker.sh`** | **`mapping_carto`** | **Cartographer 建图** | **新增，推荐** |
| `mapping_sim_superlio_docker.sh` | — | Super-LIO 建图 | Super-LIO 里程计 |
| `octomap_sim_docker.sh` | `octomap_sim` | OctoMap 3D 建图 | 八叉树地图 |
| `nav2_sim_docker.sh` | `nav2_sim` | KISS-Matcher + Nav2 | 原有导航方案 |
| **`nav2_sim_carto_docker.sh`** | **`nav2_carto`** | **Cartographer 纯定位 + Nav2** | **新增** |
| `docker_shutdown.sh` | — | 关闭/清理 | 杀掉所有 tmux 会话 + ROS 节点 |

---

## 6. 排错：常见问题与解决

### Q1：`colcon build` 报 `ModuleNotFoundError: No module named 'ament_package'`

**现象**

```text
Traceback (most recent call last):
  File "/opt/ros/humble/share/ament_cmake_core/cmake/package_templates/templates_2_cmake.py", line 21, in <module>
    from ament_package.templates import get_environment_hook_template_path
ModuleNotFoundError: No module named 'ament_package'
CMake Error at .../ament_cmake_package_templates-extras.cmake:41 (message):
  ... returned error code 1
Failed   <<< gld_robot_description [0.35s, exited with code 1]
```

**原因**
没有 `source /opt/ros/humble/setup.bash`。ROS 2 的环境脚本会把 Python site-packages
路径加入 `PYTHONPATH`，不 source 的话 Python 找不到 `ament_package`。

**解决**
在 `colcon build` 之前**必须**加 `source /opt/ros/humble/setup.bash`：

```bash
source /opt/ros/humble/setup.bash && colcon build ...
```

---

### Q2：`kiss_matcher_ros` 配置阶段从 GitHub 拉取 small_gicp 失败

**现象**

```text
fatal: unable to access 'https://github.com/koide3/small_gicp/':
GnuTLS recv error (-110): The TLS connection was non-properly terminated.
CMake Error at .../FetchContent.cmake:1087 (message): ...
Failed   <<< kiss_matcher_ros [exited with code 1]
```

**原因**
`src/registration/KISS-Matcher/ros/CMakeLists.txt` 无条件使用 `FetchContent` 从 GitHub 拉取
`small_gicp` master，国内网络不稳定。而镜像里其实已通过源码安装了 small_gicp。

**解决**
已修改 `ros/CMakeLists.txt`：优先 `find_package` 系统安装版，找不到才回退 FetchContent：

```cmake
find_package(small_gicp QUIET)
if(small_gicp_FOUND)
  message(STATUS "Using system-installed small_gicp")
  add_library(small_gicp ALIAS small_gicp::small_gicp)
else()
  include(FetchContent)
  FetchContent_Declare(small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp
    GIT_TAG master)
  FetchContent_MakeAvailable(small_gicp)
endif()
```

> 修 CMakeLists 后要删掉该包的 build 目录再重编：`docker exec lio_nav2_build rm -rf /ws/build/kiss_matcher_ros`

---

### Q3：`kiss_matcher_ros` 又从 GitHub 拉取 oneTBB v2022.0.0 失败

**现象**

```text
error: downloading 'https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2022.0.0.tar.gz' failed
OpenSSL SSL_read: error:0A000126:SSL routines::unexpected eof while reading
```

**原因**
`ros/CMakeLists.txt` 通过 `add_subdirectory` 重新编译 KISS-Matcher 核心包，该核心
默认 `USE_SYSTEM_TBB=OFF`，又去 GitHub 下载 oneTBB。
而镜像构建期 `make cppinstall` 已把 oneTBB v2022 装到了 `/usr/local/`。

**解决**
编译时加 `-DUSE_SYSTEM_TBB=ON`（已写入 `scripts/build.sh`）。其余包不认识该变量，
只会打印 `Manually-specified variables were not used` 警告，无害。

> KISS-Matcher 三方依赖（robin、TBB、small_gicp）全部可走「镜像构建期装好 + 编译期用系统版」路线，
> 从而做到 **colcon 编译全程不联网**。

---

### Q4：编译占满全部线程导致宿主机卡死

**现象**
默认 `colcon build` 并行编包（最多 CPU 核数个包同时编），每个包的 make 又各自 `-j12`，
宿主机前台程序卡死，甚至因内存吃紧（16G 里可用仅 ~9G）存在 OOM 风险。

**解决**
见 [4.1 三层限速](#41-三层限速避免拖垮宿主机)。

---

### Q5：宿主机上删不掉 `build/` 里的文件（Permission denied）

**原因**
osrf 镜像默认以 root 运行，挂载目录里生成的 `build/ install/ log/` 在宿主机上属 `root:root`。

**解决**
见 [4.4 编译产物文件属主](#44-编译产物文件属主)。

---

### Q6：`rtabmap_ros` 与 `3d_bbs` 无法在容器内直接编译（已跳过）

**原因**

- `src/localization/rtabmap_ros` 是 **0.22.1 源码**（14 个子包），需要同版本的
  librtabmap 库；apt 二进制版本不匹配，从源码编 librtabmap 需额外 30 分钟以上。
- `src/registration/3d_bbs/ros2_test/*` 依赖 `gpu_bbs3d`（需要 **CUDA**）和
  `Iridescence` 可视化库，镜像内均不具备。

同时，`scripts/` 下所有启动脚本**都没有用到这两组包**，不在 LIO + Nav2 主链路上。

**解决**
在两个目录放置 `COLCON_IGNORE` 空文件跳过编译（rosdep 也会随之跳过其依赖）：

```text
src/localization/rtabmap_ros/COLCON_IGNORE
src/registration/3d_bbs/COLCON_IGNORE
```

如需恢复编译：删除对应 `COLCON_IGNORE` 后，安装匹配版本 librtabmap / CUDA 运行时。

---

### Q7：镜像/容器被清理后如何恢复

**现象**
宿主机重启 + `docker image prune` 后镜像和容器都消失了。

**解决**
Docker BuildKit 缓存仍在时，`docker build -t lio_nav2:humble .` 全部层 `CACHED`，秒级完成。
工作空间的 `build/ install/` 是挂载在宿主机的，容器重建后 colcon 自动**增量续编**
（前提：挂载点仍是 `/ws`，路径变了缓存全部作废）。

---

### Q8：`gnome-terminal` 和 X11 问题

**现象**
`mapping_sim.sh` 等脚本用 `gnome-terminal` 多窗口启动，容器内没有桌面环境。

**解决**
- 使用容器版脚本（tmux 替代 gnome-terminal），见 [3.2 仿真建图](#32-仿真建图) 和 [3.3 仿真导航](#33-仿真导航)
- 运行容器挂载 X11 socket：`-e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri:/dev/dri`
- 宿主机执行 `xhost +local:docker`
- 关闭用 `scripts/docker_shutdown.sh`
- 详细说明见 [3. 运行](#3-运行)

---

### Q9：Point-LIO 仿真下 RViz 无数据显示

**现象**

Point-LIO 节点正常启动、`lidar_type: 2` 识别正确、Gazebo 也在发布 `/livox/lidar`，
但 `/cloud_registered` 始终无数据，`camera_init` TF 不存在，RViz 一片空白。

Point-LIO 终端输出：
```text
[proc2] surf=2804
[cbk] scan=1 lidar_type=2 ptr->size=2804 buffer=0
[cbk] scan=11 lidar_type=2 ptr->size=2804 buffer=0
```

**原因**

1. **N_SCANS 不匹配**：`lidar_type: 2`（VELO16 模式）的 `velodyne_handler` 中
   `N_SCANS` 默认为 6（Livox MID-360 的扫描线数），而仿真 Gazebo ray sensor 有 50 条
   扫描线，`ign_sim_pointcloud_tool` 产生的 ring 值从 0 到 38。
   代码 `ring < N_SCANS` 过滤掉 85% 的点，覆盖率太低导致 Point-LIO 初始化失败，
   不发布 `/cloud_registered` 和 `camera_init` TF。

2. **IMU 初始化后 reset**：`ImuProcess` 初始化达 100% 后执行 reset，清空 IMU 队列，
   主循环 `sync_packages()` 因 `imu_deque.empty()` 返回 false，不进入处理流程。

3. **RViz Fixed Frame 不存在**：Point-LIO 自带的 `pointlio_robosense.rviz` 使用
   `camera_init` 作为 Fixed Frame（Robosense 坐标系），该帧只在 Point-LIO 成功初始化
   后才发布，初始化失败时 RViz 无法渲染任何内容。

**解决**

推荐使用 **FAST-LIO** 作为仿真里程计（`mapping_sim_docker.sh` 已默认切回 FAST-LIO）：

```bash
# FAST-LIO 直接从 Gazebo 订阅 /livox/lidar (PointCloud2)，不需要 ign_sim_pointcloud_tool
new_win "FAST-LIO" "ros2 launch fast_lio mapping.launch.py"
new_win "lio_interface" "ros2 launch lio_interface fastlio_lio_interface_launch.py"
```

如果必须使用 Point-LIO，需在源码层面修改：

- `src/localization/point_lio/src/preprocess.cpp`：将 `VELO16` 分支的 `N_SCANS` 从 6 改为
  与仿真扫描线数一致的值（或改用 `MAX_LINE_NUM`）
- 在 `point_lio.launch.py` 中设置 `rviz:=False` 避免 Point-LIO 自带 RViz 的
  `camera_init` 帧问题，改用 Nav2 的 RViz 查看数据

---

### Q10：Cartographer 建图时空地图不更新 / 飘移严重

**现象**
Cartographer 节点正常运行，`/map` 话题有数据，但地图不随机器人移动而更新，或地图漂移严重。

**可能原因与解决**

1. **TF 时间戳不同步**：检查所有节点 `use_sim_time` 是否一致（仿真全为 `true`）
   ```bash
   docker exec lio_nav2 bash -c "source install/setup.bash && ros2 param get /cartographer_node use_sim_time"
   ```

2. **LaserScan 帧错误**：Cartographer 配置中 `tracking_frame = "base_footprint"`，
   需要 TF 提供 `base_footprint→livox_frame` 的静态变换。检查 URDF 是否正常发布：
   ```bash
   docker exec lio_nav2 bash -c "source install/setup.bash && ros2 run tf2_tools view_frames"
   ```

3. **IMU 数据未到达**：Cartographer 建图配置中 `use_imu_data = true`，确保 `/livox/imu` 有数据：
   ```bash
   docker exec lio_nav2 bash -c "source install/setup.bash && ros2 topic hz /livox/imu"
   ```

4. **运动速度太快**：建图时建议低速平稳运动，Cartographer 需要足够的 scan 重叠来做子图匹配。

---

### Q11：Cartographer 纯定位无法初始化

**现象**
`nav2_sim_carto_docker.sh` 启动后 Cartographer 终端显示找不到当前位置，不发布 `map→odom` TF。

**解决**
1. 确认加载的 `.pbstream` 路径正确：
   ```bash
   docker exec lio_nav2 ls -la /ws/src/me_nav2_bringup/map/map.pbstream
   ```
2. 在 RViz 中用 **"2D Pose Estimate"** 工具给机器人一个近似初始位姿
3. 如果机器人位置和建图时差别很大（如已移动到地图另一个区域），让机器人原地缓慢旋转几圈，
   扩大 `TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window`
   的值（当前定位配置中为 0.2m）
4. 确认 `/scan` 数据正常：
   ```bash
   docker exec lio_nav2 bash -c "source install/setup.bash && ros2 topic hz /scan"
   ```

---

### Q12：Cartographer RViz 显示 /map 为空

**现象**
RViz 添加了 `/map` 显示，Fixed Frame 设为 `map`，但看不到地图。

**解决**
1. 确认 `cartographer_occupancy_grid_node` 在运行：
   ```bash
   docker exec lio_nav2 bash -c "source install/setup.bash && ros2 topic hz /map"
   ```
2. RViz 的 Fixed Frame 应当设置为 `map`。如果 `map` 帧未发布（建图刚开始或纯定位未初始化），会显示 "No tf data" 错误——等 Cartographer 初始化完成即可
3. 确认 Cartographer 已收到足够的数据并开始建图，`Cartographer` tmux 窗口应打印 scan 匹配信息

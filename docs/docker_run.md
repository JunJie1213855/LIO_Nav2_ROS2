# Docker 运行指南

> 前提：已构建镜像 `lio_nav2:humble` 并完成 `colcon build`。如果还没有，请先参考 [`docs/docker_build.md`](docker_build.md)。

---

## 1. 启动容器

```bash
# 允许容器访问 X11（每次开机执行一次）
xhost +local:docker

# 启动运行容器（仿真用 8 核，Gazebo + RViz 需要 X11）
docker run -d --name lio_nav2 \
  --network host --ipc host --cpus 8 \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri:/dev/dri \
  -v $PWD:/ws \
  lio_nav2:humble sleep infinity
```

> 如果 Gazebo/RViz 报 GLX 错误，容器内执行 `export LIBGL_ALWAYS_SOFTWARE=1` 切换软件渲染。

---

## 2. 建图

系统里有两条建图路线，选一条即可：

| | 路线 A: SLAM Toolbox | 路线 B: Cartographer |
|--|----------------------|----------------------|
| 建图启动 | `mapping_sim_docker.sh` | `mapping_sim_carto_docker.sh` |
| tmux 会话 | `mapping_sim` | `mapping_carto` |
| 地图格式 | `.pgm` + `.yaml` | `.pbstream` → 导出 `.pgm` + `.yaml` |
| 导航配套 | KISS-Matcher 重定位 | Cartographer 纯定位 |
| 适合场景 | 小范围快速建图 | 大范围建图，建图定位统一框架 |

---

### 2A. SLAM Toolbox 建图

**启动**

```bash
docker exec -it lio_nav2 /ws/scripts/mapping_sim_docker.sh
```

tmux 窗口一览：

```
GUI控制 | FAST-LIO | lio_interface | Gazebo | sensor_scan | pc2laser | slam_toolbox
```

**操作**

1. `docker exec -it lio_nav2 tmux attach -t mapping_sim` 查看各节点输出
2. 在 `GUI控制` 窗口（第一个窗口）用鼠标/键盘遥控小车遍历环境
3. 观察 `slam_toolbox` 窗口确认建图正常

**保存地图**

SLAM Toolbox 路线需要保存**两个文件**，分别来自不同节点：

| 产物 | 来源 | 保存命令 | 用途 |
|------|------|---------|------|
| `.pgm` + `.yaml` | **SLAM Toolbox**（2D 栅格地图） | `save_map.sh` | Nav2 `map_server` 加载 |
| `.pcd` | **FAST-LIO**（3D 点云地图） | `save_pcd.sh` | KISS-Matcher 重定位加载 |

> PCD 不是 SLAM Toolbox 生成的。FAST-LIO 内部用 ikd-Tree 持续累积去畸变点云，`/map_save` 服务把这份 3D 点云地图导出为 `.pcd`。KISS-Matcher 需要它来做 FPFH 3D 全局匹配。

```bash
# 步骤 1: 保存 2D 栅格地图 (.pgm + .yaml)
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  /ws/scripts/save_map.sh"

# 步骤 2: 保存 3D 点云地图 (.pcd，FAST-LIO 累积的全局点云)
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  /ws/scripts/save_pcd.sh"
```




**停止**

```bash
docker exec lio_nav2 tmux kill-session -t mapping_sim
```

---

### 2B. Cartographer 建图

**启动**

```bash
docker exec -it lio_nav2 /ws/scripts/mapping_sim_carto_docker.sh
```

tmux 窗口一览：

```
GUI控制 | FAST-LIO | lio_interface | Gazebo | sensor_scan | pc2laser | Cartographer | RViz
```

RViz 窗口会自动弹出，实时显示 Cartographer 建图效果：

| RViz 显示项 | 话题 | 说明 |
|------------|------|------|
| CartographerMap | `/map` | 实时 OccupancyGrid — 这就是地图 |
| LaserScan | `/scan` | 当前 2D 激光扫描（红色） |
| RegisteredScan | `/registered_scan` | 3D 注册点云（彩色） |
| TF | 坐标树 | `map→odom→base_footprint→chassis→livox_frame` |
| RobotModel | `/robot_description` | 机器人 3D 模型 |

**操作**

1. `docker exec -it lio_nav2 tmux attach -t mapping_carto` 查看各节点输出
2. 在 RViz 中观察 `/map` 逐步构建
3. 遥控小车**慢速平稳**遍历环境（Cartographer 需要 scan 重叠来做子图匹配）
4. 观察 RViz 中回环修正——走回之前经过的地方，地图会全局调整对齐

**保存地图**

Cartographer 路线需要保存的文件：

| 产物 | 来源 | 用途 |
|------|------|------|
| `.pbstream` | **Cartographer** `/write_state` 服务 | Cartographer 纯定位加载 |
| `.pgm` + `.yaml` | 由 `.pbstream` 导出 | Nav2 `map_server` 加载 |

> ❌ **不需要保存 PCD**。Cartographer 纯定位用 `.pbstream` 中的子图做 2D scan-to-submap 匹配，不走 FPFH 3D 全局搜索，PCD 对它没有用。

```bash
# 步骤 1: 写入 pbstream（核心——保存子图+位姿图状态）
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    '{filename: \"/ws/src/me_nav2_bringup/map/map.pbstream\", include_unfinished_submaps: true}'"

# 步骤 2: 结束轨迹（停止接收新数据）
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \
    '{trajectory_id: 0}'"

# 步骤 3: 导出 pbstream → .pgm + .yaml
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename /ws/src/me_nav2_bringup/map/map.pbstream \
    -map_filestem /ws/src/me_nav2_bringup/map/my_map"
```

> ⚠️ **关键理解**：Cartographer 的三个步骤各有分工：
> - `/write_state` → 实际**写文件**（`.pbstream` 落盘）
> - `/finish_trajectory` → 标记轨迹完成，但**不写文件**
> - `pbstream_to_ros_map` → 把 `.pbstream` 转成 Nav2 能用 `.pgm`

**停止**

```bash
docker exec lio_nav2 tmux kill-session -t mapping_carto
```

---

### 2C. 两条建图路线产物对比

```
路线 A: SLAM Toolbox
  保存:
    SLAM Toolbox ──→ save_map ──→ .pgm + .yaml  (2D 栅格地图, Nav2 用)
    FAST-LIO     ──→ /map_save → .pcd           (3D 点云地图)
                         ├── robo_map.pcd        (ikdtree 地图, 备份)
                         └── dense_map.pcd        (稠密点云, KISS-Matcher 用)

路线 B: Cartographer
  保存:
    Cartographer ──→ /write_state ──→ .pbstream      (子图+位姿图, 纯定位用)
                 ──→ /finish_trajectory               (结束轨迹)
                 ──→ pbstream_to_ros_map ──→ .pgm + .yaml (Nav2 用)
                 ❌ 不需要 .pcd
```

| 对比维度 | 路线 A | 路线 B |
|---------|--------|--------|
| 2D 栅格地图 | SLAM Toolbox 直接输出 | 从 .pbstream 导出 |
| 内部地图 | FAST-LIO `.pcd`（3D 点云） | Cartographer `.pbstream`（子图+位姿图） |
| 重定位数据 | `.pcd`（dense_map.pcd）→ KISS-Matcher | `.pbstream` → Cartographer 纯定位 |
| 保存命令数 | 两条（save_map + save_pcd） | 三条（write_state + finish_trajectory + pbstream_to_ros_map） |

---

## 3. 导航

导航模式加载已建好的地图，实时重定位并执行路径规划与控制。

**导航路线的选择取决于建图路线**：

| 建图用 | 导航用 | 原因 |
|--------|--------|------|
| SLAM Toolbox | KISS-Matcher（路线 A） | KISS 加载建图时保存的 `.pcd`，与 SLAM Toolbox 地图配套 |
| Cartographer | Cartographer 纯定位（路线 B） | 加载建图时保存的 `.pbstream`，同一框架 |

> ⚠️ **不能跨配**——KISS 不认识 `.pbstream`，Cartographer 不认识 `.pcd`。

---

### 3A. SLAM Toolbox + KISS-Matcher 导航

**前置准备**

确认以下文件存在：

| 文件 | 配置位置 | 说明 |
|------|---------|------|
| `my_map.pgm` + `my_map.yaml` | `my_nav2_launch.py` 中的 `map_yaml_file` | 静态占据栅格地图 |
| `dense_map.pcd` | `global_kiss_matcher_relocalization_launch.py` 中的 `prior_pcd_file` | 3D 稠密点云地图 |

> 建图后需用 `scripts/save_pcd.sh` 保存 PCD 先验地图给 KISS-Matcher。

**启动**

```bash
docker exec -it lio_nav2 /ws/scripts/nav2_sim_docker.sh
```

tmux 窗口一览：

```
GUI控制 | FAST-LIO | lio_interface | Gazebo | sensor_scan | pc2laser | KISS+GICP | Nav2
```

**操作**

1. `docker exec -it lio_nav2 tmux attach -t nav2_sim` 查看节点输出
2. 等待 `KISS+GICP` 窗口打印 `KISSMatcher initialization succeeded`
   - 如果一直显示 `initializing`：让机器人原地缓慢旋转几圈，加速累计点云
3. 打开 RViz（容器内或宿主机均可），Fixed Frame 设为 `map`
4. 用 **"2D Pose Estimate"** 工具（工具栏绿色箭头）在机器人真实位置点击 + 拖动方向
5. 用 **"Nav2 Goal"** 工具点击目标位姿，机器人开始自主导航

**停止**

```bash
docker exec lio_nav2 tmux kill-session -t nav2_sim
```

---

### 3B. Cartographer 纯定位导航

**前置准备**

确认以下文件存在（由建图阶段 2B 生成）：

| 文件 | 配置位置 | 说明 |
|------|---------|------|
| `my_map.pgm` + `my_map.yaml` | `my_nav2_launch.py` 中的 `map_yaml_file` | 静态地图（从 .pbstream 导出） |
| `map.pbstream` | `nav2_sim_carto_docker.sh` 中的 `load_state_filename` | Cartographer 纯定位加载 |

> 确保 `my_nav2_launch.py` 中 `map_yaml_file` 指向 Cartographer 导出的 `.yaml` 文件。

**启动**

```bash
docker exec -it lio_nav2 /ws/scripts/nav2_sim_carto_docker.sh
```

tmux 窗口一览：

```
GUI控制 | FAST-LIO | lio_interface | Gazebo | sensor_scan | pc2laser | Carto定位 | Nav2
```

与 2B（建图）的区别：
- `Cartographer` 窗口换成了 `Carto定位`
- 启动参数加了 `-pure_localization`，不建图只匹配
- 不启动 `cartographer_occupancy_grid_node`（`/map` 由 Nav2 的 `map_server` 提供）

**操作**

1. `docker exec -it lio_nav2 tmux attach -t nav2_carto` 查看节点输出
2. 等待 `Carto定位` 窗口输出定位信息
3. 打开 RViz，Fixed Frame 设为 `map`
4. 用 **"2D Pose Estimate"** 工具给初始位姿——Cartographer 收到初始位姿后开始实时匹配
5. 观察 `map→odom` TF 是否正常发布（RViz 中机器人模型应对齐到地图上）
6. 用 **"Nav2 Goal"** 发送目标，开始导航

**如果定位失败**

- 确认 `.pbstream` 路径正确：`docker exec lio_nav2 ls -la /ws/src/me_nav2_bringup/map/map.pbstream`
- 让机器人原地缓慢旋转几圈，扩大 scan 覆盖范围
- 在 RViz 中重新给 **"2D Pose Estimate"**
- 检查 `/scan` 数据是否正常：`docker exec lio_nav2 bash -c "source install/setup.bash && ros2 topic hz /scan"`

**停止**

```bash
docker exec lio_nav2 tmux kill-session -t nav2_carto
```

---

## 4. 关闭 / 清理

```bash
# 只停所有节点和 tmux 会话，容器保留运行
./scripts/docker_shutdown.sh

# 停节点 + 停止容器（下次 docker start 即可恢复）
./scripts/docker_shutdown.sh --stop

# 停节点 + 删除容器（下次需重新 docker run）
./scripts/docker_shutdown.sh --rm

# 预览将要执行的操作，不实际执行
./scripts/docker_shutdown.sh --dry-run
```

---

## 5. 快速参考卡片

### 完整流程（从头开始）

```bash
# ═══ 准备 ═══
xhost +local:docker
docker run -d --name lio_nav2 --network host --ipc host --cpus 8 \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri:/dev/dri \
  -v $PWD:/ws lio_nav2:humble sleep infinity

# ═══ 建图 ═══
# 路线 A: SLAM Toolbox
docker exec -it lio_nav2 /ws/scripts/mapping_sim_docker.sh        # 启动
docker exec -it lio_nav2 tmux attach -t mapping_sim               # 查看
# ... 遥控小车建图 ...
docker exec lio_nav2 tmux kill-session -t mapping_sim             # 停止

# 路线 B: Cartographer
docker exec -it lio_nav2 /ws/scripts/mapping_sim_carto_docker.sh  # 启动
docker exec -it lio_nav2 tmux attach -t mapping_carto             # 查看
# ... 遥控小车建图，观察 RViz ...
# 保存地图（完成建图后）:
docker exec lio_nav2 bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    '{filename: \"/ws/src/me_nav2_bringup/map/map.pbstream\", include_unfinished_submaps: true}' && \
  ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory '{trajectory_id: 0}' && \
  ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename /ws/src/me_nav2_bringup/map/map.pbstream \
    -map_filestem /ws/src/me_nav2_bringup/map/my_map"
docker exec lio_nav2 tmux kill-session -t mapping_carto             # 停止

# ═══ 导航 ═══
# 路线 A: KISS-Matcher（配 SLAM Toolbox）
docker exec -it lio_nav2 /ws/scripts/nav2_sim_docker.sh           # 启动
docker exec -it lio_nav2 tmux attach -t nav2_sim                  # 查看
# ... RViz "2D Pose Estimate" → "Nav2 Goal" ...

# 路线 B: Cartographer 纯定位（配 Cartographer）
docker exec -it lio_nav2 /ws/scripts/nav2_sim_carto_docker.sh     # 启动
docker exec -it lio_nav2 tmux attach -t nav2_carto                # 查看
# ... RViz "2D Pose Estimate" → "Nav2 Goal" ...

# ═══ 关闭 ═══
./scripts/docker_shutdown.sh --stop
```

### tmux 会话速查

| 脚本 | 会话名 | 用途 | 里程计 | SLAM/定位 |
|------|--------|------|--------|-----------|
| `mapping_sim_docker.sh` | `mapping_sim` | 建图 | FAST-LIO | SLAM Toolbox |
| `mapping_sim_carto_docker.sh` | `mapping_carto` | 建图 | FAST-LIO | Cartographer |
| `nav2_sim_docker.sh` | `nav2_sim` | 导航 | FAST-LIO | KISS-Matcher |
| `nav2_sim_carto_docker.sh` | `nav2_carto` | 导航 | FAST-LIO | Cartographer 纯定位 |

### tmux 快捷键

| 快捷键 | 功能 |
|--------|------|
| `Ctrl-b n` | 下一个窗口 |
| `Ctrl-b p` | 上一个窗口 |
| `Ctrl-b d` | 退出查看（节点继续后台运行） |
| `Ctrl-b [` | 进入滚动模式，可上下翻看终端历史输出 |

### 两条路线总览

```
路线 A: SLAM Toolbox + KISS-Matcher
  建图:   /scan ──→ SLAM Toolbox ──→ /map + map→odom TF
  保存:   ros2 service call /slam_toolbox/save_map  →  .pgm + .pcd
  导航:   /registered_scan ──→ KISS-Matcher (加载 .pcd) ──→ map→odom TF
         map_server (加载 .pgm) ──→ /map

路线 B: Cartographer 全套
  建图:   /scan + /imu ──→ Cartographer ──→ /map + map→odom TF
  保存:   ros2 service call /write_state → .pbstream
          ros2 service call /finish_trajectory
          ros2 run ... cartographer_pbstream_to_ros_map → .pgm
  导航:   /scan + /imu ──→ Cartographer 纯定位 (加载 .pbstream) ──→ map→odom TF
         map_server (加载 .pgm) ──→ /map
```

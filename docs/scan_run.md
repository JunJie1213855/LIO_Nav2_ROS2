# SCAN-Planner 编译与运行指南（scan_run.md）

> SCAN-Planner 局部避障轨迹规划器（B-spline + A* 绕障）。
> 在 Gazebo + FAST-LIO 仿真环境中，机器人通过 RViz 2D Goal Pose 指定目标，规划器实时避障。
> 环境：Ubuntu 22.04 + ROS 2 Humble + Docker（容器名 `lio_nav2`，工作空间挂载到 `/ws`）。

---

## 1. 管线架构

```
Gazebo (get_urdf, simple_car + 50线LiDAR + IMU)
   └─> FAST-LIO (里程计 + /cloud_registered)
          ├─> lio_interface          (/odom 里程计)
          └─> sensor_scan_generation (/registered_scan odom 帧点云)
                 └─> cloud_z_filter  (z 轴过滤 → /registered_scan_filtered)
                        └─> SCAN-Planner (scan_planner_node)
                               ├─ 全局轨迹: min-snap 直线 (起点→目标)
                               ├─ 局部目标: 直线上 planning_horizon 距离处
                               ├─ 碰撞段检测 → A* 绕障搜索 → B-spline 优化
                               └─ 发布 B-spline 轨迹
                                      └─> closed_loop_controller → /cmd_vel
```

| 节点 | 包 | 作用 |
|------|-----|------|
| `get_urdf` | `get_urdf` | Gazebo 仿真世界 + 机器人模型 (simple_car) |
| `fastlio_mapping` | `fast_lio_robosense` | LiDAR 惯性里程计 + 点云配准 |
| `lio_interface` | `lio_interface` | 生成 `/odom` 里程计 |
| `sensor_scan_generation` | `sensor_scan_generation` | 生成 odom 帧 `/registered_scan` |
| `cloud_z_filter` | `nav2_planner_bringup` | Z 轴过滤点云（滤地面/天花板） |
| `scan_planner_node` | `scan_planner` | 局部避障规划核心 |
| `closed_loop_controller` | `scan_planner` | B-spline 轨迹 → `/cmd_vel` |

### SCAN-Planner 多包结构（重要）

```
src/planner/SCAN-Planner/src/planner/
├── bspline_opt/        # B-spline 优化器 (bspline_optimizer.cpp)
├── path_searching/     # A* 搜索 (dyn_a_star.cpp)
├── plan_env/           # 占据栅格地图 (grid_map.cpp)
├── plan_manage/        # 主节点 (scan_planner, planner_manager.cpp, scan_replan_fsm.cpp)
├── traj_utils/         # 轨迹工具
└── scan_planner_msgs/  # 自定义消息
```

---

## 2. 构建

### 2.1 编译命令（必须用 packages-up-to！）

```bash
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && cd /ws &&
  MAKEFLAGS='-j4' colcon build --packages-up-to scan_planner \
    --symlink-install --executor sequential
"
```

> **关键**：SCAN-Planner 是多包结构，修改了 `bspline_opt`/`path_searching`/`plan_env` 等依赖库源码后，
> 必须用 `--packages-up-to scan_planner` 编译，而不是 `--packages-select scan_planner`。
> 后者只编译主包，**依赖库的修改不会生效**（这是历史上避障反复失败的隐藏根因）。

### 2.2 编译限速（避免卡死）

```bash
MAKEFLAGS='-j4'            # 每包限 4 线程
--executor sequential      # 包间顺序编译
```

> 不加限速时 12 核全占满会导致宿主机/容器卡死退出。

### 2.3 验证修改已生效

```bash
# 检查 DIAG 日志字符串是否进入可执行文件
docker exec lio_nav2 bash -c "strings /ws/build/scan_planner/scan_planner_node | grep -c DIAG"
# 输出 >= 1 说明依赖库已重新编译
```

---

## 3. 运行

### 3.1 一键启动

```bash
# 宿主机先授权 X11（容器重启后需重新执行）
xhost +local:docker

docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/gazebo_scan.sh"
```

### 3.2 tmux 窗口

```bash
docker exec -it lio_nav2 tmux attach -t scan_gz
```

| 窗口 | 内容 |
|------|------|
| `Gazebo` | Gazebo 仿真世界 |
| `FAST-LIO` | LiDAR 惯性里程计 |
| `lio_if` | 里程计接口 |
| `sensor` | 点云生成 |
| `SCAN` | SCAN-Planner 规划器 |
| `SP-RViz` | RViz 可视化 |

### 3.3 指定目标

切到 `SP-RViz` 窗口，用 **"2D Goal Pose"** 工具点击目标点。

或命令行发布：

```bash
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
  ros2 topic pub /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: -5.0, y: 4.0, z: 0.0}, orientation: {x:0,y:0,z:0,w:1}}}' --once"
```

### 3.4 关键参数（gazebo_scan.sh 内）

| 参数 | 值 | 说明 |
|------|-----|------|
| `double_cylinder_radius` | 0.45 | 机器人碰撞半径（障碍物膨胀 + 机器人圆柱共用此值） |
| `double_cylinder_offset` | 0.18 | 双圆柱前后偏移 |
| `optimization.lambda_collision` | 50.0 | 碰撞代价权重 |
| `optimization.dist0` | 1.0 | **碰撞安全距离**（必须 >= 实际碰撞距离） |
| `grid_map.sliding_map_size_x/y` | 50.0 | 地图尺寸 ±25m |
| `grid_map.local_update_range_x/y` | 25.0 | 点云更新范围 |

---

## 4. 问题与解决方案

### P-S1：修改源码后避障始终不生效（隐藏根因）

**现象**：反复修改 `bspline_optimizer.cpp` 等源码，重新编译后行为毫无变化。

**根因**：SCAN-Planner 是多包结构，`bspline_opt`、`path_searching`、`plan_env` 是独立库包。
`colcon build --packages-select scan_planner` 只编译主包，不重编依赖库。

**解决**：
```bash
colcon build --packages-up-to scan_planner
```

---

### P-S2：A* 搜索池太小

**现象**：A* 超时失败，轨迹不绕障。

**根因**：`planner_manager.cpp` 硬编码 A* 池 `100×100×100`（5m³），小于 planning_horizon 7.5m。

**解决**：改为 `200×200×100`（10m×10m×5m）。

```cpp
// planner_manager.cpp
bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(200, 200, 100));
```

---

### P-S3：A* 超时太短

**根因**：`dyn_a_star.cpp` 超时 0.2s，大池搜索不够。

**解决**：0.2s → 0.5s。

---

### P-S4：碰撞检测只覆盖轨迹前 2/3

**根因**：`bspline_optimizer.cpp` 多处 `tmp * 2 / 3` 只检查前 2/3 轨迹，后半段障碍物漏检。

**解决**：全部改为检查完整轨迹 `t < tmp`，控制点循环 `i_end = cols - order_`。

---

### P-S5：地图外点被当作障碍物（-1 处理）

**根因**：`getInflateOccupancy` 对地图外返回 `-1`，C++ 中为真值被当作"占据"，
导致地图边界的控制点被误判为障碍物。

**解决**：碰撞段检测与 check_collision_and_rebound 中统一 `if (occ == -1) occ = 0`。

---

### P-S6：A* 失败导致整个优化放弃

**根因**：`initControlPoints` 中任一碰撞段 A* 失败就 `return`，所有段失去避障梯度。

**解决**：A* 失败时 push 空占位，保持索引对齐，只跳过该段：

```cpp
// A* 失败/路径过短时
a_star_paths.push_back(vector<Eigen::Vector3d>()); // empty placeholder
```

---

### P-S7：FAST-LIO Z 轴偏移 → 地面点被误判为障碍物

**现象**：机器人撞障碍物后翻倒，里程计 Z 漂移到 -16206；或地面点进入地图被当障碍物。

**根因**：`mid360.yaml` 配置了实机 AIRY 的 LiDAR-IMU 外参 `[-0.011, -0.02329, 0.04412]`，
并开启在线外参估计 `extrinsic_est_en: true`。Gazebo 中 LiDAR 与 IMU 实际同位置（同在 `livox_frame`），
外参不匹配 + 在线估计发散导致 Z 轴漂移；地面点 z 不为 0，`cloud_z_filter`（z_min=0.15）滤不掉，
地面进入地图成为障碍物。

**解决**（`src/localization/FAST_LIO/config/mid360.yaml`）：
```yaml
extrinsic_est_en: false
extrinsic_T: [ 0.0, 0.0, 0.0 ]
```

---

### P-S8：碰撞安全距离（dist0）远小于实际碰撞距离

**现象**：规划轨迹看起来绕障了，机器人仍撞上障碍物（碰撞体积太小）。

**根因**：实际碰撞距离 ≈ `double_cylinder_radius × 2 + double_cylinder_offset` ≈ 1.0m，
但 `dist0`（优化器安全距离）只有 0.2~0.3m，优化器允许轨迹逼近到 0.3m，实际在 1.0m 内已碰撞。

**解决**：`optimization.dist0 := 1.0`，并加大碰撞体积：
```
double_cylinder_radius := 0.45
double_cylinder_offset := 0.18
optimization.lambda_collision := 50.0
optimization.dist0 := 1.0
```

---

### P-S9：编译占满线程致容器卡死退出

**现象**：`colcon build` 后容器 Exited(255)。

**根因**：12 核全占满编译。

**解决**：限速编译（见 2.2），容器崩溃后 `docker start lio_nav2` 重启。

---

### P-S10：容器重启后 Gazebo 无法 spawn 机器人

**现象**：`spawn_entity` 报 "Service /spawn_entity unavailable"，gzserver 启动后渲染失败退出。

**根因**：容器重启后 X11 授权失效（"Authorization required"）。

**解决**：宿主机重新执行 `xhost +local:docker`。

---

### P-S11：DDS 通信瘫痪（ros2 node list 报 !rclpy.ok()）

**现象**：节点进程在运行，但 `ros2 node list` 报 `Fault: !rclpy.ok()`，话题查询失败。

**根因**：容器长期运行累积大量僵尸进程（父进程 PID 1 不回收）+ `/dev/shm` 残留 FastDDS 共享内存。

**解决**：
```bash
# 宿主机重启容器清僵尸
docker restart lio_nav2
# 容器内清理共享内存
docker exec lio_nav2 bash -c "rm -rf /dev/shm/*"
```

---

## 5. 诊断技巧

### 5.1 检查 DIAG 日志

`bspline_optimizer.cpp` 已加 DIAG 日志，规划时打印碰撞段数量与 A* 起点/终点：

```
[bspline_opt] [DIAG] detected N collision segments
[bspline_opt] [DIAG] segment i: in[...]=(x,y,z) out[...]=(x,y,z)
```

### 5.2 检查地图与点云

```bash
# 注意 QoS：grid_map 话题是 BEST_EFFORT
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
  ros2 topic echo /grid_map/occupancy_inflate --qos-reliability best_effort --once | head"
```

### 5.3 检查里程计 Z（发散标志）

```bash
ros2 topic echo /odom --once | grep -A3 position
# z 应接近 0；若 z 剧烈变化说明里程计发散（撞障碍物或外参错误）
```

---

## 6. 快速命令速查

```bash
# 编译（完整依赖链 + 限速）
docker exec lio_nav2 bash -c "source /opt/ros/humble/setup.bash && cd /ws && \
  MAKEFLAGS='-j4' colcon build --packages-up-to scan_planner --symlink-install --executor sequential"

# 运行（宿主机）
xhost +local:docker
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/gazebo_scan.sh"

# 查看
docker exec -it lio_nav2 tmux attach -t scan_gz

# 诊断（容器内）
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 node list"

# 崩溃恢复
docker restart lio_nav2 && xhost +local:docker
```

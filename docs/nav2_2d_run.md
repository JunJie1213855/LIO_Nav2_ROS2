# 2D 单线 LiDAR 导航 Pipeline 指南（nav2_2d_run.md）

> 2D 单线 LiDAR 差分小车仿真：Gazebo + Cartographer 2D SLAM + Nav2 在线导航。
> 一键启动后，在 RViz 用 "2D Goal Pose" 点目标，机器人实时建图并自主导航避障。
> 环境：Ubuntu 22.04 + ROS 2 Humble + Docker（容器名 `lio_nav2`，工作空间挂载到 `/ws`）。

---

## 1. 管线架构

```
Gazebo (indoor_2d.world 封闭室内场景)
   └─ 2D 差分小车 (diff_robot_2d.urdf)
         ├─ libgazebo_ros_diff_drive.so → /odom + odom→base_footprint TF
         ├─ 单线 LiDAR (ray, vertical=1) → /scan (LaserScan, 360°, 720 samples)
         └─ IMU → /imu
              └─> Cartographer 2D SLAM (在线)
                    ├─ 订阅 /scan /imu /odom
                    ├─ 发布 /map + map→odom TF
                    └─> Nav2 (在线导航，跳过 map_server/AMCL)
                          ├─ global_costmap ← /map（static_layer 订阅 Cartographer 的 /map）
                          ├─ local_costmap  ← /scan
                          ├─ NavFn 全局规划 + DWB 局部规划
                          └─> /cmd_vel → diff_drive 插件
```

| 节点 | 包 | 作用 |
|------|-----|------|
| `gazebo` | `get_urdf` | 仿真世界 + `diff_robot_2d` 模型 |
| `diff_drive` | gazebo 插件 | 差分驱动 + `/odom` + odom→base_footprint TF |
| `cartographer_node` | `cartographer_ros` | 2D SLAM，发布 map→odom TF |
| `cartographer_occupancy_grid_node` | `cartographer_ros` | 发布 `/map` 占据栅格 |
| `planner_server` | `nav2_planner` | NavFn 全局规划 |
| `controller_server` | `nav2_controller` | DWB 局部规划 |
| `bt_navigator` | `nav2_bt_navigator` | 行为树导航 |
| `behavior_server` | `nav2_behaviors` | 恢复行为（spin/backup） |

### TF 树

```
map ──> odom ──> base_footprint ──> base_link ──> laser
       (Carto)    (diff_drive)       (URDF固定)    (URDF固定)
```

---

## 2. 构建

### 2.1 编译

```bash
docker exec lio_nav2 bash -c "
  source /opt/ros/humble/setup.bash && cd /ws &&
  MAKEFLAGS='-j4' colcon build --packages-select get_urdf me_nav2_bringup \
    --symlink-install --executor sequential
"
```

> 限速 `-j4` + `--executor sequential` 避免占满 CPU 卡死宿主机。
> `refer/` 目录已放 `COLCON_IGNORE`，防止其备份副本导致重复包名报错。

---

## 3. 运行

### 3.1 一键启动

```bash
# 宿主机先授权 X11
xhost +local:docker

# 默认室内场景 indoor_2d
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/nav2_2d_sim.sh"

# 可选：指定其他世界（如 test_world）
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/nav2_2d_sim.sh test_world"
```

### 3.2 tmux 窗口

```bash
docker exec -it lio_nav2 tmux attach -t nav2_2d
```

| 窗口 | 内容 |
|------|------|
| `Gazebo` | Gazebo 仿真（diff_robot_2d + indoor_2d 世界） |
| `Carto` | Cartographer 2D SLAM |
| `Nav2` | Nav2 在线导航 |
| `RViz` | 2D 导航视图（map + scan + costmap） |

### 3.3 指定目标

切到 `RViz` 窗口，用 **"2D Goal Pose"** 工具点击目标点。

或命令行发布：

```bash
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 6.0, y: 6.0, z: 0.0}, orientation: {x:0,y:0,z:0,w:1}}}' --once"
```

### 3.4 室内场景障碍物（indoor_2d.world）

| 模型 | 位置 | 尺寸 |
|------|------|------|
| 四面墙 | x/y = ±10 | 20×0.2×1 m（围成 20×20 房间） |
| box_1 | (-3, 3) | 2×2×1 m |
| box_2 | (4, -2) | 1.5×1.5×1 m |
| box_3 | (2, 5) | 1×1×1 m |
| box_4 | (-5, -4) | 1.2×1.2×1 m |

---

## 4. 问题与解决方案

### P-N1：机器人原地打滑，轮子狂转但车不动

**现象**：`gz model --info` 显示 wheel joint 转了 20+ 圈（angle≈144 rad），但模型位置几乎不变。

**根因**：URDF 几何让轮子底部"刚好接触"地面（接触深度为零），Gazebo ODE 物理引擎无法产生足够接触力/摩擦力。

**解决**：让轮子嵌入地面约 6mm，产生真实接触力：
```xml
<!-- diff_robot_2d.urdf -->
<!-- 轮子中心 z = 0.06 - 0.033 = 0.027，半径 0.033 → 底 -0.006（嵌入 6mm）-->
<joint name="wheel_left_joint" ...>
  <origin xyz="0 0.08 -0.033" rpy="-1.5708 0 0"/>
</joint>
<!-- 底盘 box 底保持离地（不嵌入），避免卡死 -->
```

---

### P-N2：IMU 导致 map→odom TF 3D 发散

**现象**：开启 `use_imu_data=true` 时，map→odom TF 出现 Z=-2.7m、Roll/Pitch≈33° 的发散。

**根因**：空旷场景（test_world.world）scan matching 特征少，无法约束 IMU 姿态积分漂移。

**解决**：
1. 换成室内场景（墙壁提供连续特征）后，IMU 稳定（Roll≈1.8°，Z≈0）。
2. 若仍用空旷场景，可临时 `use_imu_data=false` 纯 scan matching。

> 结论：IMU 本身数据正常（gravity +9.8 在 z 正确），问题在于空旷场景约束不足，不是 IMU 配置错误。

---

### P-N3：map→odom TF 持续漂移，机器人追着"移动的目标"越走越偏

**现象**：发布 map 坐标目标 (3,2) 后，机器人却走到 (-7.5, 6.5)。map→odom TF 8 秒漂移 1.65m。

**根因**：test_world.world 是稀疏箱子场景，空旷区域对 2D SLAM 特征不足，Cartographer 定位持续漂移。固定 map 坐标的目标随地图漂移而"移动"，机器人被牵着走。

**解决**：新建 `indoor_2d.world` 封闭室内场景（四面墙 + 内部障碍物），墙壁提供连续 scan matching 特征。室内场景下 map→odom 漂移 < 5mm/10s，导航稳定。

---

### P-N4：colcon 报 Duplicate package names

**现象**：`refer/LIO_Nav2_ROS2` 备份副本与 `src/` 有同名包，colcon 报重复包名。

**解决**：在 `refer/` 目录放 `COLCON_IGNORE` 空文件。

---

### P-N5：cartographer_2d_launch.py 报 NameError

**现象**：`NameError: name 'LaunchConfiguration' is not defined`。

**解决**：补 `from launch.substitutions import LaunchConfiguration`。

---

### P-N6：目标 tolerance 太严导致反复触发恢复行为

**现象**：机器人到达目标附近 0.26m，但 `xy_goal_tolerance=0.035` 无法精确到达，反复触发 spin 恢复。

**解决**：放宽 `xy_goal_tolerance: 0.15`，footprint 调整为 diff_robot_2d 尺寸 `[0.15, 0.12]`。

---

## 5. 关键文件

| 文件 | 说明 |
|------|------|
| `src/get_urdf/model/diff_robot_2d.urdf` | 2D 差分小车（2 驱动轮 + 2 万向轮 + 单线 LiDAR + IMU） |
| `src/get_urdf/worlds/indoor_2d.world` | 封闭室内场景（四面墙 + 4 个障碍物） |
| `src/get_urdf/launch/get_urdf_launch.py` | 支持 `robot:=` 与 `world_path:=` 参数 |
| `src/me_nav2_bringup/config/cartographer_2d.lua` | Cartographer 2D 配置（use_odometry + IMU） |
| `src/me_nav2_bringup/launch/cartographer_2d_launch.py` | Cartographer 启动（cartographer_node + occupancy_grid_node） |
| `src/me_nav2_bringup/launch/nav2_online_launch.py` | Nav2 在线导航（无 map_server/AMCL） |
| `src/me_nav2_bringup/config/nav2_params.yaml` | Nav2 参数（footprint/tolerance 已适配 2D 小车） |
| `scripts/nav2_2d_sim.sh` | 一键启动脚本（可选世界参数） |

---

## 6. 快速命令速查

```bash
# 编译
docker exec lio_nav2 bash -c "source /opt/ros/humble/setup.bash && cd /ws && \
  MAKEFLAGS='-j4' colcon build --packages-select get_urdf me_nav2_bringup --symlink-install --executor sequential"

# 运行
xhost +local:docker
docker exec -it lio_nav2 bash -c "cd /ws && bash scripts/nav2_2d_sim.sh"

# 查看
docker exec -it lio_nav2 tmux attach -t nav2_2d

# 诊断（注意 RMW 匹配）
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 node list"

# 检查定位稳定性（两次采样对比 map→odom）
docker exec lio_nav2 bash -c "source /ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
  ros2 run tf2_ros tf2_echo map odom"
```

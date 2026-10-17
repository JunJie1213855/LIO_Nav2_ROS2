# mrtsp_exploration_ros2 快速上手

单机器人 MRTSP 边界探索节点（ROS 2）。本文档记录在 **Humble + TurtleBot3 + Nav2 + slam_toolbox** 环境下的完整运行命令，以及调试中遇到的坑和修复。

## 1. 环境准备

本包官方标注为 ROS 2 Jazzy，但用到的 rclpy / nav2_msgs API 在 **Humble 上可运行**，需要额外处理两件事：

```bash
# (1) ros 依赖
# TurtleBot3 仿真包（会自动拉取 gazebo_ros_pkgs）
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo

# Nav2 导航栈
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# (2) 第三方 Python 依赖
# 注意：必须用 pip 装 scipy！不要用 apt（apt 的 scipy 1.8 与
#      本机 pip 装的 numpy 2.2.6 不兼容）
pip install scipy numpy==1.26.0

# (3) 编译
cd ~/ros_ws/frontier_exploration_ws
colcon build --packages-select mrtsp_exploration_ros2 --symlink-install
source install/setup.bash
```

> 已在本仓库源码上打过 Humble 兼容补丁，见文末「已修复的问题」。

## 2. 运行命令（按顺序，每个终端一个）

```bash
# 终端 1：Gazebo 世界
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2：Nav2 导航栈（根命名空间）
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# 终端 3：SLAM 建图
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# 终端 4：RViz 可视化
ros2 launch nav2_bringup rviz_launch.py

# 终端 5：MRTSP 探索节点（根命名空间，自动开始）
ros2 launch mrtsp_exploration_ros2 explore.launch.py use_sim_time:=true
```

节点启动后**自动开始探索**（无需 enable 命令）：等 `/map` 和 `/global_costmap/costmap` 到位，自动找边界、下发 `NavigateToPose` 目标，直到边界耗尽输出 `Exploration complete`。

### RViz 里添加的调试话题（By topic）

| 话题 | 类型 | 含义 |
|---|---|---|
| `/explore/frontiers` | MarkerArray | 所有边界簇 |
| `/explore/selected_frontier` | PoseStamped | 当前选中的边界目标 |
| `/explore/optimized_map` | OccupancyGrid | 优化后的决策图（双边滤波+膨胀） |

### 查看运行状态

```bash
ros2 node info /mrtsp_explorer                 # 订阅/发布话题
ros2 topic hz /map                             # 地图是否在更新（0.2Hz 左右）
ros2 topic echo /explore/frontiers             # 边界
```

## 3. 关键参数（config/params.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map_topic` | `/map` | SLAM 地图（根命名空间绝对话题） |
| `costmap_topic` | `/global_costmap/costmap` | Nav2 全局代价地图 |
| `local_costmap_topic` | `/local_costmap/costmap` | Nav2 局部代价地图 |
| `navigate_to_pose_action_name` | `navigate_to_pose` | Nav2 导航 Action（根命名空间下为 `/navigate_to_pose`） |
| `global_frame` / `robot_base_frame` | `map` / `base_footprint` | TF 帧 |
| `occ_threshold` | **90**（原 50，见问题 2） | 代价地图阻挡判定阈值 |
| `frontier_min_goal_distance_m` | **0.4**（原 0.0，见问题 3） | 派发目标距机器人最小距离 |
| `sigma_s` / `sigma_r` | 2.0 / 30.0 | 双边滤波参数 |
| `min_frontier_size_cells` | 5 | 最小边界簇大小 |
| `planning_rate_hz` | 1.0 | 重规划频率 |

## 4. 已修复的问题（Humble 适配 & 探索卡死）

### 问题 1：`NavigateToPose_Result` 没有 `error_code`，节点崩溃

- **现象**：首次目标完成后立即崩溃 `AttributeError: 'NavigateToPose_Result' object has no attribute 'error_code'`
- **原因**：`error_code`/`error_msg` 是 Jazzy 新增字段，Humble 的 result 是空的
- **修复**：`explorer_node.py` 中改用 `getattr(result, "error_code", "n/a")` 兜底

### 问题 2：`occ_threshold: 50` 导致"没有边界"

- **现象**：节点启动后立刻输出 `Exploration complete: no frontiers remain`，但 `/map` 里 87% 还是 unknown
- **原因**：边界格要求 8 邻域在代价地图里都不被阻挡；Nav2 膨胀层把墙周围一圈（代价 37-89）全标成"阻挡"，而阈值 50 把这些全挡了，489 个候选边界被拒 484 个
- **修复**：`occ_threshold` 调到 **90**，只把真正致死的（90+）当阻挡

### 问题 3：探索"走一步就卡死"——瞬间 SUCCEEDED 死循环（重点）

- **现象**：目标点 40ms 就 `SUCCEEDED`，机器人不动，同一个目标反复派发
- **原因**：下发目标是边界**中心**（位于 unknown 区内部）。Nav2 无法规划进 unknown 格，所以在默认 `xy_goal_tolerance`（0.25m）边缘就判定"达成"；机器人停在边界边上不进去 → 边界扫不开 → 同一边界反复选中 → 机器人已处容差内 → 无限循环。静止时 slam 不更新地图，循环自我维持
- **修复**（`explorer_node.py` + `params.yaml`）：
  1. 下发**可达自由点**：取边界旁最近边界中心的自由格，而不是 unknown 区中心，Nav2 能真正开到
  2. **派发门控按实际目标点距离**：按 MRTSP 顺序逐个检查，跳过可达目标点 < 0.4m 的边界，选第一个真正需要导航的（不是按边界中心过滤）
  3. `frontier_min_goal_distance_m: 0.4`（约为 Nav2 容差的 1.5~2 倍）
  4. 全部目标太近时回退最近一个，避免冻死

- **验证**：修复后每个目标耗时 2~13s（真实导航），边界数随探索增长（如 3→12），最后正常 `Exploration complete`

### 调试技巧（本次用到的命令）

```bash
# 看机器人 map 系真实位置（odom 和 map 可能不一致！）
ros2 run tf2_ros tf2_echo map base_footprint

# 看目标点距机器人是否落入 Nav2 容差（0.25m）
# Nav2 目标容差参数：
ros2 param list /controller_server | grep -i tolerance

# 用独立节点验证 /map 能否被默认 QoS 收到（排除 QoS 问题）
# 判断地图是否在更新：两次 echo /map 对比 data 是否变化
```
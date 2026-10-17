# rrt_explore 快速上手

C++ RRT 边界探索节点（多机器人，本机单机使用）。本文档记录在 **Humble + TurtleBot3 + Nav2 + slam_toolbox** 环境下的完整运行命令，以及让机器人真正动起来的全部修复。

## 1. 环境准备

```bash
# 编译（C++ 包，编译约 1 分钟）
cd ~/ros_ws/frontier_exploration_ws
colcon build --packages-select rrt_explore --symlink-install
source install/setup.bash
```

## 2. 运行命令（按顺序，每个终端一个）

```bash
# 终端 1：Gazebo 世界
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2：SLAM 建图
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# 终端 3：RViz 可视化
ros2 launch nav2_bringup rviz_launch.py

# 终端 4：Nav2 导航栈 —— 必须用探索版参数！
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True \
  params_file:=~/ros_ws/frontier_exploration_ws/src/rrt-explore/config/nav2_params_explore.yaml

# 终端 5：RRT 探索节点 —— 必须带 namespace！
ros2 launch rrt_explore rrt.launch.py namespace:=robot1

# 终端 6：启用探索
ros2 topic pub --once /robot1/rrt/enable std_msgs/msg/Bool "{data: true}"
```

两个"必须"：
- **Nav2 用探索版参数**（`track_unknown_space: false`），否则规划器穿不过 unknown 区，边界目标全部 ABORT
- **rrt 带 `namespace:=robot1`**，否则报 `Cannot extract robot id from node namespace {/}` 退出

### RViz 调试话题

| 话题 | 类型 | 含义 |
|---|---|---|
| `/robot1/frontier_targets` | Marker | RRT 检测到的边界点 |
| `/robot1/rrt_segments` | Marker | RRT 树 |
| `/robot1/end_detection_frontiers` | Marker | 全图扫描的剩余边界 |
| `/exploration_state` | rrt_explore/ExplorationState | 探索状态 |

### 状态检查

```bash
ros2 node info /robot1/rrt                          # 应看到 action client: /navigate_to_pose
ros2 action info /navigate_to_pose                  # /robot1/rrt 应出现在 client 列表
ros2 topic hz /cmd_vel                              # 探索中应非零流动
ros2 run tf2_ros tf2_echo map base_footprint        # 机器人 map 系真实位置（别信 odom！）
```

## 3. 关键参数（config/params.yaml）

| 参数 | 值 | 说明 |
|---|---|---|
| `map_topic` | `/map` | SLAM 地图（绝对话题） |
| `costmap_topic` | `/global_costmap/costmap` | Nav2 全局代价地图 |
| `robot_base_frame` | `base_footprint` | 机器人基座帧 |
| `robot_frame_prefix` | `robot` | 命名空间前缀，必须 `robot<ID>` 形式 |
| `rate` | 100.0 | 探索频率 |
| `costmap_pixel_threshold` | **100**（原 30，见问题 3） | 边界剪枝阈值 |
| `robot_count` | 1 | 机器人数量 |

## 4. 已修复的问题（让机器人真正动起来）

### 问题 1：不带 namespace 报错退出

- **现象**：`Cannot extract robot id from node namespace {/} with prefix {robot}`
- **原因**：`rrt.cpp` 从自身命名空间提取机器人编号，根命名空间 `/` 解析不出
- **修复**：启动必须带 `namespace:=robot1`（这是包的设计要求，不是 bug）

### 问题 2：action client 连不上 Nav2

- **现象**：有 `Goal:` 输出但机器人不动；`ros2 action info /robot1/navigate_to_pose` 显示 0 个服务器
- **原因**：remap 规则 `navigate_to_pose:=/navigate_to_pose` 对 action **不生效**——action 内部话题是 `/name/_action/*`，裸名字的 remap 规则匹配不到。client 停在 `/robot1/navigate_to_pose`（没服务器），目标发去无人处
- **修复**：`rrt.cpp` 里 action 名硬编码为绝对路径 `/navigate_to_pose`，绕过命名空间直连根 Nav2

### 问题 3：Nav2 规划不到边界（track_unknown_space）

- **现象**：目标被 Nav2 `ABORTED`，手动 `ros2 action send_goal` 也中止
- **原因**：地图 87% 是 unknown，Nav2 默认 `track_unknown_space: true` 把 unknown 当 lethal，机器人被困在已知小口袋，规划不到边界
- **修复**：探索版 Nav2 参数 `nav2_params_explore.yaml` 设 `track_unknown_space: false`，允许规划器穿过 unknown 区（自主探索的标准配置）

### 问题 4：代价地图膨胀层把边界全删光

- **现象**：启动后几秒就 `Exploration Done!`，一个目标都不发
- **原因**：实时激光（obstacle 层）把贴墙的边界格标成 cost 99-100（致死）。rrt 两处过滤器全删：
  - `find_frontiers()` 的 `costmap > 0` 过滤
  - 剪枝 `mapValue(costmap) > costmap_pixel_threshold_(30)`
- **修复**：两处都放宽到只删真正占据的（`>= 100`），`costmap_pixel_threshold` 30 → 100。配合下面的目标吸附，贴墙但可探索的边界得以保留

### 问题 5：边界目标在 unknown 区，Nav2 到不了

- **现象**：修复 3 后 Nav2 能规划，但 rrt 的目标点本身在 unknown 区，仍被 lethal 挡住
- **原因**：rrt 把目标设在边界中心（unknown 区内部）
- **修复**：`rrt.cpp` 发目标前把目标**吸附到最近的免费格**（`MAP_PIXEL_FREE`），Nav2 可到达，机器人开到边界旁由传感器扫开

### 验证

```
Goal: -1.99 → -1.89 → -1.34 → -0.44 → -0.34 → 0.84 → 1.99   ← 目标覆盖全图持续推进
nonzero cmd_vel: 268/269                                       ← 机器人 99% 时间收到速度指令
```

## 5. 调试技巧

```bash
# 机器人 map 系位置（odom 和 map 可能差好几米！slam 重置会变）
ros2 run tf2_ros tf2_echo map base_footprint

# 复刻 rrt 的 find_frontiers 逻辑排查边界数（/tmp/replay_frontiers.py）

# 判断地图是否在更新（静止时 slam 不发布新图，会导致"没进展"假象）
ros2 topic echo /map --once --field data | wc -c
```

## 6. 多机器人

本包支持多机（`robot_count`），但多机需要 merged map 和命名空间 TF。单机场景也可用 workspace 里的 `mrtsp_exploration_ros2`（纯 Python，无需命名空间、自动启动），**两者不要同时运行**（都抢 Nav2 的 `navigate_to_pose`）。

# Frontier-based exploration (ROS 2)

ROS 2 (Humble) 版本的前沿探索包：在占用栅格地图上检测前沿、计算质心，并通过
Nav2 navigate_to_pose 动作服务器驱动机器人探索未知环境。原算法逻辑逐行保留，
仅把 ROS 1 API 迁移到 ROS 2。

> 本仓库同时保留了 ROS 1 的仿真资产（`launch/`、`config/`、`urdf/`、`world/`、`meshes/`）。
> 它们面向 Melodic/旧版 Gazebo 与 ROS 1 move_base，不参与 ROS 2 构建，仅作参考。

## 依赖

- ROS 2 Humble
- 构建期：`rclcpp`、`rclcpp_action`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、
  `geometry_msgs`、`nav_msgs`、`visualization_msgs`、`nav2_msgs`、`rosidl_default_generators`
- 运行期：提供 `nav2_msgs/action/NavigateToPose` 动作服务的导航栈（例如 Nav2）、
  SLAM 地图、`map -> base_link` 的 TF 树

```sh
sudo apt install ros-humble-navigation2       # 提供 nav2_msgs 与 Nav2 导航栈
```

## 构建

```sh
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select frontier_exploration
source install/setup.bash
```

## 运行

```sh
ros2 run frontier_exploration frontier_planner
```

运行时前置（由外部提供，与本包无关）：
1. `/map`（`nav_msgs/msg/OccupancyGrid`）发布者，例如 cartographer / slam_toolbox；
2. `map -> base_link`（或 `robot_base_frame` 参数指定）TF 树；
3. 提供 `nav2_msgs/action/NavigateToPose` 的动作服务器（topic `/navigate_to_pose`），例如 Nav2。

## 话题与参数

### 订阅
- `/map`（`nav_msgs/OccupancyGrid`）：全局地图

### 发布
- `/frontier`、`/centroids`（`frontier_exploration/PointArray`）：前沿单元与质心
- `/inflated_map`（`nav_msgs/OccupancyGrid`）：膨胀后的地图
- `/frontier_vis`、`/centroid_vis`、`/goal_vis`、`/home_vis`（`visualization_msgs/Marker`）：可视化
- `/cmd_vel`（`geometry_msgs/Twist`）：旋转指令

### 服务
- `get_centroids`（`frontier_exploration/GetCentroids`）：按需重新计算并返回质心

### 参数
- `obstacle_inflation`（float，默认 0.3）：膨胀半径（m）
- `map_revolution`（float，默认 0.1）：地图分辨率（m/cell）
- `cmd_topic`（string，默认 `cmd_vel`）：速度话题名
- `robot_base_frame`（string，默认 `base_link`）：机器人基座 frame
- `goal_tolerance`（float，默认 0.2）：到达目标的容差（m）
- `obstacle_tolerance`（float，默认 0.5）：距障碍物小于该值时不再原地旋转（m）
- `rotate_speed`（float，默认 0.5）：旋转角速度

## 说明与限制

- 节点启动时会等待 `/navigate_to_pose` 动作服务器与 TF，之后开始探索（与原版 ROS 1 行为一致）。
- 这是纯库移植，不包含 ROS 1 的 launch/Gazebo/URDF 仿真栈；如需完整仿真，请在 ROS 2 下
  自行用 Nav2、SLAM 与 Gazebo 搭建。
- 移植中修正的原始缺陷：
  - 服务名拼写错误 `get_centriods` → `get_centroids`；
  - 清理未使用的订阅/发布器与成员（原 `std_srvs` client、`move_base/goal`、`move_base/cancel` 等）；
  - 地图未到达时 `Rotation()` 与碰撞检测的除零/越界防护。

## 参考

- 原 ROS 1 版本：本仓库历史提交（`git log`）
- 设计文档：`docs/superpowers/specs/2026-08-24-frontier-exploration-ros2-port-design.md`

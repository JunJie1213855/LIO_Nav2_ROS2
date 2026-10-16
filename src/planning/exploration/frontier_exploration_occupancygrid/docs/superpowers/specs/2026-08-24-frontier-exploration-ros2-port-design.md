# Frontier Exploration 包 ROS2 移植设计

日期：2026-08-24
状态：已批准（2026-08-24，用户确认 MIT license + README 更新）

## 1. 背景与目标

将 ROS1 (Melodic/catkin) 的 `frontier_exploration` 包（目录 `frontier_exploration_occupancygrid`）移植为
ROS2 Humble (`ament_cmake`) 包。该包实现基于占用栅格的**单机器人前沿探索**：

- `FrontierDetector`：膨胀占用栅格地图 → 提取前沿(frontier)单元 → 分组 → 计算质心(centroid)。
- `Actuator`：按"距离+碰撞惩罚"代价函数选取目标 → 通过 move_base 动作服务器驱动机器人 → 到位后原地旋转 → 探索结束返航。
- `frontierMain.cpp`：主循环，串起上述两个类。

目标：核心 C++ 库 + 消息/服务接口 + 构建文件可移植、可编译、行为等价。

## 2. 决策记录

| 决策点 | 结论 |
|---|---|
| 转换范围 | **核心库 only**（C++、msg/srv、package.xml、CMakeLists、README） |
| 导航动作接口 | **`mbf_msgs/action/MoveBase`**（已安装的 move_base_flex，goal 同为 `target_pose`，是 move_base 的官方向后兼容替代品） |
| 许可证 | **MIT**（原 `package.xml` 为 `TODO`） |
| README | 更新为 ROS2 构建/运行说明，并标注 ROS1 仿真资产为 legacy |
| srv 类型名 | `srv/get_centroids.srv` → `srv/GetCentroids.srv`（ROS2 `rosidl` 强制接口类型名首字母大写；字段内容逐字不变，服务 topic 名仍为 `get_centroids`；C++ 生成头路径仍为 `srv/get_centroids.hpp`） |
| 类名/命名空间 | 保持原样（`FrontierDetector::`、`Actuator::`） |
| 主循环逻辑 | 1:1 保留 |

环境事实（已验证）：
- ROS2 Humble 已安装：`rclcpp`、`rclcpp_action`、`tf2/tf2_ros/tf2_geometry_msgs`、`geometry_msgs`、`nav_msgs`、`visualization_msgs`、`rosidl_default_generators`、`rosidl_default_runtime`、`mbf_msgs`(move_base_flex)。
- **未安装** `ros-humble-move-base-msgs`，且 apt 镜像中无该包；`nav2`、`xacro`、`gazebo_ros` 也未安装。
- 工作区根：`/home/ros/ros_ws/frontier_exploration_ws`，用 `colcon` 构建。

## 3. 现状分析（ROS1 依赖与接口）

源码使用到的 ROS1 API：

- `roscpp`：`ros::init`、`ros::NodeHandle`、advertise/subscribe/service、`ros::spinOnce`、`ros::ok()`、`ros::Duration`、`ros::Time`、`ros::param::param`、`ROS_INFO`。
- `tf`：`tf::TransformListener`、`tf::StampedTransform`、`tf::getYaw`、`tf::TransformException`。
- `actionlib` + `move_base_msgs`：`SimpleActionClient<MoveBaseAction>`（真实发目标的方式）；另有未使用的 `GoalPub`(move_base/goal)、`CancelgoalPub`(move_base/cancel)。
- 自定义接口：`msg/PointArray.msg`、`srv/get_centroids.srv`。
- `visualization_msgs/Marker`、`nav_msgs/OccupancyGrid`、`geometry_msgs/*`。

发现的原生 bug / 死代码：
1. 服务名拼写错误：server 广告为 `get_centriods`，client（`std_srvs::Empty`，从未被调用）却调用 `get_centroids`，类型与名字均不匹配。
2. `GoalPub`、`CancelgoalPub`、`path_pub`、`InflatedMapSub_`、`path_vis`、`inflated_map`、`CurrentGoal`、`req`/`res` 成员均声明未使用。
3. `Rotation()` 在地图到达前就读取 `raw_map.info.resolution`（默认 0 → 除零/越界）和 `raw_map.data`（空容器越界）。
4. `CancelAllGoals()` 后若目标状态转为 CANCELED，原主循环无此终态判断，可能死等。

## 4. 转换方案

### 4.1 目录布局

```
frontier_exploration_occupancygrid/
├── CMakeLists.txt                 # 重写：ament_cmake + rosidl
├── package.xml                    # 重写：format 3
├── README.md                      # 更新：ROS2 构建/运行
├── msg/PointArray.msg             # 不变
├── srv/get_centroids.srv          # 不变
├── include/frontier_exploration/
│   ├── frontier_detector.h        # 由 include/frontier_detector.h 移植
│   └── actuator.h                 # 由 include/actuator.h 移植
├── src/
│   ├── frontier_detector.cpp
│   ├── actuator.cpp
│   └── frontierMain.cpp
└── launch/ config/ urdf/ world/ meshes/   # 保留不动（ROS1 legacy，范围外）
```

### 4.2 API 映射表

| ROS1 | ROS2 |
|---|---|
| `ros::init(argc, argv, name)` | `rclcpp::init(argc, argv)` + `std::make_shared<rclcpp::Node>("frontier_planner")` |
| `ros::NodeHandle nh` | `rclcpp::Node::SharedPtr node`（传入两个类） |
| `nh.advertise<Msg>(t, q)` | `node->create_publisher<Msg>(t, q)` |
| `nh.subscribe(t, q, &C::cb, this)` | `node->create_subscription<Msg>(t, q, std::bind(&C::cb, this, _1))` |
| `nh.advertiseService(...)` / `serviceClient` | `node->create_service<...>` / `create_client<...>` |
| 服务回调返回 `bool` | 回调返回 `void`，直接写 `res->...`（SharedPtr） |
| `ros::spinOnce()` / `nh.ok()` | `rclcpp::spin_some(node)` / `rclcpp::ok()` |
| `ros::param::param<float>("~/x", v, d)` | `declare_parameter<float>("x", d)` + `get_parameter` |
| `ros::Time::now()` / `ros::Duration(0)` | `node->now()` / `rclcpp::Duration(0, 0)` |
| `ros::Duration(s).sleep()` | `rclcpp::sleep_for(std::chrono::seconds(s))` |
| `ROS_INFO(...)` | `RCLCPP_INFO(node_->get_logger(), ...)` |
| `tf::TransformListener` | `tf2_ros::Buffer` + `tf2_ros::TransformListener` |
| `tf::StampedTransform` / `.getOrigin()` / `.getRotation()` | `geometry_msgs::msg::TransformStamped` / `.transform.translation` / `.transform.rotation` |
| `tf::getYaw(q)` / `tf::TransformException` | `tf2::getYaw(q)`（经 `tf2::fromMsg`）/ `tf2::TransformException` |
| `actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>` | `rclcpp_action::Client<mbf_msgs::action::MoveBase>`（topic `move_base`） |
| `ac.sendGoal(goal)` | `ac->async_send_goal(goal, options)`（存 goal_handle_） |
| `ac.getState()` | `goal_handle_->get_status()`（`rclcpp_action::GoalStatus`） |
| `ac.cancelAllGoals()` / `ac.cancelGoal()` | `ac->async_cancel_all_goals()` / `async_cancel_goal(goal_handle_)` |
| `MoveBaseGoal.target_pose` | `mbf_msgs::action::MoveBase::Goal::target_pose` |

### 4.3 各文件改动

- **`package.xml`**：format 3，`<build_type>ament_cmake</build_type>`；依赖 `rclcpp`、`rclcpp_action`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`geometry_msgs`、`nav_msgs`、`visualization_msgs`、`mbf_msgs`；构建工具 `ament_cmake`、`rosidl_default_generators`；`member_of_group` `rosidl_interface_packages`；`exec_depend` `rosidl_default_runtime`；license `MIT`。
- **`CMakeLists.txt`**：`ament_cmake` + `rosidl_generate_interfaces`（生成 `PointArray`、`get_centroids`，依赖 `geometry_msgs`）；`add_library(frontierDetector ...)`；`add_executable(frontier_planner ...)`；`target_link_libraries(frontier_planner frontierDetector ${PROJECT_NAME})`；`install()` 规则；`ament_package()`。
- **`frontier_detector.h/.cpp`**：类型与 API 映射；构造函数接收 `rclcpp::Node::SharedPtr`；publisher/subscriber/service 为 SharedPtr；Marker 枚举改用 `visualization_msgs::msg::Marker::POINTS/ADD`；删除未用的 `CentriodClient_`（std_srvs::Empty）与 `req`/`res` 成员；服务名修正为 `get_centroids`。
- **`actuator.h/.cpp`**：TF 改为 `tf2_ros::Buffer` + `TransformListener` 成员；动作客户端 `rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr ac_` + `goal_handle_`；新增 `CancelAllGoals()` 与 `GetGoalStatus()`；删除 `GoalPub`/`CancelgoalPub`/`path_pub`/`path_vis`/`InflatedMapSub_`/`inflated_map` 等死成员；`Rotation()` 增加空地图保护。
- **`frontierMain.cpp`**：`rclcpp` 初始化与单例节点；`spin_some` 替代 `spinOnce`；目标状态循环改用 `GetGoalStatus()` 与 `rclcpp_action::GoalStatus` 常量（SUCCEEDED/LOST/ABORTED，另加 CANCELED 防死等）；`rclcpp::shutdown()` 收尾。
- **`README.md`**：ROS2 构建（`colcon build`）、运行（`ros2 run frontier_exploration frontier_planner`）、话题/参数表、运行时前置（`/map` 发布、`map→base_link` TF、提供 `mbf_msgs/action/MoveBase` 的动作服务器如 move_base_flex）、ROS1 资产标记为 legacy。

### 4.4 需要额外确认的运行时前提（不改动）

本包只负责"发现前沿 → 发目标"，机器人环境（SLAM 地图、TF、move_base/MBF 动作服务器）需由外部提供，与 ROS1 版本一致。

## 5. 验证方案

1. `colcon build`（在 `frontier_exploration_ws` 下）编译通过、零告警目标（-Wall -Wextra）。
2. `source install/setup.bash` 后：
   - `ros2 interface show frontier_exploration/msg/PointArray`
   - `ros2 interface show frontier_exploration/srv/get_centroids`
   - `ros2 pkg executables frontier_exploration` → 应列出 `frontier_planner`
   - `ros2 pkg prefix mbf_msgs` 确认依赖存在
3. 运行时冒烟测试**无法独立完成**：节点构造即阻塞等待 move_base/MBF 动作服务器与 TF（与原版行为一致），需要真实/仿真环境。此限制写入 README。

## 6. 范围外（明确不转换）

- `launch/`（ROS1 XML launch，gmapping/cartographer/move_base/gazebo）
- `config/`（move_base 的 ROS1 参数）
- `urdf/`、`world/`、`meshes/`（ROS1 xacro/gazebo 资产）
- Nav2、slam_toolbox、Gazebo 集成

这些目录保留在仓库中，仅作为 legacy 参考，不参与 ROS2 构建。

# Frontier Exploration ROS2 移植 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 ROS1 (catkin) 的 `frontier_exploration` 包（核心 C++ 库 + msg/srv + 构建文件）移植为 ROS2 Humble (`ament_cmake`) 可编译包，行为与原版等价。

**Architecture:** 保持 `FrontierDetector` / `Actuator` 两个类与 `frontierMain.cpp` 主循环结构不变，逐行把 ROS1 API 换成 ROS2 等价物（`rclcpp` / `tf2_ros` / `rclcpp_action`）。构建文件按任务渐进式扩展，保证每个任务结束时包都能通过 `colcon build`。导航目标通过 `mbf_msgs/action/MoveBase`（move_base_flex，已安装）动作客户端发送。

**Tech Stack:** ROS2 Humble（rclcpp、rclcpp_action、tf2/tf2_ros/tf2_geometry_msgs）、mbf_msgs、ament_cmake、rosidl、C++17。

**设计文档:** `docs/superpowers/specs/2026-08-24-frontier-exploration-ros2-port-design.md`（已提交 `9b7cb8b`）。

## Global Constraints

- 目标发行版：**ROS2 Humble**；构建工具：**colcon**；构建系统：**ament_cmake**；C++17。
- 包名保持 **`frontier_exploration`**；节点名 **`frontier_planner`**。
- 类名 / 命名空间保持原样：`FrontierDetector::FrontierDetector`、`Actuator::Actuator`。
- 自定义接口**内容**不改：`msg/PointArray.msg`、`srv/get_centroids.srv`（仅按 ROS2 强制要求把类型名重命名为 `GetCentroids.srv`，首字母大写；topic 名仍为 `get_centroids`，字段内容逐字不变）。
- 依赖白名单（package.xml 与 CMakeLists 只允许这些）：`rclcpp`、`rclcpp_action`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`geometry_msgs`、`nav_msgs`、`visualization_msgs`、`mbf_msgs`、`rosidl_default_generators`（构建）、`rosidl_default_runtime`（运行）、`ament_cmake`（构建工具）。
- **禁止**在 `src/`、`include/`、`package.xml`、`CMakeLists.txt` 中出现任何 ROS1 API 残留：`ros/ros.h`、`ros::`、`tf::`、`actionlib`、`move_base_msgs`、`catkin`、`add_message_files`、`generate_messages`、`roscpp`。
- license = **MIT**；`package.xml` 写 `format="3"`。
- **不修改** `launch/`、`config/`、`urdf/`、`world/`、`meshes/`（ROS1 legacy，范围外）。
- 编译标志：`-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter`（移植旧代码，容忍符号比较/未用参数告警）。
- 行为保留：前沿检测、质心计算、代价选点、碰撞检测、返航等**算法逻辑逐行不变**，只改 API 与类型。
- 所有构建命令在 `/home/ros/ros_ws/frontier_exploration_ws` 下执行，且每次先 `source /opt/ros/humble/setup.bash`。
- 提交时只 `git add` 本次任务涉及的文件，不提交他人/历史遗留改动（如已有的 `CMakeLists.txt` 未提交改动会被本次重写覆盖）。

---

## Task 1: 构建脚手架 + 接口 + 两个新头文件

**Files:**
- Overwrite: `package.xml`
- Overwrite: `CMakeLists.txt`（第一阶段：只含 ament 依赖 + 接口生成）
- Create: `include/frontier_exploration/frontier_detector.h`
- Create: `include/frontier_exploration/actuator.h`

**Interfaces:**
- Produces: `package.xml`(format 3, MIT)、`CMakeLists.txt`(ament + rosidl)、两个可被 Task2/3 编译的声明头；接口类型 `frontier_exploration/msg/PointArray`、`frontier_exploration/srv/get_centroids`。
- Consumes: 无（绿色起步）。

- [ ] **Step 1: 重写 `package.xml`**

用以下内容完全替换 `/home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid/package.xml`：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>frontier_exploration</name>
  <version>0.1.0</version>
  <description>
    ROS 2 port of a frontier-based exploration package: detects frontiers on an
    occupancy grid, computes centroids, and drives a robot to goals via the
    move_base (move_base_flex) action server.
  </description>
  <maintainer email="sdu@todo.todo">sdu</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>mbf_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: 重写 `CMakeLists.txt`（第一阶段）**

用以下内容完全替换 `/home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid/CMakeLists.txt`（注意：这会覆盖工作区里那份"catkin+ament 混杂、含 livox_ros_driver2"的未提交改动，属预期行为）：

```cmake
cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  "srv/GetCentroids.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
```

- [ ] **Step 3: 创建 `include/frontier_exploration/frontier_detector.h`**

创建目录 `include/frontier_exploration/` 并写入以下完整内容：

```cpp
#ifndef FRONTIER_EXPLORATION__FRONTIER_DETECTOR_HPP_
#define FRONTIER_EXPLORATION__FRONTIER_DETECTOR_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "frontier_exploration/msg/point_array.hpp"
#include "frontier_exploration/srv/get_centroids.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace FrontierDetector
{

struct Header_param
{
  std::string frame_id = "map";
  rclcpp::Time stamp = rclcpp::Clock().now();
};

class FrontierDetector
{
private:
  rclcpp::Node::SharedPtr node_;

  float OBSTABLE_INFLATION;  // inflate obstacles in meters
  float MapRevolution;       // map resolution (m/cell)
  nav_msgs::msg::OccupancyGrid raw_map;
  frontier_exploration::msg::PointArray Frontier;
  frontier_exploration::msg::PointArray Centroids;
  visualization_msgs::msg::Marker frontier_vis;
  visualization_msgs::msg::Marker centroid_vis;
  std::vector<geometry_msgs::msg::Point> raw_centroids;

  std::vector<geometry_msgs::msg::Point> pointGroup;    // frontier group
  std::vector<geometry_msgs::msg::Point> frontierClose; // checked-over frontiers
  Header_param header;

  bool CheckCollision(const nav_msgs::msg::OccupancyGrid & map,
                      geometry_msgs::msg::Point & start, geometry_msgs::msg::Point & end);

public:
  std::vector<geometry_msgs::msg::Point> centroids;
  std::vector<geometry_msgs::msg::Point> frontier;
  nav_msgs::msg::OccupancyGrid inflated_map;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontierMarker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centroidMarker_;
  rclcpp::Publisher<frontier_exploration::msg::PointArray>::SharedPtr FrontierPub_;
  rclcpp::Publisher<frontier_exploration::msg::PointArray>::SharedPtr CentroidsPub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflatedMapPub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr MapSub_;

  rclcpp::Service<frontier_exploration::srv::GetCentroids>::SharedPtr CentriodServer_;

  FrontierDetector(const rclcpp::Node::SharedPtr & node);
  ~FrontierDetector();

  int GridValue(nav_msgs::msg::OccupancyGrid & map, geometry_msgs::msg::Point & x1);
  int CheckNeibor(nav_msgs::msg::OccupancyGrid & inflated_map, long & index);

  bool InflateMap(nav_msgs::msg::OccupancyGrid & raw_map, nav_msgs::msg::OccupancyGrid & inflated_map);
  bool ComputeFrontier(nav_msgs::msg::OccupancyGrid & Inflated_map);
  bool ComputeCentroids(nav_msgs::msg::OccupancyGrid & inflated_map,
                        std::vector<geometry_msgs::msg::Point> & frontiers);
  void InitDetector();
  void InitVis();
  void Visualization();
  void Grouping(nav_msgs::msg::OccupancyGrid & inflated_map, geometry_msgs::msg::Point & point);
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr raw_map);
  void centroidCallback(const frontier_exploration::srv::GetCentroids::Request::SharedPtr req,
                        frontier_exploration::srv::GetCentroids::Response::SharedPtr res);

  std::vector<geometry_msgs::msg::Point> Sort(nav_msgs::msg::OccupancyGrid & inflated_map,
                                              std::vector<geometry_msgs::msg::Point> & pts);
};

}  // namespace FrontierDetector

#endif  // FRONTIER_EXPLORATION__FRONTIER_DETECTOR_HPP_
```

- [ ] **Step 4: 创建 `include/frontier_exploration/actuator.h`**

写入以下完整内容：

```cpp
#ifndef FRONTIER_EXPLORATION__ACTUATOR_HPP_
#define FRONTIER_EXPLORATION__ACTUATOR_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "frontier_exploration/msg/point_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mbf_msgs/action/move_base.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

const double PI = 3.1415926535897932385;

namespace Actuator
{

using MoveBaseAction = mbf_msgs::action::MoveBase;
using MoveBaseClient = rclcpp_action::Client<MoveBaseAction>;
using MoveBaseGoalHandle = rclcpp_action::ClientGoalHandle<MoveBaseAction>;

struct Header_param
{
  std::string frame_id = "map";
  rclcpp::Time stamp = rclcpp::Clock().now();
};

struct RobotPose
{
  geometry_msgs::msg::Point Position;
  double Yaw;
};

// Actuator: select a goal from frontiers and drive the robot via the
// move_base (move_base_flex) action server.
class Actuator
{
private:
  rclcpp::Node::SharedPtr node_;

  std::string CmdTopic;
  std::string RobotBase;
  float GoalTolerance;     // abandon next goal close to an explored point
  float ObstacleTolerance; // do not rotate when close to an obstacle
  float RotateSpeed;
  long iteration;

  Header_param header;
  std::vector<geometry_msgs::msg::Point> centroids;
  geometry_msgs::msg::Point Home;  // the mapping origin
  visualization_msgs::msg::Marker GoalMarker;
  visualization_msgs::msg::Marker HomeMarker;
  nav_msgs::msg::OccupancyGrid raw_map;

  geometry_msgs::msg::Twist RotSpeed;
  RobotPose robotPose;  // robot pose through TF

  mbf_msgs::action::MoveBase::Goal MoveGoal;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_vis;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr home_vis;
  rclcpp::Subscription<frontier_exploration::msg::PointArray>::SharedPtr centroidsSub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr RawMapSub_;

  MoveBaseClient::SharedPtr ac_;
  MoveBaseGoalHandle::SharedPtr goal_handle_;

public:
  geometry_msgs::msg::Point Goal;
  std::vector<geometry_msgs::msg::Point> GoalClose;  // traveled / unavailable points
  int GoHomeFlag;

  Actuator(const rclcpp::Node::SharedPtr & node);
  ~Actuator();

  geometry_msgs::msg::Point SelectGoal(std::vector<geometry_msgs::msg::Point> & centroids);
  void ObtainPose();
  void MoveToGoal();
  void CancelGoal();
  void CancelAllGoals();
  void Rotation(float angle);
  void ReturnHome();
  void Visualization();
  void VisInit();
  void ActuatorInit();
  void AddToClose(geometry_msgs::msg::Point & goal);
  void centroidCallback(frontier_exploration::msg::PointArray::SharedPtr msg);
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr RawMap);

  rclcpp_action::GoalStatus GetGoalStatus() const;
  int CheckCollision(const nav_msgs::msg::OccupancyGrid & map,
                     geometry_msgs::msg::Point & start, geometry_msgs::msg::Point & end);
};

}  // namespace Actuator

#endif  // FRONTIER_EXPLORATION__ACTUATOR_HPP_
```

- [ ] **Step 5: 构建并验证第一阶段**

运行（在 `/home/ros/ros_ws/frontier_exploration_ws` 下）：

```bash
source /opt/ros/humble/setup.bash && cd /home/ros/ros_ws/frontier_exploration_ws && colcon build --packages-select frontier_exploration
```

预期：构建成功（此时只生成接口 + 元数据，尚无编译目标）。然后验证接口可被工具链识别：

```bash
source install/setup.bash && ros2 interface show frontier_exploration/msg/PointArray && ros2 interface show frontier_exploration/srv/GetCentroids
```

预期输出分别为 `geometry_msgs/Point[] points` 与 `geometry_msgs/Point[] frontiers --- geometry_msgs/Point[] centroids`。

- [ ] **Step 6: 提交**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && git add package.xml CMakeLists.txt include/frontier_exploration/frontier_detector.h include/frontier_exploration/actuator.h && git commit -m "chore: ROS2 build scaffold, interfaces, and ported headers"
```

---

## Task 2: 移植 `frontier_detector.cpp` 并建立库目标

**Files:**
- Modify: `src/frontier_detector.cpp`（整体重写为 ROS2 版本）
- Modify: `CMakeLists.txt`（加入 `add_library(frontierDetector ...)`，只含 `src/frontier_detector.cpp`）

**Interfaces:**
- Consumes: Task 1 的 `frontier_exploration::msg::PointArray`、`srv::get_centroids`、头文件 `frontier_detector.h`。
- Produces: 可编译的库目标 `frontierDetector`（Task 3 向其中追加 `actuator.cpp`，Task 4 链接）。

- [ ] **Step 1: 重写 `src/frontier_detector.cpp`**

用以下完整内容替换 `/home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid/src/frontier_detector.cpp`：

```cpp
#include "frontier_exploration/frontier_detector.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

int FrontierDetector::FrontierDetector::GridValue(nav_msgs::msg::OccupancyGrid & map,
                                                  geometry_msgs::msg::Point & x1)
{
  // guard: no map yet -> treat as free (main loop calls this before the first /map)
  if (map.data.empty() || map.info.resolution <= 0.0f) {
    return 0;
  }
  float Xoriginx = map.info.origin.position.x;
  float Xoriginy = map.info.origin.position.y;
  int index;
  index = int((x1.y + std::fabs(Xoriginy)) / map.info.resolution) * map.info.width +
          int((x1.x + std::fabs(Xoriginx)) / map.info.resolution);
  int out = map.data[index];
  return out;
}

int FrontierDetector::FrontierDetector::CheckNeibor(nav_msgs::msg::OccupancyGrid & inflated_map,
                                                    long & index)
{
  int x = index % inflated_map.info.width;
  int y = index / inflated_map.info.width;
  int flag = 0;
  for (int i = x - 1; i < x + 2; i++) {
    if (flag == 1) { break; }
    for (int j = y - 1; j < y + 2; j++) {
      if (i == x && j == y) { continue; }
      if (i - 1 < 0 || i + 1 > inflated_map.info.width || j - 1 < 0 || j + 1 > inflated_map.info.height) {
        continue;
      }
      if (inflated_map.data[j * inflated_map.info.width + i] >= 35 &&
            inflated_map.data[j * inflated_map.info.width + i] < 65 ||
            inflated_map.data[j * inflated_map.info.width + i] == -1) {
        // neighbor cell is unknown -> frontier
        flag = 1;
        break;
      }
    }
  }
  return flag;
}

void FrontierDetector::FrontierDetector::InitVis()
{
  // frontier is blue
  frontier_vis.header.frame_id = header.frame_id;
  frontier_vis.header.stamp = header.stamp;
  frontier_vis.ns = "frontier";
  frontier_vis.id = 0;
  frontier_vis.lifetime = rclcpp::Duration(0, 0);
  frontier_vis.type = visualization_msgs::msg::Marker::POINTS;
  frontier_vis.action = visualization_msgs::msg::Marker::ADD;
  frontier_vis.color.a = 0.7;
  frontier_vis.color.r = 0.0;
  frontier_vis.color.g = 0.0;
  frontier_vis.color.b = 1.0;
  frontier_vis.scale.x = MapRevolution;
  frontier_vis.scale.y = MapRevolution;
  frontier_vis.scale.z = 0.0;
  frontier_vis.pose.orientation.w = 1.0;
  frontier_vis.points.clear();

  // centroids are red
  centroid_vis.header.frame_id = header.frame_id;
  centroid_vis.header.stamp = header.stamp;
  centroid_vis.ns = "centroids";
  centroid_vis.id = 1;
  centroid_vis.lifetime = rclcpp::Duration(0, 0);
  centroid_vis.type = visualization_msgs::msg::Marker::POINTS;
  centroid_vis.action = visualization_msgs::msg::Marker::ADD;
  centroid_vis.color.a = 1.0;
  centroid_vis.color.r = 1.0;
  centroid_vis.color.g = 0.0;
  centroid_vis.color.b = 0.0;
  centroid_vis.scale.x = MapRevolution * 3;
  centroid_vis.scale.y = MapRevolution * 3;
  centroid_vis.scale.z = 0.0;
  centroid_vis.pose.orientation.w = 1.0;
  centroid_vis.points.clear();
}

void FrontierDetector::FrontierDetector::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr raw_map)
{
  FrontierDetector::raw_map.header.frame_id = "inflated_map";
  FrontierDetector::raw_map.header.stamp = header.stamp;
  FrontierDetector::raw_map.info = raw_map->info;
  FrontierDetector::raw_map.data = raw_map->data;
  // inflate the grid map
  InflateMap(FrontierDetector::raw_map, FrontierDetector::inflated_map);
  // get all frontiers
  ComputeFrontier(inflated_map);
  Frontier.points = frontier;
  Centroids.points = centroids;
  FrontierPub_->publish(Frontier);
  if (Centroids.points.size() != 0) { CentroidsPub_->publish(Centroids); }
  Visualization();
}

void FrontierDetector::FrontierDetector::centroidCallback(
  const frontier_exploration::srv::GetCentroids::Request::SharedPtr req,
  frontier_exploration::srv::GetCentroids::Response::SharedPtr res)
{
  (void)req;
  ComputeCentroids(inflated_map, frontier);
  res->centroids = centroids;
  std::cout << "Centroids computing completely!" << std::endl;
}

void FrontierDetector::FrontierDetector::Visualization()
{
  if (FrontierDetector::frontier.empty()) {
    std::cout << "Frontier no found! Waiting......" << std::endl;
  }
  if (FrontierDetector::centroids.empty()) {
    std::cout << "Computing the goal! Waiting....." << std::endl;
  }
  if (frontier.size() == 0 || centroids.size() == 0) {
    // avoid node dying if no frontier or centroid found
    FrontierDetector::frontierMarker_->publish(FrontierDetector::frontier_vis);
    FrontierDetector::centroidMarker_->publish(FrontierDetector::centroid_vis);
  } else {
    FrontierDetector::frontier_vis.points.clear();
    FrontierDetector::centroid_vis.points.clear();
    FrontierDetector::frontier_vis.points = FrontierDetector::frontier;
    FrontierDetector::centroid_vis.points = FrontierDetector::centroids;
    FrontierDetector::frontierMarker_->publish(FrontierDetector::frontier_vis);
    FrontierDetector::centroidMarker_->publish(FrontierDetector::centroid_vis);
  }
}

void FrontierDetector::FrontierDetector::InitDetector()
{
  FrontierDetector::frontier.clear();
  FrontierDetector::centroids.clear();
  InitVis();
  inflated_map.data.clear();

  // parameters delivered via the parameter server
  node_->declare_parameter<float>("obstacle_inflation", 0.3);
  node_->declare_parameter<float>("map_revolution", 0.1);
  node_->get_parameter("obstacle_inflation", this->OBSTABLE_INFLATION);
  node_->get_parameter("map_revolution", this->MapRevolution);
}

bool FrontierDetector::FrontierDetector::InflateMap(nav_msgs::msg::OccupancyGrid & raw_map,
                                                    nav_msgs::msg::OccupancyGrid & inflated_map)
{
  inflated_map = raw_map;
  int dilate_amount = round(OBSTABLE_INFLATION / raw_map.info.resolution);
  for (int x = 0; x < raw_map.info.width; x++) {
    for (int y = 0; y < raw_map.info.height; y++) {
      if (raw_map.data[raw_map.info.width * y + x] < 65 &&
          raw_map.data[raw_map.info.width * y + x] >= 0) {
        inflated_map.data[raw_map.info.width * y + x] = 0;  // free cell, continue
        continue;
      }
      if (raw_map.data[raw_map.info.width * y + x] == -1) { continue; }
      for (int i = -dilate_amount; i <= dilate_amount; i++) {
        for (int j = -dilate_amount; j <= dilate_amount; j++) {
          int x_d = x + i;
          int y_d = y + j;
          if (x_d < 0 || x_d > raw_map.info.width - 1 || y_d < 0 || y_d > raw_map.info.height - 1) {
            continue;
          }
          inflated_map.data[raw_map.info.width * y_d + x_d] = 100;  // inflate with obstacle cells
        }
      }
    }
  }
  inflatedMapPub_->publish(inflated_map);
  return true;
}

bool FrontierDetector::FrontierDetector::ComputeFrontier(nav_msgs::msg::OccupancyGrid & inflated_map)
{
  FrontierDetector::frontier.clear();
  geometry_msgs::msg::Point p;
  if (!inflated_map.data.empty()) {
    for (long n = 0; n < inflated_map.data.size(); n++) {
      // if the cell is free and a neighbor is unknown, it is a frontier cell
      if (inflated_map.data[n] >= 0 && inflated_map.data[n] < 35 && CheckNeibor(inflated_map, n) == 1) {
        int n_x = n % inflated_map.info.width;
        int n_y = n / inflated_map.info.width;
        p.x = (n_x + 0.5) * inflated_map.info.resolution - std::fabs(inflated_map.info.origin.position.x);
        p.y = (n_y + 0.5) * inflated_map.info.resolution - std::fabs(inflated_map.info.origin.position.y);
        p.z = 0.0;
        FrontierDetector::frontier.push_back(p);
      }
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "map data isn't received!");
    return false;
  }
  return true;
}

bool FrontierDetector::FrontierDetector::ComputeCentroids(
  nav_msgs::msg::OccupancyGrid & inflated_map, std::vector<geometry_msgs::msg::Point> & frontiers)
{
  FrontierDetector::centroids.clear();
  FrontierDetector::raw_centroids.clear();
  pointGroup.clear();
  if (frontiers.size() == 0) {
    std::cout << "Cannot find any frontiers! Checking!!" << std::endl;
    return false;
  }
  for (int i = 0; i < frontiers.size(); i++) {
    if (frontierClose.size() > 1) {
      if (std::find(frontierClose.begin(), frontierClose.end(), frontiers[i]) != frontierClose.end()) {
        continue;
      }
    }
    pointGroup.push_back(frontiers[i]);
    frontierClose.push_back(frontiers[i]);
    // find frontier groups; every group is disconnected from each other
    Grouping(inflated_map, frontiers[i]);
    pointGroup = Sort(inflated_map, pointGroup);  // sort based on map-image index
    if (pointGroup.size() <= 6) {  // avoid too-short frontiers
      pointGroup.clear();
      continue;
    }
    int index = int(ceil(pointGroup.size() / 2));
    FrontierDetector::raw_centroids.push_back(pointGroup[index]);
    pointGroup.clear();
  }
  frontierClose.clear();

  std::vector<int> pop_index;
  // raw centroid filter: remove one of a too-close, collision-free centroid pair
  if (raw_centroids.size() >= 2) {
    for (int m = 0; m < raw_centroids.size(); m++) {
      for (int n = raw_centroids.size(); n > m; n--) {
        if (!pop_index.empty()) {
          if (std::find(pop_index.begin(), pop_index.end(), n) != pop_index.end() ||
              std::find(pop_index.begin(), pop_index.end(), m) != pop_index.end()) {
            continue;
          }
        }
        float distance = sqrt(pow((raw_centroids[m].x - raw_centroids[n].x), 2) +
                              pow((raw_centroids[m].y - raw_centroids[n].y), 2));
        if (distance < 3 && CheckCollision(raw_map, raw_centroids[m], raw_centroids[n])) {
          if (!pop_index.empty()) {
            if (std::find(pop_index.begin(), pop_index.end(), n) == pop_index.end()) {
              pop_index.push_back(n);
            }
          } else {
            pop_index.push_back(n);
          }
        }
      }
    }
    for (int num = 0; num < raw_centroids.size(); num++) {
      if (std::find(pop_index.begin(), pop_index.end(), num) == pop_index.end()) {
        centroids.push_back(raw_centroids[num]);
      }
    }
    std::cout << "pop_index: " << pop_index.size() << std::endl;
    std::cout << "raw_centroids: " << raw_centroids.size() << std::endl;
    std::cout << "centroids: " << centroids.size() << std::endl;
    raw_centroids.clear();
    pop_index.clear();
  }
  return true;
}

// lazy collision check
bool FrontierDetector::FrontierDetector::CheckCollision(
  const nav_msgs::msg::OccupancyGrid & map, geometry_msgs::msg::Point & start, geometry_msgs::msg::Point & end)
{
  float length = sqrt(pow((start.x - end.x), 2) + pow((start.y - end.y), 2));
  float COS_THETA = (end.x - start.x) / length;
  float SIN_THETA = (end.y - start.y) / length;
  float resolution = map.info.resolution;
  float STEP = resolution;
  int count = 0;

  float x_check = start.x;
  float y_check = start.y;
  while (fabs(x_check - end.x) > STEP && fabs(y_check - end.y) > STEP) {
    int x_check_world = (x_check + fabs(map.info.origin.position.x)) / map.info.resolution;
    int y_check_world = (y_check + fabs(map.info.origin.position.y)) / map.info.resolution;
    if (map.data[x_check_world + (y_check_world * map.info.width)] >= 70) {
      count++;
    }
    x_check += STEP * COS_THETA;
    y_check += STEP * SIN_THETA;
    if (count > 2) { return false; }
  }
  return true;
}

void FrontierDetector::FrontierDetector::Grouping(nav_msgs::msg::OccupancyGrid & inflated_map,
                                                  geometry_msgs::msg::Point & point)
{
  int out = 0;
  geometry_msgs::msg::Point temp;
  geometry_msgs::msg::Point worldPoint;
  // transform the point to world(image) frame
  worldPoint.x = (point.x + std::fabs(inflated_map.info.origin.position.x)) / inflated_map.info.resolution - 0.5;
  worldPoint.y = (point.y + std::fabs(inflated_map.info.origin.position.y)) / inflated_map.info.resolution - 0.5;
  worldPoint.z = 0.0;

  if (!frontier.empty()) {
    for (float i = worldPoint.x - 1; i <= worldPoint.x + 1; i++) {
      if (out == 1) { break; }
      for (float j = worldPoint.y - 1; j <= worldPoint.y + 1; j++) {
        int index;
        if (i == worldPoint.x && j == worldPoint.y) { continue; }
        temp.x = (i + 0.5) * inflated_map.info.resolution - std::fabs(inflated_map.info.origin.position.x);
        temp.y = (j + 0.5) * inflated_map.info.resolution - std::fabs(inflated_map.info.origin.position.y);
        temp.z = 0.0;  // temp is in map frame

        if (std::find(frontier.begin(), frontier.end(), temp) != frontier.end()) {
          if (frontierClose.size() > 1) {
            if (std::find(frontierClose.begin(), frontierClose.end(), temp) != frontierClose.end()) {
              continue;
            }
          }
          auto itera = std::find(frontier.begin(), frontier.end(), temp);
          index = std::distance(frontier.begin(), itera);
          pointGroup.push_back(frontier[index]);
          frontierClose.push_back(frontier[index]);
          Grouping(inflated_map, frontier[index]);  // recursive
          out = 1;
          break;
        }
      }
    }
  }
}

std::vector<geometry_msgs::msg::Point> FrontierDetector::FrontierDetector::Sort(
  nav_msgs::msg::OccupancyGrid & inflated_map, std::vector<geometry_msgs::msg::Point> & pts)
{
  std::vector<geometry_msgs::msg::Point> outcome;
  outcome = pts;
  geometry_msgs::msg::Point p;
  if (!pts.empty()) {
    p = pts[0];
    for (int i = 0; i < outcome.size() - 1; i++) {
      for (int j = 0; j < outcome.size() - 1 - i; j++) {
        // sort based on image index
        if ((outcome[j].y * inflated_map.info.width + outcome[j].x) <
            (outcome[j + 1].y * inflated_map.info.width + outcome[j + 1].x)) {
          p = outcome[j];
          outcome[j] = outcome[j + 1];
          outcome[j + 1] = p;
        }
      }
    }
  } else {
    std::cout << "Group is empty!!" << std::endl;
  }
  return outcome;
}

FrontierDetector::FrontierDetector::FrontierDetector(const rclcpp::Node::SharedPtr & node)
: node_(node),
  frontierMarker_(node->create_publisher<visualization_msgs::msg::Marker>("frontier_vis", 1000)),
  centroidMarker_(node->create_publisher<visualization_msgs::msg::Marker>("centroid_vis", 1000)),
  FrontierPub_(node->create_publisher<frontier_exploration::msg::PointArray>("frontier", 1000)),
  CentroidsPub_(node->create_publisher<frontier_exploration::msg::PointArray>("centroids", 1000)),
  inflatedMapPub_(node->create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_map", 1000)),
  MapSub_(node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 10, std::bind(&FrontierDetector::mapCallback, this, std::placeholders::_1))),
  CentriodServer_(node->create_service<frontier_exploration::srv::GetCentroids>(
    "get_centroids",
    std::bind(&FrontierDetector::centroidCallback, this, std::placeholders::_1, std::placeholders::_2)))
{
  InitDetector();
  InitVis();
  std::cout << "Planner is Ready!!!" << std::endl;
}

FrontierDetector::FrontierDetector::~FrontierDetector()
{
  FrontierDetector::frontier.clear();
  FrontierDetector::centroids.clear();
  InitVis();
  inflated_map.data.clear();
}
```

- [ ] **Step 2: 更新 `CMakeLists.txt` —— 加入库目标**

在 `rosidl_generate_interfaces(...)` 之后、`ament_package()` 之前插入：

```cmake
add_library(frontierDetector
  src/frontier_detector.cpp
)
target_include_directories(frontierDetector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frontierDetector
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  mbf_msgs
)
# ${PROJECT_NAME} is a UTILITY target in Humble's rosidl_generate_interfaces;
# link the real typesupport target so generated headers + symbols resolve.
target_link_libraries(frontierDetector ${PROJECT_NAME}__rosidl_typesupport_cpp)
```

- [ ] **Step 3: 构建验证**

```bash
source /opt/ros/humble/setup.bash && cd /home/ros/ros_ws/frontier_exploration_ws && colcon build --packages-select frontier_exploration
```

预期：`frontierDetector` 编译通过，无编译错误（可能有个别 `-Wsign-compare` 类告警，属预期）。

- [ ] **Step 4: 提交**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && git add CMakeLists.txt src/frontier_detector.cpp && git commit -m "feat: port frontier_detector to rclcpp"
```

---

## Task 3: 移植 `actuator.cpp`

**Files:**
- Modify: `src/actuator.cpp`（整体重写为 ROS2 版本，TF + rclcpp_action）
- Modify: `CMakeLists.txt`（把 `src/actuator.cpp` 加进 `frontierDetector` 库）

**Interfaces:**
- Consumes: Task 1 头文件 `actuator.h`、Task 2 的库目标 `frontierDetector`、`mbf_msgs::action::MoveBase`。
- Produces: `Actuator::GetGoalStatus()`（返回 `rclcpp_action::GoalStatus`）、`CancelAllGoals()`、`MoveToGoal()`，供 Task 4 主循环使用。

- [ ] **Step 1: 重写 `src/actuator.cpp`**

用以下完整内容替换 `/home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid/src/actuator.cpp`：

```cpp
#include "frontier_exploration/actuator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <functional>
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"

void Actuator::Actuator::centroidCallback(frontier_exploration::msg::PointArray::SharedPtr msg)
{
  centroids = msg->points;
  Visualization();
}

void Actuator::Actuator::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr RawMap)
{
  raw_map = *RawMap;
}

void Actuator::Actuator::Rotation(float angle)
{
  // guard: original read raw_map.data / resolution out-of-bounds before a map arrived
  if (raw_map.data.empty() || raw_map.info.resolution <= 0.0f) {
    RCLCPP_WARN(node_->get_logger(), "Rotation skipped: no map received yet (raw_map empty).");
    return;
  }

  ObtainPose();  // obtain the robot's current pose
  // frame transformation: map -> image
  int world_x = (robotPose.Position.x + fabs(raw_map.info.origin.position.x)) / raw_map.info.resolution;
  int world_y = (robotPose.Position.y + fabs(raw_map.info.origin.position.y)) / raw_map.info.resolution;
  int CheckXmin = int(world_x - round(ObstacleTolerance / raw_map.info.resolution)),
      CheckXmax = int(world_x + round(ObstacleTolerance / raw_map.info.resolution));
  int CheckYmin = int(world_y - round(ObstacleTolerance / raw_map.info.resolution)),
      CheckYmax = int(world_y + round(ObstacleTolerance / raw_map.info.resolution));
  // check neighbors
  for (int x = CheckXmin; x <= CheckXmax; x++) {
    for (int y = CheckYmin; y <= CheckYmax; y++) {
      if (x < 0 || y < 0 || x > raw_map.info.width || y > raw_map.info.height) { continue; }
      if (raw_map.data[x + (y * raw_map.info.width)] >= 70) {
        std::cout << "Position close to obstacle. Cannot rotate" << std::endl;
        return;
      }
    }
  }

  double rotated_angle = 0.0;  // rotated angle
  time_t initTime, currTime;
  time_t duration = 0L;
  time_t limitation = 15L;  // rotate within 15 seconds
  time(&initTime);

  while ((rotated_angle < angle) && (rclcpp::ok()) && bool(duration < limitation)) {
    time(&currTime);
    duration = currTime - initTime;
    double old_yaw = robotPose.Yaw;
    cmdPub->publish(RotSpeed);
    ObtainPose();
    rclcpp::spin_some(node_);
    double dYaw = robotPose.Yaw - old_yaw;
    if ((robotPose.Yaw >= 0 && robotPose.Yaw < 350) && (old_yaw >= 350 && old_yaw < 360)) {
      dYaw = robotPose.Yaw + (360.0 - old_yaw);
    }
    rotated_angle += dYaw;
  }
  duration = 0;
}

void Actuator::Actuator::CancelGoal()
{
  if (goal_handle_) {
    ac_->async_cancel_goal(goal_handle_);
  }
}

void Actuator::Actuator::CancelAllGoals()
{
  ac_->async_cancel_all_goals();
}

void Actuator::Actuator::ReturnHome()
{
  Goal = Home;
  MoveToGoal();
}

void Actuator::Actuator::MoveToGoal()
{
  goal_handle_ = nullptr;  // clear the previous goal's status so GetGoalStatus() won't report a stale SUCCEEDED
  MoveGoal.target_pose.pose.position = Goal;
  MoveGoal.target_pose.pose.orientation.w = 1.0;
  MoveGoal.target_pose.header.stamp = node_->now();

  auto send_goal_options = MoveBaseClient::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const MoveBaseGoalHandle::SharedPtr & goal_handle) {
      if (goal_handle) { goal_handle_ = goal_handle; }
    };
  send_goal_options.result_callback =
    [this](const MoveBaseClient::WrappedResult & /*result*/) {
      // goal status is tracked via goal_handle_ in the main loop
    };
  ac_->async_send_goal(MoveGoal, send_goal_options);
}

void Actuator::Actuator::Visualization()
{
  GoalMarker.points.clear();
  HomeMarker.points.clear();
  GoalMarker.pose.position = Goal;
  HomeMarker.pose.position = Home;
  goal_vis->publish(GoalMarker);
  home_vis->publish(HomeMarker);
}

void Actuator::Actuator::VisInit()
{
  // goal is pink
  GoalMarker.header.frame_id = header.frame_id;
  GoalMarker.header.stamp = header.stamp;
  GoalMarker.ns = "goal";
  GoalMarker.id = 4;
  GoalMarker.lifetime = rclcpp::Duration(0, 0);
  GoalMarker.type = visualization_msgs::msg::Marker::SPHERE;
  GoalMarker.action = visualization_msgs::msg::Marker::ADD;
  GoalMarker.color.a = 1.0;
  GoalMarker.color.r = 0.79;
  GoalMarker.color.g = 0.06;
  GoalMarker.color.b = 0.47;
  GoalMarker.scale.x = 0.3;
  GoalMarker.scale.y = 0.3;
  GoalMarker.scale.z = 0.3;
  GoalMarker.pose.orientation.w = 1.0;
  GoalMarker.points.clear();

  // home is green
  HomeMarker.header.frame_id = header.frame_id;
  HomeMarker.header.stamp = header.stamp;
  HomeMarker.ns = "home";
  HomeMarker.id = 5;
  HomeMarker.lifetime = rclcpp::Duration(0, 0);
  HomeMarker.type = visualization_msgs::msg::Marker::SPHERE;
  HomeMarker.action = visualization_msgs::msg::Marker::ADD;
  HomeMarker.color.a = 1.0;
  HomeMarker.color.r = 0.0;
  HomeMarker.color.g = 1.0;
  HomeMarker.color.b = 0.0;
  HomeMarker.scale.x = 0.3;
  HomeMarker.scale.y = 0.3;
  HomeMarker.scale.z = 0.3;
  HomeMarker.pose.orientation.w = 1.0;
  HomeMarker.points.clear();
}

void Actuator::Actuator::ObtainPose()
{
  geometry_msgs::msg::TransformStamped transform;
  tf_buffer_.canTransform("map", RobotBase, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
  int temp = 0;

  while (temp == 0 && rclcpp::ok()) {
    try {
      transform = tf_buffer_.lookupTransform("map", RobotBase, tf2::TimePointZero);
      temp = 1;
      robotPose.Position.x = transform.transform.translation.x;
      robotPose.Position.y = transform.transform.translation.y;
      robotPose.Position.z = 0.0;
      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      double Yaw = tf2::getYaw(q);
      if (Yaw < 0) {
        Yaw = 2 * PI - fabs(Yaw);
      }
      robotPose.Yaw = 180 * Yaw / PI;  // angle in degrees
    } catch (const tf2::TransformException & ex) {
      (void)ex;
      temp = 0;
      std::cout << "Cannot Obtain robot pose!!" << std::endl;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue;  // keep the previous valid robotPose on lookup failure
    }
  }
}

void Actuator::Actuator::ActuatorInit()
{
  std::string cmd_topic = "cmd_vel";
  std::string base_frame = "base_link";  // default value
  node_->declare_parameter<std::string>("cmd_topic", cmd_topic);
  node_->declare_parameter<std::string>("robot_base_frame", base_frame);
  node_->declare_parameter<float>("goal_tolerance", 0.2);      // m
  node_->declare_parameter<float>("obstacle_tolerance", 0.5);  // m
  node_->declare_parameter<float>("rotate_speed", 0.5);        // rad/s
  node_->get_parameter("cmd_topic", CmdTopic);
  node_->get_parameter("robot_base_frame", RobotBase);
  node_->get_parameter("goal_tolerance", GoalTolerance);
  node_->get_parameter("obstacle_tolerance", ObstacleTolerance);
  node_->get_parameter("rotate_speed", RotateSpeed);

  iteration = 0;
  GoHomeFlag = 0;
  centroids.clear();
  GoalClose.clear();
  RotSpeed.linear.x = 0.0;
  RotSpeed.linear.y = 0.0;
  RotSpeed.linear.z = 0.0;
  RotSpeed.angular.x = 0.0;
  RotSpeed.angular.y = 0.0;
  RotSpeed.angular.z = RotateSpeed;

  MoveGoal.target_pose.header.frame_id = "map";  // goal coordinates are computed in the map frame
  MoveGoal.target_pose.pose.position.z = 0.0;
  MoveGoal.target_pose.pose.orientation.w = 1.0;
}

void Actuator::Actuator::AddToClose(geometry_msgs::msg::Point & goal)
{
  // assure the current goal is not already in the close list
  if (std::find(GoalClose.begin(), GoalClose.end(), goal) == GoalClose.end()) {
    GoalClose.push_back(goal);
  }
}

geometry_msgs::msg::Point Actuator::Actuator::SelectGoal(std::vector<geometry_msgs::msg::Point> & centroids)
{
  ObtainPose();
  int index = 0;
  int count = 0;  // whether all centroids are in GoalClose
  double shortest = 10000;
  double temp;
  if (centroids.size() == 0) {
    std::cout << "No centroids!  No goal!" << std::endl;
    return Home;
  } else {
    for (int i = 0; i < centroids.size(); i++) {
      if (GoalClose.size() != 0) {
        for (int n = 0; n < GoalClose.size(); n++) {
          float Distance = sqrt(pow((centroids[i].x - GoalClose[n].x), 2) +
                                pow((centroids[i].y - GoalClose[n].y), 2));
          if (Distance < GoalTolerance && Distance > 0.0001) {
            GoalClose.push_back(centroids[i]);  // abandon centroid close to an explored goal
            break;
          }
        }
        if (std::find(GoalClose.begin(), GoalClose.end(), centroids[i]) != GoalClose.end()) {
          count++;
          continue;
        }
      }
      temp = sqrt(pow((robotPose.Position.x - centroids[i].x), 2) +
                  pow((robotPose.Position.y - centroids[i].y), 2));
      int collision = CheckCollision(raw_map, robotPose.Position, centroids[i]);
      auto sigmoid = [collision]() { return 2 / (1 + exp(-0.3 * collision)) - 1; };
      temp = temp * (1 + sigmoid());  // cost function, shortest goal is prioritized
      if (temp < shortest) {
        shortest = temp;
        index = i;
      }
    }
    if (count == centroids.size()) {
      GoHomeFlag = 1;
      return Home;
    }
    std::cout << "GoalClose List size: " << GoalClose.size() << std::endl;
    Goal = centroids[index];
    iteration++;
    std::cout << "/************************/" << std::endl;
    std::cout << "Iteration: " << iteration << ": \nGoal: " << Goal.x << "," << Goal.y
              << "\nDistance: " << shortest << std::endl;
    return Goal;
  }
}

int Actuator::Actuator::CheckCollision(const nav_msgs::msg::OccupancyGrid & map,
                                       geometry_msgs::msg::Point & start, geometry_msgs::msg::Point & end)
{
  // guard: no map yet -> assume a collision-free straight segment
  if (map.data.empty() || map.info.resolution <= 0.0f) {
    return 0;
  }
  float length = sqrt(pow((start.x - end.x), 2) + pow((start.y - end.y), 2));
  float COS_THETA = (end.x - start.x) / length;
  float SIN_THETA = (end.y - start.y) / length;
  float resolution = map.info.resolution;
  float STEP = resolution;
  int count = 0;

  float x_check = start.x;
  float y_check = start.y;
  // segment checking
  while (fabs(x_check - end.x) > STEP && fabs(y_check - end.y) > STEP) {
    int x_check_world = (x_check + fabs(map.info.origin.position.x)) / map.info.resolution;
    int y_check_world = (y_check + fabs(map.info.origin.position.y)) / map.info.resolution;
    if (map.data[x_check_world + (y_check_world * map.info.width)] > 65) {
      count++;
    }
    x_check += STEP * COS_THETA;
    y_check += STEP * SIN_THETA;
  }
  return count;
}

rclcpp_action::GoalStatus Actuator::Actuator::GetGoalStatus() const
{
  if (goal_handle_) {
    return goal_handle_->get_status();
  }
  return rclcpp_action::GoalStatus::STATUS_UNKNOWN;
}

Actuator::Actuator::Actuator(const rclcpp::Node::SharedPtr & node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_, node),
  goal_vis(node->create_publisher<visualization_msgs::msg::Marker>("goal_vis", 1000)),
  home_vis(node->create_publisher<visualization_msgs::msg::Marker>("home_vis", 1000)),
  centroidsSub(node->create_subscription<frontier_exploration::msg::PointArray>(
    "centroids", 10, std::bind(&Actuator::centroidCallback, this, std::placeholders::_1))),
  RawMapSub_(node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 10, std::bind(&Actuator::mapCallback, this, std::placeholders::_1)))
{
  ActuatorInit();
  VisInit();
  ac_ = rclcpp_action::create_client<MoveBaseAction>(node, "move_base");

  // wait for the move_base (move_base_flex) action server, like the ROS1
  // SimpleActionClient("move_base", true) did
  while (rclcpp::ok() && !ac_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for move_base action server...");
  }

  ObtainPose();
  Home = robotPose.Position;
  Goal = Home;
  std::cout << "Home pose: " << Home.x << "," << Home.y << std::endl;
  cmdPub = node->create_publisher<geometry_msgs::msg::Twist>(CmdTopic, 1000);
}

Actuator::Actuator::~Actuator()
{
  GoalMarker.points.clear();
  GoalClose.clear();
  centroids.clear();
  VisInit();
}
```

- [ ] **Step 2: 更新 `CMakeLists.txt` —— 把 `actuator.cpp` 加入库**

把 Task 2 插入的 `add_library(frontierDetector ...)` 改为：

```cmake
add_library(frontierDetector
  src/frontier_detector.cpp
  src/actuator.cpp
)
```

- [ ] **Step 3: 构建验证**

```bash
source /opt/ros/humble/setup.bash && cd /home/ros/ros_ws/frontier_exploration_ws && colcon build --packages-select frontier_exploration
```

预期：`frontierDetector` 连同 `actuator.cpp` 编译通过，无错误。

- [ ] **Step 4: 提交**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && git add CMakeLists.txt src/actuator.cpp && git commit -m "feat: port actuator to rclcpp_action + tf2"
```

---

## Task 4: 移植 `frontierMain.cpp` 并建立可执行目标 + 安装规则

**Files:**
- Modify: `src/frontierMain.cpp`（整体重写为 ROS2 版本）
- Modify: `CMakeLists.txt`（加入 `add_executable(frontier_planner ...)` + `install(...)`）

**Interfaces:**
- Consumes: Task 2/3 的 `frontierDetector` 库、`Actuator::GetGoalStatus()`/`CancelAllGoals()`。
- Produces: 可执行文件 `frontier_planner`，安装规则。

- [ ] **Step 1: 重写 `src/frontierMain.cpp`**

用以下完整内容替换 `/home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid/src/frontierMain.cpp`：

```cpp
#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "frontier_exploration/actuator.h"
#include "frontier_exploration/frontier_detector.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("frontier_planner");

  time_t startTime, currTime, Duration;
  time_t Limit = 180;  // max time limitation for single goal navigation
  int changeFlag = 0;  // give up the current goal immediately if too close to an obstacle

  std::cout << "------------Exploration Starting------------" << std::endl;
  // create the frontier detector and actuator
  FrontierDetector::FrontierDetector frontier_detector(node);
  Actuator::Actuator actuator(node);
  // rotate 360 degrees to initialize the environment
  actuator.Rotation(360.0);

  // -------------------- Main Loop -------------------- //
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    // get all centroids
    frontier_detector.ComputeCentroids(frontier_detector.inflated_map, frontier_detector.frontier);
    // select a goal according to the cost value
    actuator.SelectGoal(frontier_detector.centroids);
    std::cout << "Found frontier cells: " << frontier_detector.frontier.size() << std::endl
              << "Found frontier: " << frontier_detector.centroids.size() << std::endl;
    // navigate to the goal
    actuator.MoveToGoal();
    time(&startTime);
    Duration = 0;
    // if the goal is abandoned, aborted, cancelled, or over time, select the next goal
    // (STATUS_LOST does not exist in ROS2 action_msgs/msg/GoalStatus — removed from the ROS1 port)
    while (rclcpp::ok() &&
           actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_SUCCEEDED &&
           actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_ABORTED &&
           actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_CANCELED &&
           bool(Duration < Limit)) {
      rclcpp::spin_some(node);
      if (frontier_detector.GridValue(frontier_detector.inflated_map, actuator.Goal) >= 65) {
        changeFlag = 1;
        actuator.CancelAllGoals();
        std::cout << "Goal's close to obstacle. Changed Goal!!" << std::endl;
        break;
      }
      time(&currTime);
      Duration = currTime - startTime;  // avoid spending too much time on one goal
    }
    if (Duration >= Limit) { std::cout << "Overtime. Changed Goal!!" << std::endl; }
    actuator.AddToClose(actuator.Goal);  // add to closeList to avoid revisiting the explored goal
    if (changeFlag != 1 && Duration < Limit) {
      std::cout << "Reached the goal!" << std::endl;
      actuator.Rotation(0.0);
    } else {
      changeFlag = 0;
    }

    if (frontier_detector.frontier.size() == 0 || actuator.GoHomeFlag == 1) {  // for homing
      actuator.ReturnHome();
      while (rclcpp::ok() &&
             actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
        std::cout << "Exploration finished! Returning home.." << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(5));
        rclcpp::spin_some(node);
      }
      rclcpp::shutdown();
    }
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: 更新 `CMakeLists.txt` —— 加入可执行目标与安装规则**

把 Task 1 末尾的 `ament_package()` 之前（即库目标之后）插入：

```cmake
add_executable(frontier_planner src/frontierMain.cpp)
target_include_directories(frontier_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(frontier_planner
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  mbf_msgs
)
target_link_libraries(frontier_planner frontierDetector ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS frontier_planner frontierDetector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
```

最终 `CMakeLists.txt` 应为：Task1 的 header + find_package + `rosidl_generate_interfaces`，随后库目标、可执行目标、`install(...)`、`ament_package()`（顺序严格如上）。

- [ ] **Step 3: 构建 + 安装验证**

```bash
source /opt/ros/humble/setup.bash && cd /home/ros/ros_ws/frontier_exploration_ws && colcon build --packages-select frontier_exploration
source install/setup.bash && ros2 pkg executables frontier_exploration
```

预期：构建成功；`ros2 pkg executables frontier_exploration` 输出 `frontier_planner`。

- [ ] **Step 4: 提交**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && git add CMakeLists.txt src/frontierMain.cpp && git commit -m "feat: port frontier_planner main loop to rclcpp"
```

---

## Task 5: README 更新、清理遗留头文件、最终验证

**Files:**
- Modify: `README.md`（重写为 ROS2 说明）
- Delete: `include/frontier_detector.h`、`include/actuator.h`（旧扁平 ROS1 头，已被 `include/frontier_exploration/*.h` 取代）

**Interfaces:**
- Consumes: Task 1-4 全部产物。
- Produces: 文档 + 干净的最终交付物。

- [ ] **Step 1: 重写 `README.md`**

用以下内容替换 `/home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid/README.md`：

````markdown
# Frontier-based exploration (ROS 2)

ROS 2 (Humble) 版本的前沿探索包：在占用栅格地图上检测前沿、计算质心，并通过
move_base (move_base_flex) 动作服务器驱动机器人探索未知环境。原算法逻辑逐行保留，
仅把 ROS 1 API 迁移到 ROS 2。

> 本仓库同时保留了 ROS 1 的仿真资产（`launch/`、`config/`、`urdf/`、`world/`、`meshes/`）。
> 它们面向 Melodic/旧版 Gazebo 与 ROS 1 move_base，不参与 ROS 2 构建，仅作参考。

## 依赖

- ROS 2 Humble
- 构建期：`rclcpp`、`rclcpp_action`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、
  `geometry_msgs`、`nav_msgs`、`visualization_msgs`、`mbf_msgs`、`rosidl_default_generators`
- 运行期：提供 `mbf_msgs/action/MoveBase` 动作服务的导航栈（例如 move_base_flex）、
  SLAM 地图、`map -> base_link` 的 TF 树

```sh
sudo apt install ros-humble-move-base-flex   # 若需要 move_base_flex 动作服务器
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
3. 提供 `mbf_msgs/action/MoveBase` 的动作服务器（topic `/move_base`），例如 move_base_flex。

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

- 节点启动时会等待 `/move_base` 动作服务器与 TF，之后开始探索（与原版 ROS 1 行为一致）。
- 这是纯库移植，不包含 ROS 1 的 launch/Gazebo/URDF 仿真栈；如需完整仿真，请在 ROS 2 下
  自行用 move_base_flex / Nav2、SLAM 与 Gazebo 搭建。
- 移植中修正的原始缺陷：
  - 服务名拼写错误 `get_centriods` → `get_centroids`；
  - 清理未使用的订阅/发布器与成员（原 `std_srvs` client、`move_base/goal`、`move_base/cancel` 等）；
  - 地图未到达时 `Rotation()` 与碰撞检测的除零/越界防护。

## 参考

- 原 ROS 1 版本：本仓库历史提交（`git log`）
- 设计文档：`docs/superpowers/specs/2026-08-24-frontier-exploration-ros2-port-design.md`
````

- [ ] **Step 2: 删除旧扁平头文件**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && git rm include/frontier_detector.h include/actuator.h
```

预期：`git status` 显示两个删除（旧头文件）。

- [ ] **Step 3: 全仓 ROS1 残留检查**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && grep -rnE 'ros/ros\.h|(^|[^A-Za-z0-9_])ros::|tf::|actionlib|move_base_msgs|catkin|add_message_files|generate_messages|roscpp' src include package.xml CMakeLists.txt
```

预期：**无任何输出**（README 的 legacy 说明是 prose，不在检查范围）。注意 `(^|[^A-Za-z0-9_])ros::` 要求 `ros::` 前面不是标识符字符，从而排除合法的 `tf2_ros::` 误报。

- [ ] **Step 4: 干净重编译 + 接口/依赖冒烟检查**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws && rm -rf build/install/log && source /opt/ros/humble/setup.bash && colcon build --packages-select frontier_exploration
source install/setup.bash && ros2 interface show frontier_exploration/msg/PointArray && ros2 interface show frontier_exploration/srv/GetCentroids && ros2 pkg executables frontier_exploration && ros2 pkg prefix mbf_msgs
```

预期：构建成功；接口与可执行名正确输出；`ros2 pkg prefix mbf_msgs` 输出 `/opt/ros/humble`。

- [ ] **Step 5: 提交**

```bash
cd /home/ros/ros_ws/frontier_exploration_ws/src/frontier_exploration_occupancygrid && git add README.md && git commit -m "docs: rewrite README for ROS 2 and drop legacy flat headers"
```

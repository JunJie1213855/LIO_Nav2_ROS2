#include "frontier_exploration/actuator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <functional>
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace
{
void log_actuator_timing(
  const rclcpp::Node::SharedPtr & node, const char * stage, double duration_ms)
{
  RCLCPP_INFO(node->get_logger(),
    "legacy_timing module=Actuator stage=%s duration_ms=%.3f",
    stage, duration_ms);
}
}

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
  if (raw_map.data.empty() || raw_map.info.resolution <= 0.0f)
  {
    RCLCPP_WARN(node_->get_logger(), "Rotation skipped: no map received yet (raw_map empty).");
    return;
  }

  // 获取当前位姿
  ObtainPose(); // obtain the robot's current pose
  // frame transformation: map -> image
  int world_x = static_cast<int>(std::floor(
    (robotPose.Position.x - raw_map.info.origin.position.x) / raw_map.info.resolution));
  int world_y = static_cast<int>(std::floor(
    (robotPose.Position.y - raw_map.info.origin.position.y) / raw_map.info.resolution));
  int CheckXmin = int(world_x - round(ObstacleTolerance / raw_map.info.resolution)),
      CheckXmax = int(world_x + round(ObstacleTolerance / raw_map.info.resolution));
  int CheckYmin = int(world_y - round(ObstacleTolerance / raw_map.info.resolution)),
      CheckYmax = int(world_y + round(ObstacleTolerance / raw_map.info.resolution));
  // check neighbors
  for (int x = CheckXmin; x <= CheckXmax; x++)
  {
    for (int y = CheckYmin; y <= CheckYmax; y++)
    {
      if (x < 0 || y < 0 || x >= static_cast<int>(raw_map.info.width) ||
          y >= static_cast<int>(raw_map.info.height))
      {
        continue;
      }
      if (raw_map.data[x + (y * raw_map.info.width)] >= 70)
      {
        std::cout << "Position close to obstacle. Cannot rotate" << std::endl;
        return;
      }
    }
  }

  // 记录已经旋转的角度
  double rotated_angle = 0.0; // rotated angle
  time_t initTime, currTime;
  time_t duration = 0L;
  time_t limitation = 15L; // rotate within 15 seconds
  time(&initTime);

  while ((rotated_angle < angle) && (rclcpp::ok()) && bool(duration < limitation))
  {
    time(&currTime);
    duration = currTime - initTime;
    // 记录旧角度
    double old_yaw = robotPose.Yaw;
    cmdPub->publish(RotSpeed); // 旋转一次
    // 获取位姿
    ObtainPose();
    rclcpp::spin_some(node_);
    double dYaw = robotPose.Yaw - old_yaw; // 记录增量角度
    if ((robotPose.Yaw >= 0 && robotPose.Yaw < 350) && (old_yaw >= 350 && old_yaw < 360))
    {
      dYaw = robotPose.Yaw + (360.0 - old_yaw);
    }
    rotated_angle += dYaw; // 记录已经旋转的总角度
  }
  duration = 0;
}

void Actuator::Actuator::CancelGoal()
{
  if (goal_handle_)
  {
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
  const auto started_at = std::chrono::steady_clock::now();
  // 清空上一个目标点
  goal_handle_ = nullptr; // clear the previous goal's status

  // 设置目标点的坐标
  MoveGoal.pose.header.frame_id = "map";
  MoveGoal.pose.pose.position = Goal;
  MoveGoal.pose.pose.orientation.w = 1.0;
  MoveGoal.pose.header.stamp = node_->now();

  auto send_goal_options = NavGoalClient::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const NavGoalHandle::SharedPtr &goal_handle)
  {
    if (goal_handle)
    {
      goal_handle_ = goal_handle;
    }
  };
  send_goal_options.result_callback =
      [this](const NavGoalClient::WrappedResult & /*result*/)
  {
    // goal status is tracked via goal_handle_ in the main loop
  };
  // 发送到达目标
  ac_->async_send_goal(MoveGoal, send_goal_options);
  const auto finished_at = std::chrono::steady_clock::now();
  log_actuator_timing(node_, "MoveToGoal", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
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
  // 目标点设置为粉红色
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

  // 初始点设置为绿色
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
  const auto started_at = std::chrono::steady_clock::now();
  geometry_msgs::msg::TransformStamped transform;
  // 查看 ${RobotBase} 到 map 的 tf 位姿是否存在
  tf_buffer_.canTransform("map", RobotBase, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
  int temp = 0;

  while (temp == 0 && rclcpp::ok())
  {
    try
    {
      // 获取从 map -> ${RobotBase} 的 TF 变换
      transform = tf_buffer_.lookupTransform("map", RobotBase, tf2::TimePointZero);
      temp = 1;

      // 获取机器人平移 x, y
      robotPose.Position.x = transform.transform.translation.x;
      robotPose.Position.y = transform.transform.translation.y;
      robotPose.Position.z = 0.0; // 固定 z = 0, 因为是平面运动

      // 获取旋转角 yaw
      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      // 获取 yaw
      double Yaw = tf2::getYaw(q);
      if (Yaw < 0) // 如果是不规则的 yaw，则转换
      {
        Yaw = 2 * PI - fabs(Yaw);
      }
      robotPose.Yaw = 180 * Yaw / PI; // angle in degrees
    }
    catch (const tf2::TransformException &ex)
    {
      (void)ex;
      temp = 0;
      std::cout << "Cannot Obtain robot pose!!" << std::endl;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue; // keep the previous valid robotPose on lookup failure
    }
  }
  const auto finished_at = std::chrono::steady_clock::now();
  log_actuator_timing(node_, "ObtainPose", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
}

void Actuator::Actuator::ActuatorInit()
{
  std::string cmd_topic = "cmd_vel";
  std::string base_frame = "base_link"; // default value
  node_->declare_parameter<std::string>("cmd_topic", cmd_topic);
  node_->declare_parameter<std::string>("robot_base_frame", base_frame);
  node_->declare_parameter<float>("goal_tolerance", 0.2);     // m
  node_->declare_parameter<float>("obstacle_tolerance", 0.5); // m
  node_->declare_parameter<float>("rotate_speed", 0.5);       // rad/s
  node_->get_parameter("cmd_topic", CmdTopic);
  node_->get_parameter("robot_base_frame", RobotBase);
  node_->get_parameter("goal_tolerance", GoalTolerance);
  node_->get_parameter("obstacle_tolerance", ObstacleTolerance);
  node_->get_parameter("rotate_speed", RotateSpeed); // 每次旋转的角度

  iteration = 0;
  GoHomeFlag = 0;
  centroids.clear();
  GoalClose.clear();

  // 将初始化的旋转速度转换为 cmd 发布者可以接收的方式
  RotSpeed.linear.x = 0.0;
  RotSpeed.linear.y = 0.0;
  RotSpeed.linear.z = 0.0;
  RotSpeed.angular.x = 0.0;
  RotSpeed.angular.y = 0.0;
  RotSpeed.angular.z = RotateSpeed;

  // 移动目标点，设置初始值，其中 z = 0 （恒定），旋转的 w 固定为 1.0
  MoveGoal.pose.header.frame_id = "map"; // goal coordinates are computed in the map frame
  MoveGoal.pose.pose.position.z = 0.0;
  MoveGoal.pose.pose.orientation.w = 1.0;
}

void Actuator::Actuator::AddToClose(geometry_msgs::msg::Point &goal)
{
  // assure the current goal is not already in the close list
  if (std::find(GoalClose.begin(), GoalClose.end(), goal) == GoalClose.end())
  {
    GoalClose.push_back(goal);
  }
}

geometry_msgs::msg::Point Actuator::Actuator::SelectGoal(std::vector<geometry_msgs::msg::Point> &centroids)
{
  const auto started_at = std::chrono::steady_clock::now();
  // 获取当前位姿
  ObtainPose();
  int index = 0;
  int count = 0; // whether all centroids are in GoalClose
  double shortest = 10000;
  double temp;
  // 如果没有质心，就直接返回初始点
  if (centroids.size() == 0)
  {
    std::cout << "No centroids!  No goal!" << std::endl;
    const auto finished_at = std::chrono::steady_clock::now();
    log_actuator_timing(node_, "SelectGoal", std::chrono::duration<double, std::milli>(
      finished_at - started_at).count());
    return Home;
  }
  else
  {
    for (int i = 0; i < centroids.size(); i++)
    {
      if (GoalClose.size() != 0)
      {
        for (int n = 0; n < GoalClose.size(); n++)
        {
          // 计算距离
          float Distance = sqrt(pow((centroids[i].x - GoalClose[n].x), 2) +
                                pow((centroids[i].y - GoalClose[n].y), 2));
          // 阈值判断是否到达
          if (Distance < GoalTolerance && Distance > 0.0001)
          {
            GoalClose.push_back(centroids[i]); // abandon centroid close to an explored goal
            break;
          }
        }
        if (std::find(GoalClose.begin(), GoalClose.end(), centroids[i]) != GoalClose.end())
        {
          count++;
          continue;
        }
      }
      temp = sqrt(pow((robotPose.Position.x - centroids[i].x), 2) +
                  pow((robotPose.Position.y - centroids[i].y), 2));
      int collision = CheckCollision(raw_map, robotPose.Position, centroids[i]);
      auto sigmoid = [collision]()
      { return 2 / (1 + exp(-0.3 * collision)) - 1; };
      temp = temp * (1 + sigmoid()); // cost function, shortest goal is prioritized
      if (temp < shortest)
      {
        shortest = temp;
        index = i;
      }
    }
    if (count == centroids.size())
    {
      GoHomeFlag = 1;
      const auto finished_at = std::chrono::steady_clock::now();
      log_actuator_timing(node_, "SelectGoal", std::chrono::duration<double, std::milli>(
        finished_at - started_at).count());
      return Home;
    }
    std::cout << "GoalClose List size: " << GoalClose.size() << std::endl;
    Goal = centroids[index];
    iteration++;
    std::cout << "/************************/" << std::endl;
    std::cout << "Iteration: " << iteration << ": \nGoal: " << Goal.x << "," << Goal.y
              << "\nDistance: " << shortest << std::endl;
    const auto finished_at = std::chrono::steady_clock::now();
    log_actuator_timing(node_, "SelectGoal", std::chrono::duration<double, std::milli>(
      finished_at - started_at).count());
    return Goal;
  }
}

int Actuator::Actuator::CheckCollision(const nav_msgs::msg::OccupancyGrid &map,
                                       geometry_msgs::msg::Point &start, geometry_msgs::msg::Point &end)
{
  // guard: no map yet -> assume a collision-free straight segment
  if (map.data.empty() || map.info.resolution <= 0.0f)
  {
    return 0;
  }
  if (map.info.resolution <= 0.0 || map.info.width == 0 || map.info.height == 0 ||
      map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height) {
    return 0;
  }
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1e-6) {
    return 0;
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(length / map.info.resolution)));
  int count = 0;
  for (int step = 0; step <= steps; ++step)
  {
    const double ratio = static_cast<double>(step) / steps;
    const int x_check_world = static_cast<int>(std::floor(
      (start.x + ratio * dx - map.info.origin.position.x) / map.info.resolution));
    const int y_check_world = static_cast<int>(std::floor(
      (start.y + ratio * dy - map.info.origin.position.y) / map.info.resolution));
    if (x_check_world < 0 || y_check_world < 0 ||
        x_check_world >= static_cast<int>(map.info.width) ||
        y_check_world >= static_cast<int>(map.info.height)) {
      return count;
    }
    if (map.data[x_check_world + (y_check_world * map.info.width)] > 65)
    {
      count++;
    }
  }
  return count;
}

rclcpp_action::GoalStatus Actuator::Actuator::GetGoalStatus() const
{
  rclcpp_action::GoalStatus status;
  if (goal_handle_)
  {
    status.status = goal_handle_->get_status();
  }
  else
  {
    status.status = rclcpp_action::GoalStatus::STATUS_UNKNOWN;
  }
  return status;
}

Actuator::Actuator::Actuator(const rclcpp::Node::SharedPtr &node)
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
  ac_ = rclcpp_action::create_client<NavGoalAction>(node, "navigate_to_pose");

  // wait for the Nav2 navigate_to_pose action server, like the ROS1
  // SimpleActionClient("move_base", true) did
  while (rclcpp::ok() && !ac_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigate_to_pose action server...");
  }

  // 获取当前机器人的位姿，并且设置为初始点位姿
  ObtainPose();
  Home = robotPose.Position;
  Goal = Home;
  // 答应
  std::cout << "Home pose: " << Home.x << "," << Home.y << std::endl;
  // 创建指令发布者
  cmdPub = node->create_publisher<geometry_msgs::msg::Twist>(CmdTopic, 1000);
}

Actuator::Actuator::~Actuator()
{
  GoalMarker.points.clear();
  GoalClose.clear();
  centroids.clear();
  VisInit();
}

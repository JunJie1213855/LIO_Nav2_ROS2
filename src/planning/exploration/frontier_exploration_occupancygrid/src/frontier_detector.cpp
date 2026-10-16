#include "frontier_exploration/frontier_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>

namespace
{
void log_detector_timing(
  const rclcpp::Node::SharedPtr & node, const char * stage, double duration_ms)
{
  RCLCPP_INFO(node->get_logger(),
    "legacy_timing module=FrontierDetector stage=%s duration_ms=%.3f",
    stage, duration_ms);
}
}

// 获取占据式地图的值
// map 中的值表示
// 0：空闲区域，机器人可行走
// 1~99：不确定区域
// 100：占据区域，障碍物
// -1：未知区域
int FrontierDetector::FrontierDetector::GridValue(nav_msgs::msg::OccupancyGrid &map,
                                                  geometry_msgs::msg::Point &x1)
{
  // guard: no map yet -> treat as free (main loop calls this before the first /map)
  if (map.data.empty() || map.info.resolution <= 0.0f)
  {
    return 0;
  }
  // 获取地图的原点坐标值 x和y
  const int mx = static_cast<int>(std::floor(
    (x1.x - map.info.origin.position.x) / map.info.resolution));
  const int my = static_cast<int>(std::floor(
    (x1.y - map.info.origin.position.y) / map.info.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(map.info.width) ||
      my >= static_cast<int>(map.info.height))
  {
    return 100;
  }
  const std::size_t index = static_cast<std::size_t>(my) * map.info.width + mx;
  if (index >= map.data.size()) {
    return 100;
  }
  // 代价值
  int out = map.data[index];
  return out;
}

int FrontierDetector::FrontierDetector::CheckNeibor(nav_msgs::msg::OccupancyGrid &inflated_map,
                                                    long &index)
{
  if (inflated_map.info.width == 0 || inflated_map.info.height == 0 ||
      index < 0 || static_cast<std::size_t>(index) >= inflated_map.data.size()) {
    return 0;
  }
  const int x = index % static_cast<int>(inflated_map.info.width);
  const int y = index / static_cast<int>(inflated_map.info.width);
  /** 八邻域检测，只要该点八邻域存在小于 65 且大于等于 35 或者是非探索点直接设置为 frontier 点
   * [x - 1, y - 1] [x , y - 1] [x + 1, y - 1]
   * [x - 1, y    ] [x , y    ] [x + 1, y    ]
   * [x - 1, y + 1] [x , y + 1] [x + 1, y + 1]
   */
  for (int i = x - 1; i <= x + 1; i++)
  {
    for (int j = y - 1; j <= y + 1; j++)
    {
      // 如果是当前点直接跳过
      if (i == x && j == y)
      {
        continue;
      }
      if (i < 0 || i >= static_cast<int>(inflated_map.info.width) ||
          j < 0 || j >= static_cast<int>(inflated_map.info.height))
      {
        continue;
      }
      // 判断法则
      if ((inflated_map.data[j * inflated_map.info.width + i] >= 35 &&
           inflated_map.data[j * inflated_map.info.width + i] < 65) ||
          inflated_map.data[j * inflated_map.info.width + i] == -1)
      {
        // neighbor cell is unknown -> frontier
        return 1;
      }
    }
  }
  return 0;
}

void FrontierDetector::FrontierDetector::InitVis()
{
  // frontier 前沿点是蓝色的
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

  // centroids 质心是红色的
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

// 每次接收 map 就会执行一次
void FrontierDetector::FrontierDetector::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr raw_map)
{
  const auto started_at = std::chrono::steady_clock::now();
  // 首先设置 frontier map 的相关信息
  FrontierDetector::raw_map.header = raw_map->header;
  FrontierDetector::raw_map.info = raw_map->info;
  FrontierDetector::raw_map.data = raw_map->data;

  // 膨胀 grid map
  InflateMap(FrontierDetector::raw_map, FrontierDetector::inflated_map);

  // 获取所有 frontier 点
  ComputeFrontier(inflated_map);
  Frontier.points = frontier;

  // 质心点设置
  Centroids.points = centroids;

  // 发布信息
  FrontierPub_->publish(Frontier);
  if (Centroids.points.size() != 0)
  {
    CentroidsPub_->publish(Centroids);
  }
  Visualization();
  const auto finished_at = std::chrono::steady_clock::now();
  log_detector_timing(node_, "mapCallback_total", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
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
  if (FrontierDetector::frontier.empty())
  {
    std::cout << "Frontier no found! Waiting......" << std::endl;
  }
  if (FrontierDetector::centroids.empty())
  {
    std::cout << "Computing the goal! Waiting....." << std::endl;
  }
  if (frontier.size() == 0 || centroids.size() == 0)
  {
    // avoid node dying if no frontier or centroid found
    FrontierDetector::frontierMarker_->publish(FrontierDetector::frontier_vis);
    FrontierDetector::centroidMarker_->publish(FrontierDetector::centroid_vis);
  }
  else
  {
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

bool FrontierDetector::FrontierDetector::InflateMap(nav_msgs::msg::OccupancyGrid &raw_map,
                                                    nav_msgs::msg::OccupancyGrid &inflated_map)
{
  const auto started_at = std::chrono::steady_clock::now();
  inflated_map = raw_map;
  if (raw_map.info.resolution <= 0.0 || raw_map.info.width == 0 || raw_map.info.height == 0 ||
      raw_map.data.size() != static_cast<std::size_t>(raw_map.info.width) * raw_map.info.height) {
    return false;
  }
  const int dilate_amount = std::max(0, static_cast<int>(std::lround(
    OBSTABLE_INFLATION / raw_map.info.resolution)));
  for (int y = 0; y < static_cast<int>(raw_map.info.height); y++)
  {
    for (int x = 0; x < static_cast<int>(raw_map.info.width); x++)
    {
      const std::size_t index = static_cast<std::size_t>(raw_map.info.width) * y + x;
      if (raw_map.data[index] < 65 && raw_map.data[index] >= 0)
      {
        inflated_map.data[index] = 0; // free cell, continue
        continue;
      }
      if (raw_map.data[index] == -1)
      {
        continue;
      }
      for (int i = -dilate_amount; i <= dilate_amount; i++)
      {
        for (int j = -dilate_amount; j <= dilate_amount; j++)
        {
          int x_d = x + i;
          int y_d = y + j;
          if (x_d < 0 || x_d >= static_cast<int>(raw_map.info.width) ||
              y_d < 0 || y_d >= static_cast<int>(raw_map.info.height))
          {
            continue;
          }
          inflated_map.data[raw_map.info.width * y_d + x_d] = 100; // inflate with obstacle cells
        }
      }
    }
  }
  inflatedMapPub_->publish(inflated_map);
  const auto finished_at = std::chrono::steady_clock::now();
  log_detector_timing(node_, "InflateMap", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
  return true;
}

bool FrontierDetector::FrontierDetector::ComputeFrontier(nav_msgs::msg::OccupancyGrid &inflated_map)
{
  const auto started_at = std::chrono::steady_clock::now();
  FrontierDetector::frontier.clear();
  FrontierDetector::frontier_indices.clear();
  geometry_msgs::msg::Point p;
  if (!inflated_map.data.empty())
  {
    const auto width = static_cast<int>(inflated_map.info.width);
    const auto height = static_cast<int>(inflated_map.info.height);
    if (width <= 0 || height <= 0 || inflated_map.data.size() !=
        static_cast<std::size_t>(width) * height) {
      return false;
    }
    frontier_indices.reserve(inflated_map.data.size() / 20);
    for (std::size_t n = 0; n < inflated_map.data.size(); n++)
    {
      // if the cell is free and a neighbor is unknown, it is a frontier cell
      // 如果当前点为高置信度点且邻域有一个是低置信度点或者未知区域，就将其设置为 frontier 
      long neighbor_index = static_cast<long>(n);
      if (inflated_map.data[n] >= 0 && inflated_map.data[n] < 35 &&
          CheckNeibor(inflated_map, neighbor_index) == 1)
      {
        const int n_x = static_cast<int>(n % inflated_map.info.width);
        const int n_y = static_cast<int>(n / inflated_map.info.width);
        p.x = (n_x + 0.5) * inflated_map.info.resolution + inflated_map.info.origin.position.x;
        p.y = (n_y + 0.5) * inflated_map.info.resolution + inflated_map.info.origin.position.y;
        p.z = 0.0;
        frontier_indices.push_back(static_cast<int>(n));
        FrontierDetector::frontier.push_back(p);
      }
    }
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "map data isn't received!");
    return false;
  }
  const auto finished_at = std::chrono::steady_clock::now();
  log_detector_timing(node_, "ComputeFrontier", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
  return true;
}

bool FrontierDetector::FrontierDetector::ComputeCentroids(
    nav_msgs::msg::OccupancyGrid &inflated_map, std::vector<geometry_msgs::msg::Point> &frontiers)
{
  const auto started_at = std::chrono::steady_clock::now();
  // 清空质心点和原始质心点
  FrontierDetector::centroids.clear();
  FrontierDetector::raw_centroids.clear();
  pointGroup.clear();
  if (frontiers.size() == 0)
  {
    std::cout << "Cannot find any frontiers! Checking!!" << std::endl;
    return false;
  }


  if (frontier_indices.size() != frontiers.size()) {
    frontier_indices.clear();
    frontier_indices.reserve(frontiers.size());
    for (const auto & point : frontiers) {
      const int mx = static_cast<int>(std::floor(
        (point.x - inflated_map.info.origin.position.x) / inflated_map.info.resolution));
      const int my = static_cast<int>(std::floor(
        (point.y - inflated_map.info.origin.position.y) / inflated_map.info.resolution));
      if (mx >= 0 && my >= 0 && mx < static_cast<int>(inflated_map.info.width) &&
          my < static_cast<int>(inflated_map.info.height)) {
        frontier_indices.push_back(my * static_cast<int>(inflated_map.info.width) + mx);
      }
    }
  }

  const int width = static_cast<int>(inflated_map.info.width);
  const int height = static_cast<int>(inflated_map.info.height);
  const std::size_t cell_count = static_cast<std::size_t>(width) * height;
  std::vector<uint8_t> is_frontier(cell_count, 0);
  std::vector<uint8_t> visited(cell_count, 0);
  for (const int index : frontier_indices) {
    if (index >= 0 && static_cast<std::size_t>(index) < cell_count) {
      is_frontier[index] = 1;
    }
  }
  static constexpr int offsets[8][2] = {
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}};

  // Cluster directly on integer grid indices. This avoids recursive point lookup and
  // turns the previous repeated std::find operations into a linear traversal.
  for (const int seed : frontier_indices)
  {
    if (seed < 0 || static_cast<std::size_t>(seed) >= cell_count || visited[seed]) {
      continue;
    }
    std::queue<int> queue;
    std::vector<int> cluster;
    visited[seed] = 1;
    queue.push(seed);
    while (!queue.empty()) {
      const int current = queue.front();
      queue.pop();
      cluster.push_back(current);
      const int x = current % width;
      const int y = current / width;
      for (const auto &offset : offsets) {
        const int nx = x + offset[0];
        const int ny = y + offset[1];
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }
        const int next = ny * width + nx;
        if (is_frontier[next] && !visited[next]) {
          visited[next] = 1;
          queue.push(next);
        }
      }
    }
    if (cluster.size() <= 6)
    { // avoid too-short frontiers
      continue;
    }
    const int index = cluster[cluster.size() / 2];
    geometry_msgs::msg::Point centroid;
    const int cx = index % width;
    const int cy = index / width;
    centroid.x = (cx + 0.5) * inflated_map.info.resolution + inflated_map.info.origin.position.x;
    centroid.y = (cy + 0.5) * inflated_map.info.resolution + inflated_map.info.origin.position.y;
    centroid.z = 0.0;
    raw_centroids.push_back(centroid);
  }
  frontierClose.clear();

  std::vector<uint8_t> removed(raw_centroids.size(), 0);
  // close-pair filter: keep one of two centroids closer than 3m along a collision-free line
  if (raw_centroids.size() >= 2)
  {
    for (int m = 0; m < raw_centroids.size(); m++)
    {
      for (int n = static_cast<int>(raw_centroids.size()) - 1; n > m; n--)
      {
        if (removed[n] || removed[m])
        {
          continue;
        }
        // 计算距离
        const float dx = raw_centroids[m].x - raw_centroids[n].x;
        const float dy = raw_centroids[m].y - raw_centroids[n].y;
        const float distance = std::sqrt(dx * dx + dy * dy);
        // 距离小于 3
        if (distance < 3 && CheckCollision(raw_map, raw_centroids[m], raw_centroids[n]))
        {
          removed[n] = 1;
        }
      }
    }
  }
  // BUG FIX: 原来 centroids 只在 raw_centroids.size()>=2 分支里填充；单个前沿群组
  // （探索接近尾声"只剩一片前沿"的常见情形）会得到 0 个质心 → 主循环没目标可探、
  // 又不满足返航条件，无限空转。现在无论多少 raw centroid 都照常输出。
  for (int num = 0; num < raw_centroids.size(); num++)
  {
    if (!removed[num])
    {
      centroids.push_back(raw_centroids[num]);
    }
  }
  std::cout << "pop_index: " << std::count(removed.begin(), removed.end(), 1) << std::endl;
  std::cout << "raw_centroids: " << raw_centroids.size() << std::endl;
  std::cout << "centroids: " << centroids.size() << std::endl;
  raw_centroids.clear();
  removed.clear();
  const auto finished_at = std::chrono::steady_clock::now();
  log_detector_timing(node_, "ComputeCentroids", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
  return true;
}

// lazy collision check
bool FrontierDetector::FrontierDetector::CheckCollision(
    const nav_msgs::msg::OccupancyGrid &map, geometry_msgs::msg::Point &start, geometry_msgs::msg::Point &end)
{
  // 起始点到终点的距离
  if (map.info.resolution <= 0.0 || map.info.width == 0 || map.info.height == 0 ||
      map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height) {
    return false;
  }
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1e-6) {
    return true;
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
      return false;
    }
    if (map.data[x_check_world + (y_check_world * map.info.width)] >= 70)
    {
      count++;
    }
    if (count > 2)
    {
      return false;
    }
  }
  return true;
}

// 可优化
void FrontierDetector::FrontierDetector::Grouping(nav_msgs::msg::OccupancyGrid &inflated_map,
                                                  geometry_msgs::msg::Point &point)
{
  // DFS 八邻域聚类，可以换成 BFS 更好一点
  int out = 0;
  geometry_msgs::msg::Point temp;
  geometry_msgs::msg::Point worldPoint;
  // transform the point to world(image) frame
  worldPoint.x = (point.x + std::fabs(inflated_map.info.origin.position.x)) / inflated_map.info.resolution - 0.5;
  worldPoint.y = (point.y + std::fabs(inflated_map.info.origin.position.y)) / inflated_map.info.resolution - 0.5;
  worldPoint.z = 0.0;

  if (!frontier.empty())
  {
    // 八 邻域聚类
    for (float i = worldPoint.x - 1; i <= worldPoint.x + 1; i++)
    {
      if (out == 1)
      {
        break;
      }
      for (float j = worldPoint.y - 1; j <= worldPoint.y + 1; j++)
      {
        int index;
        if (i == worldPoint.x && j == worldPoint.y)
        {
          continue;
        }
        temp.x = (i + 0.5) * inflated_map.info.resolution - std::fabs(inflated_map.info.origin.position.x);
        temp.y = (j + 0.5) * inflated_map.info.resolution - std::fabs(inflated_map.info.origin.position.y);
        temp.z = 0.0; // temp is in map frame

        if (std::find(frontier.begin(), frontier.end(), temp) != frontier.end())
        {
          if (frontierClose.size() > 1)
          {
            if (std::find(frontierClose.begin(), frontierClose.end(), temp) != frontierClose.end())
            {
              continue;
            }
          }
          auto itera = std::find(frontier.begin(), frontier.end(), temp);
          index = std::distance(frontier.begin(), itera);
          pointGroup.push_back(frontier[index]);
          frontierClose.push_back(frontier[index]);
          // 递归
          Grouping(inflated_map, frontier[index]); // recursive
          out = 1;
          break;
        }
      }
    }
  }
}

std::vector<geometry_msgs::msg::Point> FrontierDetector::FrontierDetector::Sort(
    nav_msgs::msg::OccupancyGrid &inflated_map, std::vector<geometry_msgs::msg::Point> &pts)
{
  const auto started_at = std::chrono::steady_clock::now();
  std::vector<geometry_msgs::msg::Point> outcome;
  outcome = pts;
  geometry_msgs::msg::Point p;
  if (!pts.empty())
  {
    p = pts[0];
    for (int i = 0; i < outcome.size() - 1; i++)
    {
      for (int j = 0; j < outcome.size() - 1 - i; j++)
      {
        // sort based on image index
        if ((outcome[j].y * inflated_map.info.width + outcome[j].x) <
            (outcome[j + 1].y * inflated_map.info.width + outcome[j + 1].x))
        {
          p = outcome[j];
          outcome[j] = outcome[j + 1];
          outcome[j + 1] = p;
        }
      }
    }
  }
  else
  {
    std::cout << "Group is empty!!" << std::endl;
  }
  const auto finished_at = std::chrono::steady_clock::now();
  log_detector_timing(node_, "Sort", std::chrono::duration<double, std::milli>(
    finished_at - started_at).count());
  return outcome;
}

// 初始化发布者和订阅者
// map 订阅者
// 获取质心点服务
FrontierDetector::FrontierDetector::FrontierDetector(const rclcpp::Node::SharedPtr &node)
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

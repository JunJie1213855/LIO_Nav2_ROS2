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

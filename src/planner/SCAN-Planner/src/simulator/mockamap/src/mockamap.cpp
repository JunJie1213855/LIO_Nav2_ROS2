#include <algorithm>
#include <memory>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "maps.hpp"

namespace
{
void optimizeMap(mocka::Maps::BasicInfo &info)
{
  std::vector<int> enclosed;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(info.cloud->makeShared());
  const double radius = 1.75 / info.scale;
  for (size_t i = 0; i < info.cloud->size(); ++i)
  {
    std::vector<int> indices;
    std::vector<float> distances;
    if (tree.radiusSearch(info.cloud->points[i], radius, indices, distances) >= 27)
      enclosed.push_back(static_cast<int>(i));
  }
  for (auto iterator = enclosed.rbegin(); iterator != enclosed.rend(); ++iterator)
    info.cloud->points.erase(info.cloud->points.begin() + *iterator);
  info.cloud->width = static_cast<uint32_t>(info.cloud->size());
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
}
}  // namespace

class MockamapNode : public rclcpp::Node
{
public:
  MockamapNode() : Node("mockamap_node")
  {
    const int seed = declare_parameter<int>("seed", 4546);
    const double update_frequency = declare_parameter<double>("update_freq", 1.0);
    const double resolution = declare_parameter<double>("resolution", 0.38);
    const int size_x = declare_parameter<int>("x_length", 100);
    const int size_y = declare_parameter<int>("y_length", 100);
    const int size_z = declare_parameter<int>("z_length", 10);
    const int type = declare_parameter<int>("type", 1);
    const double scale = 1.0 / resolution;

    mocka::Maps::BasicInfo info;
    info.node = this;
    info.sizeX = static_cast<int>(size_x * scale);
    info.sizeY = static_cast<int>(size_y * scale);
    info.sizeZ = static_cast<int>(size_z * scale);
    info.seed = seed;
    info.scale = scale;
    info.output = &output_;
    info.cloud = &cloud_;
    mocka::Maps map;
    map.setInfo(info);
    map.generate(type);
    output_.header.frame_id = "world";

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "mock_map", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, update_frequency)),
        std::bind(&MockamapNode::publishMap, this));
    publishMap();
  }

private:
  void publishMap()
  {
    output_.header.stamp = now();
    publisher_->publish(output_);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  sensor_msgs::msg::PointCloud2 output_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockamapNode>());
  rclcpp::shutdown();
  return 0;
}

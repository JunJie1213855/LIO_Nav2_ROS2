#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher() : Node("map_pub")
  {
    const std::string file_name = declare_parameter<std::string>("file_name", "");
    if (file_name.empty()) throw std::runtime_error("file_name parameter must reference a PCD file");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const double publish_rate = declare_parameter<double>("publish_rate", 0.2);
    const double downsample_resolution = declare_parameter<double>("downsample_res", 0.0);
    const double offset_x = declare_parameter<double>("map_offset_x", 0.0);
    const double offset_y = declare_parameter<double>("map_offset_y", 0.0);
    const double offset_z = declare_parameter<double>("map_offset_z", 0.0);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(file_name, cloud) != 0)
      throw std::runtime_error("failed to load PCD file: " + file_name);
    for (auto &point : cloud)
    {
      point.x += offset_x;
      point.y += offset_y;
      point.z += offset_z;
    }
    if (downsample_resolution > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      pcl::PointCloud<pcl::PointXYZ> downsampled;
      filter.setInputCloud(cloud.makeShared());
      filter.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      filter.filter(downsampled);
      cloud.swap(downsampled);
    }
    pcl::toROSMsg(cloud, message_);
    message_.header.frame_id = frame_id_;
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "global_cloud", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate)),
        std::bind(&MapPublisher::publishMap, this));
    publishMap();
    RCLCPP_INFO(get_logger(), "Loaded %zu PCD points from %s", cloud.size(), file_name.c_str());
  }

private:
  void publishMap()
  {
    message_.header.stamp = now();
    publisher_->publish(message_);
  }
  std::string frame_id_;
  sensor_msgs::msg::PointCloud2 message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<MapPublisher>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_pub"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

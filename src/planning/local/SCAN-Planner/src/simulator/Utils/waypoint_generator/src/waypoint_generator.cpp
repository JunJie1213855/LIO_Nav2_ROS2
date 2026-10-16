#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class WaypointGenerator : public rclcpp::Node
{
public:
  WaypointGenerator() : Node("waypoint_generator")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { latest_odom_ = *message; have_odom_ = true; });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", 10,
        std::bind(&WaypointGenerator::goalCallback, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("waypoints", 10);
    pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    if (!have_odom_)
    {
      RCLCPP_WARN(get_logger(), "Ignoring goal until body_pose is available");
      return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    geometry_msgs::msg::PoseStamped start;
    start.header = path.header;
    start.pose = latest_odom_.pose.pose;
    path.poses = {start, *goal};
    path.poses.back().header = path.header;
    path_pub_->publish(path);

    geometry_msgs::msg::PoseArray array;
    array.header = path.header;
    for (const auto &pose : path.poses) array.poses.push_back(pose.pose);
    pose_array_pub_->publish(array);
  }

  std::string frame_id_;
  bool have_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointGenerator>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_planner
{
class Go2KinematicSim : public rclcpp::Node
{
public:
  Go2KinematicSim() : Node("go2_kinematic_sim")
  {
    x_ = declare_parameter<double>("init_x", 0.0);
    y_ = declare_parameter<double>("init_y", 0.0);
    z_ = declare_parameter<double>("init_z", 0.3);
    yaw_ = declare_parameter<double>("init_yaw", 0.0);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    const double sim_rate = declare_parameter<double>("sim_rate", 100.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 100);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 20, std::bind(&Go2KinematicSim::cmdCallback, this, std::placeholders::_1));
    last_cmd_time_ = now();
    last_sim_time_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, sim_rate)),
        std::bind(&Go2KinematicSim::simCallback, this));
    RCLCPP_INFO(get_logger(), "Go2 kinematic simulator ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_cmd_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    vy_cmd_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    vyaw_cmd_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    last_cmd_time_ = now();
  }

  void publishOdom(const rclcpp::Time &stamp)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    const auto orientation = tf2::toMsg(quaternion);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = vx_world_;
    odom.twist.twist.linear.y = vy_world_;
    odom.twist.twist.angular.z = vyaw_cmd_;
    odom_pub_->publish(odom);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = z_;
      transform.transform.rotation = orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void simCallback()
  {
    const auto current_time = now();
    double dt = (current_time - last_sim_time_).seconds();
    last_sim_time_ = current_time;
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    double vx = vx_cmd_, vy = vy_cmd_, wz = vyaw_cmd_;
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_)
      vx = vy = wz = 0.0;
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    vx_world_ = c * vx - s * vy;
    vy_world_ = s * vx + c * vy;
    x_ += vx_world_ * dt;
    y_ += vy_world_ * dt;
    yaw_ = normalizeAngle(yaw_ + wz * dt);
    publishOdom(current_time);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_{0.0}, y_{0.0}, z_{0.3}, yaw_{0.0};
  double vx_cmd_{0.0}, vy_cmd_{0.0}, vyaw_cmd_{0.0};
  double vx_world_{0.0}, vy_world_{0.0};
  double max_vx_{0.75}, max_vy_{0.35}, max_vyaw_{1.0}, cmd_timeout_{0.3};
  bool publish_tf_{false};
  std::string frame_id_, child_frame_id_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2KinematicSim>());
  rclcpp::shutdown();
  return 0;
}

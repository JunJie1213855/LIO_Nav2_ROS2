#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace scan_planner
{
class Go2GaitPublisher : public rclcpp::Node
{
public:
  Go2GaitPublisher() : Node("go2_gait_publisher")
  {
    const double rate = declare_parameter<double>("rate", 60.0);
    gait_frequency_ = declare_parameter<double>("gait_frequency", 2.2);
    min_walk_speed_ = declare_parameter<double>("min_walk_speed", 0.05);
    max_walk_speed_ = declare_parameter<double>("max_walk_speed", 1.0);
    always_trot_ = declare_parameter<bool>("always_trot", false);
    hip_swing_ = declare_parameter<double>("hip_swing", 0.08);
    thigh_swing_ = declare_parameter<double>("thigh_swing", 0.32);
    calf_swing_ = declare_parameter<double>("calf_swing", 0.42);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&Go2GaitPublisher::odomCallback, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
        std::bind(&Go2GaitPublisher::timerCallback, this));

    joint_msg_.name = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    const double twist_speed = std::hypot(vx, vy);
    rclcpp::Time stamp(odom->header.stamp);
    if (stamp.nanoseconds() == 0) stamp = now();
    const double x = odom->pose.pose.position.x;
    const double y = odom->pose.pose.position.y;
    double pose_speed = 0.0;
    if (has_prev_pose_)
    {
      const double dt = (stamp - last_odom_time_).seconds();
      if (dt > 1e-4) pose_speed = std::hypot(x - last_odom_x_, y - last_odom_y_) / dt;
    }
    horizontal_speed_ = twist_speed > min_walk_speed_ ? twist_speed : pose_speed;
    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_time_ = stamp;
    has_prev_pose_ = true;
    has_odom_ = true;
  }

  void timerCallback()
  {
    const auto stamp = now();
    if (!has_odom_)
    {
      publishStance(stamp);
      return;
    }
    const double ratio = always_trot_ ? 0.45 : std::clamp(
        (horizontal_speed_ - min_walk_speed_) / std::max(1e-3, max_walk_speed_ - min_walk_speed_),
        0.0, 1.0);
    if (ratio <= 1e-3)
    {
      publishStance(stamp);
      return;
    }
    const double phase = 2.0 * kPi * gait_frequency_ * stamp.seconds();
    fillLeg(0, phase, ratio, true);
    fillLeg(3, phase + kPi, ratio, false);
    fillLeg(6, phase + kPi, ratio, true);
    fillLeg(9, phase, ratio, false);
    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  void publishStance(const rclcpp::Time &stamp)
  {
    const std::array<double, 12> stance = {
        0.05, 0.82, -1.58, -0.05, 0.82, -1.58,
        0.05, 0.95, -1.62, -0.05, 0.95, -1.62};
    for (size_t i = 0; i < stance.size(); ++i)
    {
      joint_msg_.position[i] = stance[i];
      joint_msg_.velocity[i] = 0.0;
    }
    joint_msg_.header.stamp = stamp;
    joint_pub_->publish(joint_msg_);
  }

  void fillLeg(size_t offset, double phase, double ratio, bool left_side)
  {
    const double s = std::sin(phase), c = std::cos(phase), swing = std::max(0.0, s);
    const double side = left_side ? 1.0 : -1.0;
    const bool rear = offset >= 6;
    const double thigh_stance = rear ? 0.95 : 0.82;
    const double calf_stance = rear ? -1.62 : -1.58;
    joint_msg_.position[offset] = side * (0.05 + hip_swing_ * ratio * 0.35 * s);
    joint_msg_.position[offset + 1] = thigh_stance + thigh_swing_ * ratio * c;
    joint_msg_.position[offset + 2] = calf_stance + calf_swing_ * ratio * swing - 0.10 * ratio * (1.0 - swing);
    joint_msg_.position[offset] = std::clamp(joint_msg_.position[offset], -1.0, 1.0);
    joint_msg_.position[offset + 1] = std::clamp(joint_msg_.position[offset + 1], -1.2, 3.0);
    joint_msg_.position[offset + 2] = std::clamp(joint_msg_.position[offset + 2], -2.6, -0.9);
    const double omega = 2.0 * kPi * gait_frequency_;
    joint_msg_.velocity[offset] = side * hip_swing_ * ratio * 0.35 * c * omega;
    joint_msg_.velocity[offset + 1] = -thigh_swing_ * ratio * s * omega;
    joint_msg_.velocity[offset + 2] = calf_swing_ * ratio * (s > 0.0 ? c : 0.0) * omega;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_msg_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double horizontal_speed_{0.0}, last_odom_x_{0.0}, last_odom_y_{0.0};
  double gait_frequency_{2.2}, min_walk_speed_{0.05}, max_walk_speed_{1.0};
  double hip_swing_{0.08}, thigh_swing_{0.32}, calf_swing_{0.42};
  bool always_trot_{false}, has_odom_{false}, has_prev_pose_{false};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2GaitPublisher>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class OpenLoopController : public rclcpp::Node
{
public:
  OpenLoopController() : Node("open_loop_controller")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "quadruped");
    const double publish_rate = declare_parameter<double>("publish_rate", 100.0);
    yaw_min_speed_ = declare_parameter<double>("yaw_min_speed", 0.05);
    hold_final_position_ = declare_parameter<bool>("hold_final_position", true);
    last_yaw_ = declare_parameter<double>("init_yaw", 0.0);
    current_pos_ = Eigen::Vector3d(
        declare_parameter<double>("init_x", 0.0),
        declare_parameter<double>("init_y", 0.0),
        declare_parameter<double>("init_z", 0.3));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 20);
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&OpenLoopController::bsplineCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate)),
        std::bind(&OpenLoopController::publishOdom, this));
    RCLCPP_INFO(get_logger(), "Open-loop controller ready");
  }

private:
  bool parseBspline(const scan_planner_msgs::msg::Bspline::ConstSharedPtr &msg,
                    UniformBspline &pos_traj)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return false;
    }
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      pos_pts.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    pos_traj = UniformBspline(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);
    return true;
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    UniformBspline pos_traj;
    if (!parseBspline(msg, pos_traj)) return;
    traj_ = {pos_traj, pos_traj.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    start_time_ = rclcpp::Time(msg->start_time);
    traj_id_ = msg->traj_id;
    traj_duration_ = traj_[0].getTimeSum();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void publishState(const rclcpp::Time &stamp, const Eigen::Vector3d &pos,
                    const Eigen::Vector3d &vel, double yaw, double yaw_rate)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation = tf2::toMsg(quaternion);
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    odom.twist.twist.angular.z = yaw_rate;
    odom_pub_->publish(odom);
  }

  void publishOdom()
  {
    const auto now = this->now();
    if (!receive_traj_)
    {
      publishState(now, current_pos_, current_vel_, last_yaw_, 0.0);
      return;
    }
    const double elapsed = (now - start_time_).seconds();
    if (elapsed < 0.0)
    {
      publishState(now, current_pos_, current_vel_, last_yaw_, 0.0);
      return;
    }
    if (!hold_final_position_ && elapsed > traj_duration_) return;
    const double t = std::clamp(elapsed, 0.0, traj_duration_);
    const Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t);
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    if (elapsed <= traj_duration_)
    {
      vel = traj_[1].evaluateDeBoorT(t);
      acc = traj_[2].evaluateDeBoorT(t);
    }
    const double speed = std::hypot(vel.x(), vel.y());
    if (speed > yaw_min_speed_) last_yaw_ = std::atan2(vel.y(), vel.x());
    const double yaw_rate = speed > yaw_min_speed_
        ? (vel.x() * acc.y() - vel.y() * acc.x()) / std::max(speed * speed, 1e-6) : 0.0;
    current_pos_ = pos;
    current_vel_ = vel;
    publishState(now, pos, vel, last_yaw_, yaw_rate);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool receive_traj_{false};
  bool hold_final_position_{true};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::int64_t traj_id_{0};
  std::string frame_id_;
  std::string child_frame_id_;
  double last_yaw_{0.0};
  double yaw_min_speed_{0.05};
  Eigen::Vector3d current_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_vel_{Eigen::Vector3d::Zero()};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::OpenLoopController>());
  rclcpp::shutdown();
  return 0;
}

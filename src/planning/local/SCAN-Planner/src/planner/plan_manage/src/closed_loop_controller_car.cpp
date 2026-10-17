#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/execution_progress.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/car_path_tracker.h"

namespace scan_planner
{
class ClosedLoopControllerCar : public rclcpp::Node
{
public:
  ClosedLoopControllerCar() : Node("closed_loop_controller_car")
  {
    CarTrackerConfig config;
    config.max_vx = declare_parameter("max_vx", config.max_vx);
    config.max_vyaw = declare_parameter("max_vyaw", config.max_vyaw);
    config.max_accel = declare_parameter("max_accel", config.max_accel);
    config.max_decel = declare_parameter("max_decel", config.max_decel);
    config.max_yaw_accel = declare_parameter("max_yaw_accel", config.max_yaw_accel);
    config.max_lateral_accel = declare_parameter("max_lateral_accel", config.max_lateral_accel);
    config.lookahead_min = declare_parameter("lookahead_min", config.lookahead_min);
    config.lookahead_max = declare_parameter("lookahead_max", config.lookahead_max);
    config.lookahead_time = declare_parameter("lookahead_time", config.lookahead_time);
    config.rotate_enter = declare_parameter("rotate_enter", config.rotate_enter);
    config.rotate_exit = declare_parameter("rotate_exit", config.rotate_exit);
    config.kp_rotate = declare_parameter("kp_rotate", config.kp_rotate);
    config.finish_dist = declare_parameter("finish_dist", config.finish_dist);
    config.max_tracking_error = declare_parameter("max_tracking_error", config.max_tracking_error);
    config.progress_search_distance = declare_parameter("progress_search_distance", config.progress_search_distance);
    tracker_ = std::make_unique<CarPathTracker>(config);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);
    sample_dt_ = declare_parameter("sample_dt", 0.03);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
        !std::isfinite(sample_dt_) || sample_dt_ <= 0.0)
      throw std::invalid_argument("odom_timeout and sample_dt must be positive and finite");

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    progress_pub_ = create_publisher<scan_planner_msgs::msg::ExecutionProgress>("planning/execution_progress", 10);
    spline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>("planning/bspline", 10,
        std::bind(&ClosedLoopControllerCar::splineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopControllerCar::odomCallback, this, std::placeholders::_1));
    last_tick_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&ClosedLoopControllerCar::controlCallback, this));
    RCLCPP_INFO(get_logger(), "Differential-drive controller ready: spatial pure pursuit, vx <= %.2f, wz <= %.2f",
                 config.max_vx, config.max_vyaw);
  }

private:
  void publish(const CarControl &control, bool frozen)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = control.linear;
    command.angular.z = control.angular;
    command_pub_->publish(command);
    std_msgs::msg::Bool freeze;
    freeze.data = frozen;
    frozen_pub_->publish(freeze);
    if (have_path_)
    {
      scan_planner_msgs::msg::ExecutionProgress progress;
      progress.header.stamp = now();
      progress.header.frame_id = odom_frame_;
      progress.traj_id = traj_id_;
      progress.time_from_start = control.progress_time;
      progress.reached_end = control.reached_end;
      progress.tracking_lost = control.tracking_lost;
      progress_pub_->publish(progress);
    }
  }

  void splineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    const rclcpp::Time start(msg->start_time, get_clock()->get_clock_type());
    if (have_path_ && (start < trajectory_start_ || (start == trajectory_start_ && msg->traj_id <= traj_id_)))
      return;
    bool valid = msg->order == 3 && msg->pos_pts.size() >= 4 && msg->pos_pts.size() <= 20000 &&
                 msg->knots.size() == msg->pos_pts.size() + static_cast<std::size_t>(msg->order) + 1;
    if (valid)
    {
      for (std::size_t i = 0; i < msg->knots.size(); ++i)
        valid = valid && std::isfinite(msg->knots[i]) &&
                (i == 0 || msg->knots[i] >= msg->knots[i - 1]);
      // Velocity derivative denominators must be nonzero.
      for (std::size_t i = 0; i + 1 < msg->pos_pts.size(); ++i)
        valid = valid && msg->knots[i + 4] - msg->knots[i + 1] > 1e-9;
    }
    if (valid)
    {
      Eigen::MatrixXd points(3, msg->pos_pts.size());
      Eigen::VectorXd knots(msg->knots.size());
      for (std::size_t i = 0; i < msg->pos_pts.size(); ++i)
        points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
      for (std::size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
      const double duration = knots(points.cols()) - knots(msg->order);
      valid = points.allFinite() && duration > 1e-6 && std::isfinite(duration) &&
              duration / sample_dt_ <= 20000;
      if (valid)
      {
        UniformBspline spline(points, msg->order, 0.1);
        spline.setKnot(knots);
        UniformBspline velocity = spline.getDerivative();
        const auto intervals = std::max<std::size_t>(1, std::ceil(duration / sample_dt_));
        std::vector<CarPathSample> path;
        path.reserve(intervals + 1);
        for (std::size_t i = 0; i <= intervals; ++i)
        {
          const double t = duration * static_cast<double>(i) / intervals;
          const Eigen::Vector2d v = velocity.evaluateDeBoorT(t).head<2>();
          CarPathSample sample;
          sample.position = spline.evaluateDeBoorT(t).head<2>();
          sample.time = t;
          sample.speed = v.norm();
          path.push_back(sample);
        }
        valid = tracker_->setPath(std::move(path));
      }
    }
    if (!valid)
    {
      have_path_ = false;
      tracker_->stop();
      publish(CarControl{}, true);
      RCLCPP_ERROR(get_logger(), "Invalid B-spline; stopping differential-drive controller");
      return;
    }
    traj_id_ = msg->traj_id;
    trajectory_start_ = start;
    have_path_ = true;
    last_progress_ = 0.0;
    RCLCPP_INFO(get_logger(), "Tracking trajectory %lld with measured spatial progress",
                 static_cast<long long>(traj_id_));
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if ((!odom_frame_.empty() && msg->header.frame_id != odom_frame_) ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) || norm < 1e-12)
    {
      have_odom_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid odometry pose or unexpected frame; stopping");
      return;
    }
    position_ << p.x, p.y;
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    rotation.normalize();
    yaw_ = tf2::getYaw(rotation);
    odom_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    odom_received_ = std::chrono::steady_clock::now();
    have_odom_ = true;
  }

  void controlCallback()
  {
    const auto current = now();
    const double dt = (current - last_tick_).seconds();
    last_tick_ = current;
    const double receipt_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - odom_received_).count();
    const double stamp_age = (current - odom_stamp_).seconds();
    if (!have_path_ || !have_odom_ || receipt_age > odom_timeout_ ||
        stamp_age > odom_timeout_ || stamp_age < -0.1 || current < trajectory_start_ ||
        dt < 0.0 || dt > 0.2)
    {
      tracker_->stop();
      CarControl stopped;
      stopped.progress_time = last_progress_;
      publish(stopped, have_path_);
      return;
    }
    // A wall timer can run twice between /clock messages; do not reset motion then.
    if (dt == 0.0)
      return;
    const auto control = tracker_->update(position_, yaw_, dt);
    last_progress_ = control.progress_time;
    publish(control, control.rotating || control.tracking_lost);
    if (control.tracking_lost)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Tracking error %.3f m exceeds limit; stopped and requesting replan",
                           control.cross_track_error);
  }

  std::unique_ptr<CarPathTracker> tracker_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr frozen_pub_;
  rclcpp::Publisher<scan_planner_msgs::msg::ExecutionProgress>::SharedPtr progress_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr spline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool have_path_{false}, have_odom_{false};
  std::int64_t traj_id_{0};
  Eigen::Vector2d position_{Eigen::Vector2d::Zero()};
  double yaw_{0.0}, odom_timeout_{0.5}, sample_dt_{0.03}, last_progress_{0.0};
  std::string odom_frame_;
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME}, trajectory_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point odom_received_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<scan_planner::ClosedLoopControllerCar>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("closed_loop_controller_car"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

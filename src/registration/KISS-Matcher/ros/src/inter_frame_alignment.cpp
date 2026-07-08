#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "./tictoc.hpp"
#include "slam/loop_closure.h"
#include "slam/utils.hpp"

using namespace kiss_matcher;

class InterFrameAligner : public rclcpp::Node {
 public:
  explicit InterFrameAligner(const rclcpp::NodeOptions &options)
      : rclcpp::Node("inter_frame_aligner", options) {
    double frame_update_hz;

    LoopClosureConfig lc_config;

    auto &gc = lc_config.gicp_config_;
    auto &mc = lc_config.matcher_config_;

    source_frame_   = declare_parameter<std::string>("source_frame", "");
    target_frame_   = declare_parameter<std::string>("target_frame", "");
    world_frame_    = declare_parameter<std::string>("world", "world");
    frame_update_hz = declare_parameter<double>("frame_update_hz", 0.2);
    tf_hz_          = declare_parameter<double>("tf_hz", 100.0);

    lc_config.voxel_res_ = declare_parameter<double>("voxel_resolution", 1.0);
    lc_config.verbose_   = declare_parameter<bool>("loop.verbose", false);
    verbose_             = declare_parameter<bool>("verbose", false);

    gc.num_threads_               = declare_parameter<int>("local_reg.num_threads", 8);
    gc.correspondence_randomness_ = declare_parameter<int>("local_reg.correspondences_number", 20);
    gc.max_num_iter_              = declare_parameter<int>("local_reg.max_num_iter", 32);
    gc.scale_factor_for_corr_dist_ =
        declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
    gc.overlap_threshold_ = declare_parameter<double>("local_reg.overlap_threshold", 90.0);

    lc_config.enable_global_registration_ = declare_parameter<bool>("global_reg.enable", false);
    lc_config.num_inliers_threshold_ =
        declare_parameter<int>("global_reg.num_inliers_threshold", 20);

    rclcpp::QoS qos(1);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    tf_source_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_target_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reg_module_ = std::make_shared<LoopClosure>(lc_config, this->get_logger());

    source_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/src", qos);
    target_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/tgt", qos);
    coarse_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/coarse_alignment", qos);
    fine_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/fine_alignment", qos);

    inter_alignment_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / frame_update_hz),
                                std::bind(&InterFrameAligner::performAlignment, this));

    tf_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / tf_hz_),
                                        std::bind(&InterFrameAligner::publishTF, this));

    // 20 Hz is enough as long as it's faster than the full registration process.
    cloud_vis_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0),
                                std::bind(&InterFrameAligner::visualizeClouds, this));

    sub_source_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "source", qos, std::bind(&InterFrameAligner::callbackSource, this, std::placeholders::_1));
    sub_target_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "target", qos, std::bind(&InterFrameAligner::callbackTarget, this, std::placeholders::_1));

    source_cloud_.reset(new pcl::PointCloud<PointType>());
    target_cloud_.reset(new pcl::PointCloud<PointType>());
  }

  void callbackSource(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Source map cloud has come!");
    }
    pcl::fromROSMsg(*msg, *source_cloud_);
    is_source_updated_ = true;
    source_timestamp_  = rclcpp::Time(msg->header.stamp);
  }

  void callbackTarget(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Target map cloud has come!");
    }
    pcl::fromROSMsg(*msg, *target_cloud_);
    is_target_updated_ = true;
    target_timestamp_  = rclcpp::Time(msg->header.stamp);
  }

  void performAlignment() {
    if (!is_source_updated_ || !is_target_updated_) return;

    kiss_matcher::TicToc timer;
    // Only for visualization
    reg_module_->setSrcAndTgtCloud(*source_cloud_, *target_cloud_);
    const auto &reg_output = reg_module_->coarseToFineAlignment(*source_cloud_, *target_cloud_);
    const auto t           = timer.toc();

    RCLCPP_INFO(this->get_logger(), "Timing (msec) → Total: %.1f", t);

    // NOTE(hlim): No matter how the result is imprecise, flags should be updated
    is_source_updated_ = false;
    is_target_updated_ = false;

    need_cloud_vis_update_ = true;

    const double eps = 1e-6;

    if (!reg_output.is_valid_) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Alignment rejected. # of inliers: " << reg_output.num_final_inliers_);
    }

    target_T_source_ = reg_output.pose_;
    publishTF();
  }

  geometry_msgs::msg::TransformStamped createTransformStamped(const Eigen::Matrix4d &from_T_to,
                                                              const std::string &from_frame,
                                                              const std::string &to_frame,
                                                              const rclcpp::Time &stamp) {
    geometry_msgs::msg::TransformStamped transform_msg;

    // Extract rotation and translation
    Eigen::Matrix3d rot   = from_T_to.block<3, 3>(0, 0);
    Eigen::Vector3d trans = from_T_to.block<3, 1>(0, 3);
    Eigen::Quaterniond q(rot);

    // Fill message
    transform_msg.header.stamp            = stamp;
    transform_msg.header.frame_id         = from_frame;
    transform_msg.child_frame_id          = to_frame;
    transform_msg.transform.translation.x = trans.x();
    transform_msg.transform.translation.y = trans.y();
    transform_msg.transform.translation.z = trans.z();
    transform_msg.transform.rotation.x    = q.x();
    transform_msg.transform.rotation.y    = q.y();
    transform_msg.transform.rotation.z    = q.z();
    transform_msg.transform.rotation.w    = q.w();

    return transform_msg;
  }

  void publishTF() {
    static bool has_warned_waiting_for_clouds = false;
    if ((!source_timestamp_.has_value()) || (!target_timestamp_.has_value())) {
      if (!has_warned_waiting_for_clouds) {
        RCLCPP_WARN(this->get_logger(), "Waiting for map clouds...");
        has_warned_waiting_for_clouds = true;
      }
      return;
    }

    // NOTE(hlim): To visualize real-time topics, such as current scans,
    // we have to incrementally increase the timestamp
    rclcpp::Time now = this->get_clock()->now();
    if (!source_timestamp_base_.has_value() || *source_timestamp_ != *source_timestamp_base_) {
      source_timestamp_base_ = source_timestamp_;
      last_real_time_        = now;
    }
    if (!target_timestamp_base_.has_value() || *target_timestamp_ != *target_timestamp_base_) {
      target_timestamp_base_ = target_timestamp_;
      last_real_time_        = now;
    }

    rclcpp::Duration elapsed = now - last_real_time_;
    if (elapsed.nanoseconds() < 0) {
      elapsed = rclcpp::Duration::from_nanoseconds(0);
    }

    rclcpp::Time source_time = *source_timestamp_base_ + elapsed;
    rclcpp::Time target_time = *target_timestamp_base_ + elapsed;

    const auto &world_to_source_msg =
        createTransformStamped(target_T_source_, world_frame_, source_frame_, source_time);
    const auto &world_to_target_msg = createTransformStamped(
        Eigen::Matrix4d::Identity(), world_frame_, target_frame_, target_time);

    tf_source_broadcaster_->sendTransform(world_to_source_msg);
    tf_target_broadcaster_->sendTransform(world_to_target_msg);
  }

  void visualizeClouds() {
    if (!need_cloud_vis_update_) {
      return;
    }
    source_pub_->publish(toROSMsg(std::move(reg_module_->getSourceCloud()), world_frame_));
    target_pub_->publish(toROSMsg(std::move(reg_module_->getTargetCloud()), world_frame_));
    fine_aligned_pub_->publish(
        toROSMsg(std::move(reg_module_->getFinalAlignedCloud()), world_frame_));
    coarse_aligned_pub_->publish(
        toROSMsg(std::move(reg_module_->getCoarseAlignedCloud()), world_frame_));

    RCLCPP_INFO(this->get_logger(), "Clouds published!");
    need_cloud_vis_update_ = false;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_source_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_target_;

  std::string source_frame_;
  std::string target_frame_;
  std::string world_frame_;

  pcl::PointCloud<PointType>::Ptr source_cloud_;
  pcl::PointCloud<PointType>::Ptr target_cloud_;

  std::optional<rclcpp::Time> source_timestamp_;
  std::optional<rclcpp::Time> source_timestamp_base_;
  std::optional<rclcpp::Time> target_timestamp_;
  std::optional<rclcpp::Time> target_timestamp_base_;
  rclcpp::Time last_real_time_;

  bool verbose_ = false;

  bool is_source_updated_ = false;
  bool is_target_updated_ = false;

  bool need_cloud_vis_update_ = false;

  std::shared_ptr<kiss_matcher::LoopClosure> reg_module_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_source_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_target_broadcaster_;

  kiss_matcher::TicToc timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fine_aligned_pub_;

  rclcpp::TimerBase::SharedPtr inter_alignment_timer_;
  rclcpp::TimerBase::SharedPtr cloud_vis_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  double tf_hz_;

  Eigen::Matrix4d target_T_source_ = Eigen::Matrix4d::Identity();
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<InterFrameAligner>(options);

  // To allow timer callbacks to run concurrently using multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

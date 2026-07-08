#ifndef GLOBAL_KISS_MATCHER_RELOCALIZATION_HPP_
#define GLOBAL_KISS_MATCHER_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "slam/loop_closure.h"
#include "global_kiss_matcher_relocalization/packet.hpp"

namespace global_kiss_matcher_relocalization
{

class GlobalKissMatcherRelocalizationNode : public rclcpp::Node
{
public:
  explicit GlobalKissMatcherRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();
  kiss_matcher::LoopClosureConfig createLoopClosureConfig();
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool tryKissAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud,
    Eigen::Isometry3d & pose_out,
    size_t & inliers_out);
  bool tryGicpAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud,
    const Eigen::Isometry3d & initial_guess,
    Eigen::Isometry3d & pose_out);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  RelocState reloc_state_;  // 重定位状态机
  double global_leaf_size_;
  double registered_leaf_size_;
  double coarse_leaf_size_;
  double medium_leaf_size_;
  double fine_leaf_size_;

  int num_threads_;
  int num_neighbors_;
  double max_dist_sq_;
  double voxel_resolution_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;

  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // KISSMatcher global initialization
  std::shared_ptr<kiss_matcher::LoopClosure> reg_module_;
  bool initial_aligned_ = false;
  bool use_global_initialization_;
  bool use_kiss_recovery_;
  bool verify_kiss_with_gicp_;
  int gicp_fail_count_ = 0;
  int gicp_max_consecutive_failures_;
  int recovery_min_points_;
  double recovery_cooldown_sec_;
  rclcpp::Time last_recovery_attempt_time_;
};

}  // namespace global_kiss_matcher_relocalization

#endif  // GLOBAL_KISS_MATCHER_RELOCALIZATION_HPP_

#include "global_small_gicp_relocalization/global_small_gicp_relocalization.hpp"

namespace global_small_gicp_relocalization
{

GlobalSmallGicpRelocalizationNode::GlobalSmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("coarse_leaf_size", 0.8);
  this->declare_parameter("medium_leaf_size", 0.5);
  this->declare_parameter("fine_leaf_size", 0.25);
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("coarse_leaf_size", coarse_leaf_size_);
  this->get_parameter("medium_leaf_size", medium_leaf_size_);
  this->get_parameter("fine_leaf_size", fine_leaf_size_);
  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // Setup KISSMatcher for global initialization
  {
    kiss_matcher::LoopClosureConfig lc_config;

    auto &gc = lc_config.gicp_config_;
    auto &mc = lc_config.matcher_config_;

    lc_config.voxel_res_ = this->declare_parameter<double>("voxel_resolution", 0.25);
    lc_config.verbose_   = this->declare_parameter<bool>("loop.verbose", true);

    gc.num_threads_ = num_threads_;
    gc.correspondence_randomness_ = 20;
    gc.max_num_iter_ = 32;
    gc.scale_factor_for_corr_dist_ = 5.0;
    gc.overlap_threshold_ = 90.0;

    lc_config.enable_global_registration_ = true;
    lc_config.num_inliers_threshold_ = 3; // 测试中

    use_global_initialization_ = this->declare_parameter<bool>("use_global_initialization", true);

    reg_module_ = std::make_shared<kiss_matcher::LoopClosure>(lc_config, this->get_logger());
  }

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }

  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  preprocessTargetCloud();

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&GlobalSmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&GlobalSmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GlobalSmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&GlobalSmallGicpRelocalizationNode::publishTransform, this));
}

void GlobalSmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void GlobalSmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  *accumulated_cloud_ += *scan;
}

void GlobalSmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  // Phase 1: Use KISSMatcher for global initialization (no initial pose needed)
  if (!initial_aligned_ && use_global_initialization_) {
    // Downsample before KISSMatcher to avoid OOM on large maps
    double voxel_res;
    this->get_parameter("voxel_resolution", voxel_res);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(voxel_res, voxel_res, voxel_res);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_down(new pcl::PointCloud<pcl::PointXYZ>());
    vg.setInputCloud(accumulated_cloud_);
    vg.filter(*src_down);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_down(new pcl::PointCloud<pcl::PointXYZ>());
    vg.setInputCloud(global_map_);
    vg.filter(*tgt_down);

    RCLCPP_INFO(this->get_logger(), "KISSMatcher init: src=%zu, tgt=%zu (downsampled from %zu, %zu)",
                src_down->size(), tgt_down->size(), accumulated_cloud_->size(), global_map_->size());

    // Convert PointXYZ to PointXYZI for KISSMatcher
    pcl::PointCloud<PointType>::Ptr src_i(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr tgt_i(new pcl::PointCloud<PointType>());
    src_i->reserve(accumulated_cloud_->size());
    tgt_i->reserve(tgt_down->size());
    for (const auto & p : *accumulated_cloud_) {
      src_i->push_back({p.x, p.y, p.z, 0.f});
    }
    for (const auto & p : *tgt_down) {
      tgt_i->push_back({p.x, p.y, p.z, 0.f});
    }

    const auto & reg_output = reg_module_->coarseToFineAlignment(*src_i, *tgt_i);
    if (reg_output.is_valid_) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = reg_output.pose_.block<3, 3>(0, 0);
      pose.translation() = reg_output.pose_.block<3, 1>(0, 3);
      result_t_ = previous_result_t_ = pose;
      initial_aligned_ = true;
      RCLCPP_INFO(this->get_logger(), "KISSMatcher initialization succeeded. Inliers: %zu",
                  reg_output.num_final_inliers_);
    } else {
      RCLCPP_WARN(this->get_logger(), "KISSMatcher initialization failed. Inliers: %zu. Retrying next cycle...",
                  reg_output.num_final_inliers_);
    }
    accumulated_cloud_->clear();
    return;
  }

  // Phase 2: GICP continuous tracking (using previous result as initial guess)
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }

  accumulated_cloud_->clear();
}

void GlobalSmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void GlobalSmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

void GlobalSmallGicpRelocalizationNode::preprocessTargetCloud()
{
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  coarse_target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, coarse_leaf_size_);
  small_gicp::estimate_covariances_omp(*coarse_target_, num_neighbors_, num_threads_);
  coarse_target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    coarse_target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  medium_target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, medium_leaf_size_);
  small_gicp::estimate_covariances_omp(*medium_target_, num_neighbors_, num_threads_);
  medium_target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    medium_target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  fine_target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, fine_leaf_size_);
  small_gicp::estimate_covariances_omp(*fine_target_, num_neighbors_, num_threads_);
  fine_target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    fine_target_, small_gicp::KdTreeBuilderOMP(num_threads_));
}

}  // namespace global_small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(global_small_gicp_relocalization::GlobalSmallGicpRelocalizationNode)

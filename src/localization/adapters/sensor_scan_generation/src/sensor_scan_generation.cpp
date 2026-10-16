#include "sensor_scan_generation/sensor_scan_generation.hpp"

namespace sensor_scan_generation
{

  SensorScanGeneration::SensorScanGeneration(const rclcpp::NodeOptions &options)
      : Node("sensor_scan_generation", options)
  {

    // ── 帧名参数 (可通过 launch 覆盖) ────────────────────────────────────
    lidar_frame_         = this->declare_parameter<std::string>("lidar_frame", "livox_frame");
    base_footprint_frame_ = this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    chassis_frame_       = this->declare_parameter<std::string>("chassis_frame", "chassis");

    // ── TF 工具 ──────────────────────────────────────────────────────────
    // tf_buffer_ + tf_listener_ : 从 TF 树读取 URDF 静态外参
    // br_                       : 发布 odom → base_footprint 动态 TF
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    br_          = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── 输出话题 ─────────────────────────────────────────────────────────
    // /lidar_frame_pcd : livox_frame 系下的点云, 供重定位和 3D→2D 切片使用
    // /odom            : 标准里程计话题 (odom → base_footprint + 速度)
    pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar_frame_pcd", 2);
    pub_base_footprint_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 2);

    // ── 输入话题 (通过 message_filters 同步) ──────────────────────────────
    // 来自 lio_interface:
    //   /registered_odometry : odom → livox_frame 位姿
    //   /registered_scan     : odom 系下的点云
    //
    // 使用 BEST_EFFORT 的 QoS: 允许偶尔丢帧, 保证低延迟
    rmw_qos_profile_t qos_profile = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};

    laser_cloud_sub_.subscribe(this, "/registered_scan", qos_profile);
    odometry_sub_.subscribe(this, "/registered_odometry", qos_profile);

    // ── 时间同步 ─────────────────────────────────────────────────────────
    // 位姿和点云来自同一个 LIO 时刻, 但因为走不同的话题路径,
    // 到达时间可能有微小的偏差。ApproximateTime 以时间戳最近为原则配对。
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(100),    // 队列长度 100, 容纳短暂乱序
        odometry_sub_,
        laser_cloud_sub_);

    sync_->registerCallback(std::bind(
        &SensorScanGeneration::laserCloudAndOdometryHandler, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), MAG "SensorScanGeneration node is START" RST);
  }

  SensorScanGeneration::~SensorScanGeneration()
  {
    RCLCPP_INFO(this->get_logger(), MAG "SensorScanGeneration node is OVER" RST);
  }

  // ==========================================================================
  // laserCloudAndOdometryHandler — 核心计算回调
  // ==========================================================================
  //
  //  输入 (已由 message_filters 同步):
  //    odometry_msg : /registered_odometry,  frame_id="odom", child_frame="livox_frame"
  //                   → T_{odom}^{livox_frame}
  //    pcd_msg      : /registered_scan,      frame_id="odom"
  //                   → odom 系下的点云
  //
  //  输出:
  //    TF:   odom → base_footprint  (持续更新)
  //   话题: /odom                   (nav_msgs/Odometry, 含速度)
  //   话题: /lidar_frame_pcd        (转换回 livox_frame 系的点云)
  //
  //
  // 【计算过程】
  //
  //  步骤 1 — 解析输入位姿:
  //    tf_odom_to_lidar = T_{odom}^{livox_frame}
  //    (从 odometry_msg 的 pose 字段读出)
  //
  //  步骤 2 — 从 TF 树读取静态外参:
  //    tf_lidar_to_base_footprint = T_{livox_frame}^{base_footprint}
  //
  //    这个变换来自 URDF 的关节链:
  //      base_footprint → chassis → livox_frame,
  //    由 robot_state_publisher 发布到 /tf_static。
  //    sensor_scan_generation 从 TF 树反查:
  //      target = "livox_frame", source = "base_footprint"
  //    因为 TF 支持双向查询, 查逆变换不需要自己算。
  //
  //  步骤 3 — 合成:
  //    tf_odom_to_base_footprint = tf_odom_to_lidar * tf_lidar_to_base_footprint
  //                              = T_{odom}^{livox_frame} · T_{livox_frame}^{base_footprint}
  //                              = T_{odom}^{base_footprint}
  //
  //  步骤 4 — 发布:
  //    TF:  odom → base_footprint  (frame_id="odom", child_frame="base_footprint")
  //   话题: /odom                  (frame_id="odom", child_frame="base_footprint")
  //
  //
  //  【为什么还要把点云转回 livox_frame 系?】
  //   lio_interface 已经把点云转到了 odom 系 (方便统一),
  //   但重定位节点 (KISS-Matcher) 和 3D→2D 切片节点 (pointcloud_to_laserscan)
  //   需要的是 livox_frame 系下的点云 (即 LiDAR 自身坐标系)。
  //   所以这里做一次逆变换: pcd_lidar = T_{odom}^{livox_frame}^{-1} · pcd_odom
  //
  // ==========================================================================
  void SensorScanGeneration::laserCloudAndOdometryHandler(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odometry_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg)
  {
    RCLCPP_INFO(this->get_logger(), BLU "Received synchronized odometry and point cloud messages" RST);

    tf2::Transform tf_lidar_to_chassis;
    tf2::Transform tf_odom_to_chassis;
    tf2::Transform tf_odom_to_base_footprint_;
    tf2::Transform tf_odom_to_lidar;

    // ── 步骤 1: 解析 lio_interface 给的位姿 ────────────────────────
    // odometry_msg->pose.pose = T_{odom}^{livox_frame}
    tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

    // ── 步骤 2: 从 TF 树读取 URDF 静态外参 ──────────────────────────
    // 查询 livox_frame → base_footprint (即 base_footprint 在 LiDAR 坐标系下的位置)
    // 这是 robot_state_publisher 从 URDF 静态链发布的 /tf_static
    tf_lidar_to_base_footprint_ = getTransform(
        lidar_frame_, base_footprint_frame_, pcd_msg->header.stamp);

    // 同时也查 livox_frame → chassis (用于调试/完整性)
    tf_lidar_to_chassis = getTransform(
        lidar_frame_, chassis_frame_, pcd_msg->header.stamp);

    // ── 步骤 3: 合成 odom → base_footprint ─────────────────────────
    //
    //   T_{odom}^{chassis} = T_{odom}^{livox_frame} · T_{livox_frame}^{chassis}
    //   T_{odom}^{base_footprint} = T_{odom}^{livox_frame} · T_{livox_frame}^{base_footprint}
    //
    tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
    tf_odom_to_base_footprint_ = tf_odom_to_lidar * tf_lidar_to_base_footprint_;

    // ── 步骤 4a: 发布 odom → base_footprint TF ─────────────────────
    publishTransform(
        tf_odom_to_base_footprint_,
        odometry_msg->header.frame_id,    // "odom"
        base_footprint_frame_,            // "base_footprint"
        pcd_msg->header.stamp);

    // ── 步骤 4b: 发布 /odom 话题 (带速度估算) ─────────────────────
    publishOdometry(
        tf_odom_to_base_footprint_,
        odometry_msg->header.frame_id,    // "odom"
        base_footprint_frame_,            // "base_footprint"
        pcd_msg->header.stamp);

    // ── 步骤 4c: 点云逆变换 (odom 系 → livox_frame 系) ────────────
    // 下游重定位和 3D→2D 切片需要 LiDAR 自身坐标系下的点云
    //
    //   pcd_{lidar} = (T_{odom}^{livox_frame})^{-1} · pcd_{odom}
    //               = T_{livox_frame}^{odom} · pcd_{odom}
    //
    sensor_msgs::msg::PointCloud2 out;
    pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
    pub_laser_cloud_->publish(out);
  }

  // ==========================================================================
  // getTransform — 从 TF 树查询变换
  // ==========================================================================
  //
  //  查询 target → source 的 TF 变换。
  //  因为 TF 树不是 DAG 而是双向可查的结构, 所以即使 URDF 定义的是
  //  source → target, 也可以反向查询获得 target → source。
  //
  //  查不到时返回 identity (不阻塞整个管线)。
  //
  // ==========================================================================
  tf2::Transform SensorScanGeneration::getTransform(
      const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time)
  {
    try
    {
      auto transform_stamped = tf_buffer_->lookupTransform(
          target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);
      return transform;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
      return tf2::Transform::getIdentity();
    }
  }

  // ==========================================================================
  // publishTransform — 发布 TF
  // ==========================================================================
  //
  //  发布 parent_frame → child_frame 的动态 TF 变换,
  //  写入 /tf 话题, 供整个 TF 树的其他节点查询。
  //
  //  这条 TF 是整个导航栈的核心链路:
  //    map  →  odom  →  base_footprint
  //     ↑        ↑           ↑
  //   SLAM   本节点      机器人底盘
  //
  // ==========================================================================
  void SensorScanGeneration::publishTransform(
      const tf2::Transform &transform, const std::string &parent_frame,
      const std::string &child_frame, const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp      = stamp;
    transform_msg.header.frame_id   = parent_frame;   // "odom"
    transform_msg.child_frame_id    = child_frame;    // "base_footprint"
    transform_msg.transform         = tf2::toMsg(transform);
    br_->sendTransform(transform_msg);
  }

  // ==========================================================================
  // publishOdometry — 发布 nav_msgs/Odometry
  // ==========================================================================
  //
  //  发布标准的 /odom 话题。
  //  除了位姿 (position + orientation), 还需要速度 (linear + angular)。
  //
  //  【速度估算方法】
  //   因为 LIO 不直接输出速度, 所以用差分法近似:
  //
  //   线速度:
  //     v = (position_now - position_prev) / dt
  //
  //   角速度:
  //     先算两个四元数之间的差值: q_diff = q_now * q_prev^{-1}
  //     再转为轴角表示:          w = axis * angle / dt
  //
  //  【注意】
  //   这是数值差分, 高频噪声会被放大。Nav2 的 velocity_smoother
  //   会对 cmd_vel 做平滑, 可容忍此问题。
  //
  // ==========================================================================
  void SensorScanGeneration::publishOdometry(
      const tf2::Transform &transform, std::string parent_frame, const std::string &child_frame,
      const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp      = stamp;
    out.header.frame_id   = parent_frame;   // "odom"
    out.child_frame_id    = child_frame;    // "base_footprint"

    // ── 位姿 ──────────────────────────────────────────────────────────
    const auto &origin = transform.getOrigin();
    out.pose.pose.position.x  = origin.x();
    out.pose.pose.position.y  = origin.y();
    out.pose.pose.position.z  = origin.z();
    out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

    // ── 速度 (差分计算) ──────────────────────────────────────────────
    static tf2::Transform previous_transform;
    static auto previous_time = std::chrono::steady_clock::now();
    const auto current_time = std::chrono::steady_clock::now();

    const double dt =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            current_time - previous_time).count() * 1e-9;

    if (dt > 0)
    {
      // 线速度: v = Δposition / Δt
      const auto linear_velocity =
          (transform.getOrigin() - previous_transform.getOrigin()) / dt;

      // 角速度: ω = axis · angle / Δt
      //   q_diff = q_now · q_prev^{-1}  →  轴角: axis * angle
      const tf2::Quaternion q_diff =
          transform.getRotation() * previous_transform.getRotation().inverse();
      const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

      out.twist.twist.linear.x  = linear_velocity.x();
      out.twist.twist.linear.y  = linear_velocity.y();
      out.twist.twist.linear.z  = linear_velocity.z();
      out.twist.twist.angular.x = angular_velocity.x();
      out.twist.twist.angular.y = angular_velocity.y();
      out.twist.twist.angular.z = angular_velocity.z();
    }

    previous_transform = transform;
    previous_time = current_time;

    pub_base_footprint_odometry_->publish(out);
  }

} // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGeneration)

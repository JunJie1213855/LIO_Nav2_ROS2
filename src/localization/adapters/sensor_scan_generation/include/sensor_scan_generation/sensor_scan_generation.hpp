#ifndef SENSOR_SCAN_GENERATION_HPP
#define SENSOR_SCAN_GENERATION_HPP

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pcl_ros/transforms.hpp"

#define RST  "\033[0m"
#define BLU  "\033[34m"
#define MAG  "\033[35m"

namespace sensor_scan_generation
{

/**
 * ===========================================================================
 * sensor_scan_generation — 标准化里程计发布节点
 * ===========================================================================
 *
 * 【它解决什么问题】
 * lio_interface 输出的是 "odom → livox_frame" (LiDAR 在 odom 下的位姿),
 * 但 ROS 2 导航栈 (Nav2 / slam_toolbox) 需要的是 "odom → base_footprint"
 * (底盘在 odom 下的位姿) 以及标准的 /odom 话题。
 * 本节点用 URDF 中 LiDAR→底盘的静态外参做这步换算。
 *
 *
 * 【坐标系约定】
 *
 *   odom        : ROS 标准里程计世界帧。原点 = 机器人启动时 base_footprint 的位置。
 *   base_footprint : 机器人底盘在地面的投影。Nav2 的参考帧。
 *   livox_frame : LiDAR 传感器的安装位置 (URDF 中定义的 link)。
 *   chassis     : 机器人机身的 link (URDF 中定义)。
 *
 *
 * 【数据流】
 *
 *   lio_interface →  /registered_odometry  (odom → livox_frame)
 *                    /registered_scan       (odom 系下的点云)
 *                              │
 *                              ▼
 *                  sensor_scan_generation (本节点)
 *                     │                   │
 *    ┌────────────────┼───────────────────┼──────────────────────┐
 *    │                │                   │                      │
 *    ▼                ▼                   ▼                      ▼
 * /odom 话题      TF: odom→              /lidar_frame_pcd
 * (带速度)        base_footprint         (livox_frame 系点云)
 *
 *
 * 【核心计算】
 *
 *   T_{odom}^{base_footprint} = T_{odom}^{livox_frame} · T_{livox_frame}^{base_footprint}
 *                                  ↑                           ↑
 *                          lio_interface 给的          从 TF 树查的 (URDF 外参)
 *
 *   其中:
 *     T_{livox_frame}^{base_footprint} = (T_{base_footprint}^{livox_frame})^{-1}
 *     base_footprint → livox_frame 由 robot_state_publisher 从 URDF 读取并发布
 *
 *
 * 【消息同步】
 *   使用 message_filters::ApproximateTime 同步 /registered_odometry
 *   和 /registered_scan, 确保每帧位姿和点云时间戳一致。
 *
 *
 * 【速度估算】
 *   Nav2 需要里程计消息含线速度和角速度。本节点通过前后两帧的位姿差分计算:
 *     v_linear  = (position_t - position_{t-1}) / dt
 *     v_angular = (quat_t * quat_{t-1}^{-1}).axis * angle / dt
 *
 * ===========================================================================
 */
class SensorScanGeneration : public rclcpp::Node
{
public:
    explicit SensorScanGeneration(const rclcpp::NodeOptions & options);
    ~SensorScanGeneration();

private:

    // ── 帧名参数 ─────────────────────────────────────────────────────────

    std::string lidar_frame_;           // "livox_frame"
    std::string base_footprint_frame_;  // "base_footprint"
    std::string chassis_frame_;         // "chassis"

    // ── TF 缓冲 ──────────────────────────────────────────────────────────

    /**
     * 从 TF 树查到的 livox_frame → base_footprint 静态外参 (URDF 定义)。
     * 每个同步回调里从 /tf_static 读一次, 因为有时序问题不一定能直接缓存。
     */
    tf2::Transform tf_lidar_to_base_footprint_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;     // 用于发布动态 TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;            // TF 查询缓冲
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── 话题 ─────────────────────────────────────────────────────────────

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;      // /lidar_frame_pcd
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_base_footprint_odometry_; // /odom

    // ── 消息同步 ─────────────────────────────────────────────────────────

    message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;        // /registered_odometry
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_; // /registered_scan

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ── 回调 ─────────────────────────────────────────────────────────────

    /**
     * 同步回调: 每对齐一帧位姿+点云, 就计算并发布 odom → base_footprint。
     */
    void laserCloudAndOdometryHandler(
        const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg);

    // ── 辅助 ─────────────────────────────────────────────────────────────

    /** 从 TF 树查询 parent→child 的变换 */
    tf2::Transform getTransform(
        const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

    /** 发布一个 TF 变换 (frame → child) */
    void publishTransform(
        const tf2::Transform & transform, const std::string & parent_frame,
        const std::string & child_frame, const rclcpp::Time & stamp);

    /** 发布 nav_msgs/Odometry 消息 (含速度估算) */
    void publishOdometry(
        const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
        const rclcpp::Time & stamp);

};

} // namespace sensor_scan_generation


# endif

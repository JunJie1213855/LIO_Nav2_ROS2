#ifndef LIO_INTERFACE_HPP
#define LIO_INTERFACE_HPP

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "lio_interface/packet.hpp"

#define RST  "\033[0m"    /* Reset to default color */
#define BLU  "\033[34m"   /* Blue */
#define MAG  "\033[35m"   /* Magenta */

namespace lio_interface
{

/**
 * ===========================================================================
 * lio_interface — LIO 坐标系桥接节点
 * ===========================================================================
 *
 * 【它解决什么问题】
 * FAST-LIO / Point-LIO 内部使用 "camera_init" 作为世界坐标系原点。
 * 但 ROS 2 导航栈 (Nav2 / slam_toolbox) 期望使用标准的 "odom" 坐标系。
 * 这两个坐标系的原点不同，不能直接混用。
 *
 * 本节点的作用：把 LIO 的输出从 camera_init 坐标系 "平移对齐" 到 odom 坐标系，
 * 使下游导航栈可以正常工作。
 *
 *
 * 【坐标系约定】
 *
 *   camera_init : LIO 算法内部的世界帧。
 *                 原点 = LIO 初始化时 IMU (body) 所在的位置。
 *                 一旦初始化后，camera_init 在世界中固定不动。
 *
 *   body        : IMU 传感器本体的坐标系。
 *                 FAST-LIO 估计的就是 body 在 camera_init 中的位姿。
 *
 *   livox_frame : LiDAR 传感器本体的坐标系 (安装在机器人上的实际位置)。
 *                 在 URDF 中定义了 livox_frame 相对于 base_footprint 的静态外参。
 *
 *   odom        : ROS REP-105 标准里程计坐标系。
 *                 本项目约定：odom 的原点 = 机器人启动时 base_footprint 所在的位置。
 *
 *   base_footprint : 机器人底盘在地面的投影。Nav2 以此帧作为机器人参考帧。
 *
 *
 * 【TF 树结构】
 *
 *    odom ──(动态, 本节点+sensor_scan_generation 发布)──→ base_footprint
 *                                                              │
 *                                               (URDF 静态外参, robot_state_publisher 发布)
 *                                                              ↓
 *                                                        livox_frame
 *
 *
 * 【数据流】
 *
 *   FAST-LIO 输出:
 *     /Odometry        : camera_init → body          (IMU 在世界中的位姿)
 *     /cloud_registered: 相机_init 系下的点云
 *
 *   lio_interface (本节点):
 *     /registered_odometry : odom → livox_frame       (标准坐标系下的位姿)
 *     /registered_scan     : odom 系下的点云
 *
 *   sensor_scan_generation:
 *     接收 /registered_odometry + /registered_scan
 *     → 计算 odom → base_footprint, 发布 /odom 话题 + TF
 *
 *
 * 【对齐原理（核心逻辑）】
 *
 *   步骤 1 — 初始化 (只执行一次):
 *     从 TF 树读取 base_footprint → livox_frame 的静态外参 (来自 URDF)。
 *     把这个外参作为 odom → camera_init 的固定偏移量存储:
 *       tf_odom_to_lidar_odom_ = T_{base_footprint}^{livox_frame}
 *
 *     为什么可以这样做？
 *       启动时机器人静止, camera_init → body ≈ identity (没有位移),
 *       且 body 和 livox_frame 物理安装接近 (外参小),
 *       所以: odom 原点 ≈ camera_init 原点偏移了 base_footprint → livox_frame。
 *
 *   步骤 2 — 每帧运行时:
 *     T_{odom}^{livox_frame} = tf_odom_to_lidar_odom_ × T_{camera_init}^{body}
 *        ↑ odom 下的位姿       ↑ 固定偏移 (步骤1)     ↑ FAST-LIO 给的实时位姿
 *
 *
 * 【已知局限】
 *   FAST-LIO 输出的 child_frame 是 "body" (IMU 帧), 但本节点把它当作
 *   "livox_frame" (LiDAR 帧) 来用, 中间跳过了 IMU→LiDAR 的外参变换。
 *
 *   当前 FAST-LIO 配置中 extrinsic_T ≈ [0, 0, 0.28], extrinsic_R ≈ I,
 *   IMU 和 LiDAR 基本同轴、只差 28cm, 所以误差可忽略。
 *   如果 IMU 和 LiDAR 安装位置差得远, 需要在此处补上 T_{body}^{livox_frame}。
 *
 * ===========================================================================
 */
class LioInterface : public rclcpp::Node
{
public:
    explicit LioInterface(const rclcpp::NodeOptions & options);
    ~LioInterface() override;

private:

    // ── 参数 ──────────────────────────────────────────────────────────────

    std::string odometry_sub_;   // LIO 里程计话题名 (FAST-LIO: /Odometry, Point-LIO: /aft_mapped_to_init)

    // ── 状态 ──────────────────────────────────────────────────────────────

    bool base_frame_to_lidar_initialized_;  // 是否已完成初始化 (读取了 TF 外参)

    // ── 核心变量 ──────────────────────────────────────────────────────────

    /**
     * odom 到 camera_init 的固定对齐偏移量。
     *
     * 在初始化时设置, 之后不再改变。
     * 值等于 base_footprint → livox_frame 的静态外参 (从 URDF/TF 树读取)。
     *
     * 命名说明:
     *   lidar_odom = camera_init (LIO 的世界帧)
     *   所以 "tf_odom_to_lidar_odom" = T_{odom}^{camera_init}
     */
    tf2::Transform tf_odom_to_lidar_odom_;

    std::shared_ptr<lio_interface::data_> data_position_;

    // ── 回调 ──────────────────────────────────────────────────────────────

    /** 点云回调: camera_init 系点云 → odom 系点云 */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    /** 里程计回调: camera_init 系位姿 → odom 系位姿, 并初始化对齐偏移 */
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    // ── 话题 ──────────────────────────────────────────────────────────────

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ── TF ────────────────────────────────────────────────────────────────

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

}

#endif  // LIO_INTERFACE_HPP

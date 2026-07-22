#include "lio_interface/lio_interface.hpp"

namespace lio_interface
{

LioInterface::LioInterface(const rclcpp::NodeOptions & options)
: Node("lio_interface", options){

    this->declare_parameter("odometry_sub", "Odometry");
    this->get_parameter("odometry_sub", odometry_sub_);

    base_frame_to_lidar_initialized_ = false;

    data_position_ = std::make_shared<lio_interface::data_>();

    // ── 初始化 TF 监听器 ────────────────────────────────────────────────
    // 用于从 TF 树读取 base_footprint → livox_frame 的静态外参 (URDF 定义)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── 输出话题 ─────────────────────────────────────────────────────────
    // /registered_scan     : 转换到 odom 坐标系后的点云
    // /registered_odometry : 转换到 odom 坐标系后的位姿
    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 5);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/registered_odometry", 5);

    // ── 输入话题 ─────────────────────────────────────────────────────────
    // /cloud_registered : FAST-LIO 输出的 camera_init 系点云
    // odometry_sub_     : FAST-LIO 输出的 camera_init → body 位姿
    //                     (FAST-LIO 默认: /Odometry, Point-LIO: /aft_mapped_to_init)
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", rclcpp::SensorDataQoS(),
        std::bind(&LioInterface::pointCloudCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_sub_, rclcpp::SensorDataQoS(),
        std::bind(&LioInterface::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), MAG "LioInterface node is START" RST);
}

LioInterface::~LioInterface(){
    RCLCPP_INFO(this->get_logger(), MAG "LioInterface node is OVER" RST);
}

// ============================================================================
// pointCloudCallback — 点云坐标系转换
// ============================================================================
//
//  输入:  /cloud_registered — camera_init 坐标系下的点云
//  输出:  /registered_scan  — odom 坐标系下的点云
//
//  变换:  P_{odom} = T_{odom}^{camera_init} · P_{camera_init}
//                         ↑
//                    tf_odom_to_lidar_odom_ (初始化时计算好的固定偏移)
//
//  这里的坐标系转换很简单: 把点云从 LIO 的世界帧 (camera_init)
//  平移到 ROS 标准世界帧 (odom), 旋转+平移量就是初始化的对齐偏移。
//
// ============================================================================
void LioInterface::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // 将点云的 frame_id 从 camera_init 改为 odom, 并施加变换
    // tf_odom_to_lidar_odom_ = T_{odom}^{camera_init} — 固定对齐偏移
    pcl_ros::transformPointCloud("odom", tf_odom_to_lidar_odom_, *msg, *out);
    pcd_pub_->publish(*out);
}

// ============================================================================
// odometryCallback — 位姿坐标系转换
// ============================================================================
//
//  输入:  odometry_sub_ (默认 /Odometry) — camera_init → body
//        FAST-LIO 输出: frame_id = "camera_init", child_frame_id = "body"
//
//  输出:  /registered_odometry — odom → livox_frame
//        frame_id = "odom", child_frame_id = "livox_frame"
//
//
// 【分两步走】
//
//   第一步 (初始化, 只跑一次):
//     ┌──────────────────────────────────────────────────────────────┐
//     │ 从 TF 树查询 base_footprint → livox_frame 的静态外参         │
//     │ (这个外参来自 URDF, 由 robot_state_publisher 发布到 /tf_static) │
//     │                                                              │
//     │ 设: T_bf_lf = T_{base_footprint}^{livox_frame}               │
//     │                                                              │
//     │ 初始化: tf_odom_to_lidar_odom_ = T_bf_lf                    │
//     │         即: T_{odom}^{camera_init} = T_{base_footprint}^{livox_frame} │
//     │                                                              │
//     │ 为什么可以这样设？                                            │
//     │  - 启动时机器人静止, camera_init → body ≈ identity           │
//     │  - body 和 livox_frame 物理接近, T_{body}^{livox_frame} ≈ I  │
//     │  - 所以 odom 原点就对齐到了 base_footprint 此刻的位置         │
//     └──────────────────────────────────────────────────────────────┘
//
//   第二步 (每帧运行):
//     ┌──────────────────────────────────────────────────────────────┐
//     │ 从 FAST-LIO 消息读取 camera_init → body 的实时位姿:          │
//     │   tf_lidar_odom_to_lidar_frame = T_{camera_init}^{body}     │
//     │   (变量名写的是 lidar_frame, 但 FAST-LIO 实际发的是 body)     │
//     │                                                              │
//     │ 合成 odom 下的位姿:                                          │
//     │   tf_odom_to_lidar = tf_odom_to_lidar_odom_ × tf_lidar_odom_to_lidar_frame │
//     │                    = T_{odom}^{camera_init}  × T_{camera_init}^{body}       │
//     │                    ≈ T_{odom}^{livox_frame}  (近似, 见已知局限) │
//     │                                                              │
//     │ 发布到 /registered_odometry, frame_id="odom",                │
//     │ child_frame_id="livox_frame"                                │
//     └──────────────────────────────────────────────────────────────┘
//
//
// 【已知局限】
//   FAST-LIO 输出的是 camera_init → body (IMU 本体),
//   但这里把它当作 camera_init → livox_frame (LiDAR 传感器) 来用,
//   跳过了 extrinsic: T_{body}^{livox_frame}。
//
//   正确的应该是:
//     T_{odom}^{livox_frame} = T_{odom}^{camera_init} × T_{camera_init}^{body} × T_{body}^{livox_frame}
//                                                                          ↑ 当前缺失
//   当前外参很小 (extrinsic_T ≈ [0, 0, 0.28], extrinsic_R ≈ I),
//   所以误差可忽略。
//
// ============================================================================
void LioInterface::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg){

    // ── 第一步: 初始化 (只执行一次) ──────────────────────────────────────
    if (!base_frame_to_lidar_initialized_) {
    try {
        // 从 TF 树查询 URDF 中定义的 base_footprint → livox_frame 静态外参
        // 这个 TF 由 robot_state_publisher 根据 URDF 发布到 /tf_static
        auto tf_stamped = tf_buffer_->lookupTransform(
            "base_footprint",   // target frame
            "livox_frame",      // source frame — LiDAR 在底盘上的安装位置
            tf2::TimePointZero, // 使用最新的 TF
            std::chrono::seconds(1)
        );

        tf2::Transform tf_base_frame_to_lidar_frame;  // T_{base_footprint}^{livox_frame}

        tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar_frame);

        // 核心: 把 base_footprint → livox_frame 的外参当作 odom → camera_init 的对齐偏移
        //
        // 推理:
        //   启动时 camera_init → body ≈ identity (机器人没动)
        //   body 和 livox_frame 物理接近 (外参小, 28cm)
        //   → odom 原点 = 机器人 base_footprint 当前位置
        //   → odom 到 camera_init 的偏移 ≈ base_footprint 到 livox_frame 的偏移
        //
        // 变量:
        //   tf_odom_to_lidar_odom_ = T_{odom}^{camera_init}
        //   (变量名中 lidar_odom = camera_init, 即 LIO 的世界帧)
        tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar_frame;

        base_frame_to_lidar_initialized_ = true;

        RCLCPP_INFO(this->get_logger(),
            "lio_interface 初始化完成: T_{odom}^{camera_init} = "
            "(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
            tf_odom_to_lidar_odom_.getOrigin().x(),
            tf_odom_to_lidar_odom_.getOrigin().y(),
            tf_odom_to_lidar_odom_.getOrigin().z(),
            0.0, 0.0, 0.0);  // rpy 简化, 需要可从四元数提取
    }
    catch (const tf2::TransformException & ex) {
        // TF 还没就绪就跳过这一帧, 等下一帧再重试
        RCLCPP_WARN(this->get_logger(),
            "等待 TF (base_footprint → livox_frame) 就绪: %s", ex.what());
        return;
    }
    }

    // ── 第二步: 每帧的坐标系转换 ─────────────────────────────────────────

    // 变量名写的是 "lidar_odom_to_lidar_frame", 但 FAST-LIO 实际输出的是
    // camera_init → body (IMU), 不是 camera_init → livox_frame (LiDAR)。
    // (变量名保留是因为历史原因, 语义上它代表 "LIO 世界帧到传感器帧")
    tf2::Transform tf_lidar_odom_to_lidar_frame;  // = T_{camera_init}^{body}

    // 合成后的 odom 下的传感器位姿
    tf2::Transform tf_odom_to_lidar;              // = T_{odom}^{body} ≈ T_{odom}^{livox_frame}

    // 从 FAST-LIO 里程计消息中解析位姿: camera_init → body
    tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar_frame);

    // 合成 odom 下的位姿:
    //   T_{odom}^{body} = T_{odom}^{camera_init} × T_{camera_init}^{body}
    //                    = 固定偏移 (初始化时算好) × LIO 实时位姿
    tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar_frame;

    // ── 构造输出消息 ─────────────────────────────────────────────────────
    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "odom";          // 标准 ROS 世界坐标系
    out.child_frame_id = "livox_frame";    // LiDAR 传感器帧

    const auto & origin = tf_odom_to_lidar.getOrigin();
    out.pose.pose.position.x = origin.x();
    out.pose.pose.position.y = origin.y();
    out.pose.pose.position.z = origin.z();
    out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

    odom_pub_->publish(out);

}

}   // namespace lio_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lio_interface::LioInterface)

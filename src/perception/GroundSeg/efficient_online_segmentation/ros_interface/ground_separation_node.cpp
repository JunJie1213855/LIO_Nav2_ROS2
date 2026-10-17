/* Copyright 2024. Paired cloud/odometry segmentation and cloud-only processing. */
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "core/efficient_online_segmentation.h"
#include "point_cloud_io.h"

class GroundSeparationNode : public rclcpp::Node
{
public:
    explicit GroundSeparationNode(bool standalone)
    : Node("ground_separation"), standalone_(standalone)
    {
        LoadAlgorithmParams();
        if (!params_.UpdateInternalParams())
            throw std::invalid_argument("Invalid segmentation parameters");
        segmentation_.reset(new EfficientOnlineSegmentation(params_));
        segmentation_->SetVerbose(declare_parameter<bool>("verbose", false));
        ground_pub_ = create_publisher<ground_seg::Cloud>(pub_ground_cloud_topic_, 5);
        nonground_pub_ = create_publisher<ground_seg::Cloud>(pub_no_ground_cloud_topic_, 5);
        if (standalone_) {
            cloud_sub_ = create_subscription<ground_seg::Cloud>(sub_cloud_topic_, rclcpp::SensorDataQoS(),
                [this](ground_seg::Cloud::ConstSharedPtr cloud) { process(cloud, nullptr); });
        } else {
            pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(pub_odometry_topic_, 10);
            cloud_filter_.subscribe(this, sub_cloud_topic_, rmw_qos_profile_sensor_data);
            pose_filter_.subscribe(this, odometry_topic_, rmw_qos_profile_sensor_data);
            sync_.reset(new Sync(cloud_filter_, pose_filter_, sync_queue_size_));
            sync_->registerCallback(std::bind(&GroundSeparationNode::process, this,
                                              std::placeholders::_1, std::placeholders::_2));
        }
        RCLCPP_INFO(get_logger(), "%s mode; input cloud: %s; TF is not used or published",
                    standalone_ ? "Cloud-only" : "Exact cloud/odometry pairing", sub_cloud_topic_.c_str());
    }

private:
    void LoadAndConvertDegToRad(const std::string& name, float& variable)
    {
        const double deg2rad = 1.0 / 180.0 * M_PI;
        double value;
        this->get_parameter(name, value);
        variable = static_cast<float>(value * deg2rad);
    }

    void LoadAndConvertDegToSlope(const std::string& name, float& variable)
    {
        const double deg2rad = 1.0 / 180.0 * M_PI;
        double value;
        this->get_parameter(name, value);
        variable = static_cast<float>(std::tan(value * deg2rad));
    }

    void LoadAlgorithmParams()
    {
        // ROS interface params.
        this->declare_parameter<std::string>("sub_cloud_topic", standalone_ ? "/livox/lidar" : "/registered_scan");
        this->declare_parameter<std::string>("pub_ground_cloud_topic", "/ground_cloud");
        this->declare_parameter<std::string>("pub_no_ground_cloud_topic", "/no_ground_cloud");
        this->declare_parameter<std::string>("sensor_frame_id", "livox_frame");
        input_cloud_is_world_ = this->declare_parameter<bool>("input_cloud_is_world", !standalone_);
        odometry_topic_ = this->declare_parameter<std::string>("odometry_topic", "/odom");
        pub_odometry_topic_ = this->declare_parameter<std::string>("pub_odometry_topic", "/ground_seg/odom");
        odometry_is_sensor_pose_ = this->declare_parameter<bool>("odometry_is_sensor_pose", false);
        body_frame_id_ = this->declare_parameter<std::string>("body_frame_id", "base_footprint");

        sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 30);
        if (sync_queue_size_ < 1 || sync_queue_size_ > 10000)
            throw std::invalid_argument("sync_queue_size must be in [1, 10000]");
        if (standalone_ && input_cloud_is_world_)
            throw std::invalid_argument("Cloud-only mode requires sensor-frame input");

        // LiDAR parameters (angles in degree, converted below).
        this->declare_parameter<int>("kLidarRows", 50);
        this->declare_parameter<int>("kLidarCols", 360);
        this->declare_parameter<double>("kLidarHorizRes", 1.0);
        this->declare_parameter<double>("kLidarVertRes", 1.273);
        this->declare_parameter<double>("kLidarVertFovMax", 55.2);
        this->declare_parameter<double>("kLidarVertFovMin", -7.2);
        this->declare_parameter<double>("kLidarProjectionError", 0.5);

        // Basic segmentation parameters.
        this->declare_parameter<int>("kNumSectors", 180);
        this->declare_parameter<double>("kGroundYInterceptTolerance", 0.5);
        this->declare_parameter<double>("kGroundPointLineDistThres", 0.1);
        this->declare_parameter<int>("kWallLineMinBinNum", 3);
        this->declare_parameter<double>("kWallPointLineDistThres", 0.1);

        // Extrinsics (from base(ground) to sensor).
        this->declare_parameter<std::vector<double>>(
            "kExtrinsicTrans", std::vector<double>{0.0, 0.0, 0.25});
        this->declare_parameter<std::vector<double>>(
            "kExtrinsicRot", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        // Identify ground / wall (angles in degree, converted to slope below).
        this->declare_parameter<double>("kGroundSameLineTolerance", 2.0);
        this->declare_parameter<double>("kGroundSlopeTolerance", 10.0);
        this->declare_parameter<double>("kWallSameLineTolerance", 10.0);
        this->declare_parameter<double>("kWallSlopeTolerance", 75.0);

        // Read plain params (no conversion).
        sub_cloud_topic_ = this->get_parameter("sub_cloud_topic").as_string();
        pub_ground_cloud_topic_ = this->get_parameter("pub_ground_cloud_topic").as_string();
        pub_no_ground_cloud_topic_ = this->get_parameter("pub_no_ground_cloud_topic").as_string();
        sensor_frame_id_ = this->get_parameter("sensor_frame_id").as_string();

        params_.kLidarRows = this->get_parameter("kLidarRows").as_int();
        params_.kLidarCols = this->get_parameter("kLidarCols").as_int();
        params_.kNumSectors = this->get_parameter("kNumSectors").as_int();
        params_.kGroundYInterceptTolerance = static_cast<float>(
            this->get_parameter("kGroundYInterceptTolerance").as_double());
        params_.kGroundPointLineDistThres = static_cast<float>(
            this->get_parameter("kGroundPointLineDistThres").as_double());
        params_.kWallLineMinBinNum = this->get_parameter("kWallLineMinBinNum").as_int();
        params_.kWallPointLineDistThres = static_cast<float>(
            this->get_parameter("kWallPointLineDistThres").as_double());

        // Read angle params (degree -> rad).
        LoadAndConvertDegToRad("kLidarHorizRes", params_.kLidarHorizRes);
        LoadAndConvertDegToRad("kLidarVertRes", params_.kLidarVertRes);
        LoadAndConvertDegToRad("kLidarVertFovMax", params_.kLidarVertFovMax);
        LoadAndConvertDegToRad("kLidarVertFovMin", params_.kLidarVertFovMin);
        LoadAndConvertDegToRad("kLidarProjectionError", params_.kLidarProjectionError);

        // Read slope params (degree -> tan(rad)).
        LoadAndConvertDegToSlope("kGroundSameLineTolerance", params_.kGroundSameLineTolerance);
        LoadAndConvertDegToSlope("kGroundSlopeTolerance", params_.kGroundSlopeTolerance);
        LoadAndConvertDegToSlope("kWallSameLineTolerance", params_.kWallSameLineTolerance);
        LoadAndConvertDegToSlope("kWallSlopeTolerance", params_.kWallSlopeTolerance);

        // Extrinsics (double arrays -> Eigen float matrices, row-major).
        std::vector<double> ext_trans_vec = this->get_parameter("kExtrinsicTrans").as_double_array();
        std::vector<double> ext_rot_vec = this->get_parameter("kExtrinsicRot").as_double_array();
        if (ext_trans_vec.size() != 3 || ext_rot_vec.size() != 9)
            throw std::invalid_argument("Extrinsics require 3 translation and 9 rotation values");
        if (ext_trans_vec.size() == 3) {
            for (int i = 0; i < 3; ++i) {
                params_.kExtrinsicTrans(i) = static_cast<float>(ext_trans_vec[i]);
            }
        }
        if (ext_rot_vec.size() == 9) {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    params_.kExtrinsicRot(r, c) = static_cast<float>(ext_rot_vec[r * 3 + c]);
                }
            }
        }
    }

    void process(const ground_seg::Cloud::ConstSharedPtr& cloud,
                 const nav_msgs::msg::Odometry::ConstSharedPtr& pose)
    {
        try {
            Eigen::Isometry3d world_from_sensor = Eigen::Isometry3d::Identity();
            std::string output_frame = cloud->header.frame_id;
            if (output_frame.empty()) throw std::invalid_argument("Empty cloud frame");
            if (pose) {
                const auto expected_child = odometry_is_sensor_pose_ ? sensor_frame_id_ : body_frame_id_;
                if (pose->header.frame_id.empty() || pose->child_frame_id != expected_child ||
                    cloud->header.frame_id != (input_cloud_is_world_ ? pose->header.frame_id : sensor_frame_id_))
                    throw std::invalid_argument("Cloud/odometry frame mismatch");
                const auto& p = pose->pose.pose.position;
                const auto& q = pose->pose.pose.orientation;
                Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
                Eigen::Vector3d translation(p.x, p.y, p.z);
                if (!translation.allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-9)
                    throw std::invalid_argument("Invalid odometry pose");
                world_from_sensor.linear() = rotation.normalized().toRotationMatrix();
                world_from_sensor.translation() = translation;
                if (!odometry_is_sensor_pose_) {
                    Eigen::Isometry3d body_from_sensor = Eigen::Isometry3d::Identity();
                    body_from_sensor.linear() = params_.kExtrinsicRot.cast<double>();
                    body_from_sensor.translation() = params_.kExtrinsicTrans.cast<double>();
                    world_from_sensor = world_from_sensor * body_from_sensor;
                }
                output_frame = pose->header.frame_id;
            } else if (cloud->header.frame_id != sensor_frame_id_) {
                throw std::invalid_argument("Cloud-only input must match sensor_frame_id");
            }
            const ground_seg::CloudView view(*cloud);
            auto ground = view.empty(output_frame);
            auto nonground = view.empty(output_frame);
            pcl::PointCloud<pcl::PointXYZI>::Ptr local(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<PointXYZIRT>::Ptr ring_local(new pcl::PointCloud<PointXYZIRT>);
            std::vector<std::size_t> offsets;
            const auto sensor_from_world = world_from_sensor.inverse();
            for (uint32_t row = 0; row < cloud->height; ++row) {
                for (uint32_t column = 0; column < cloud->width; ++column) {
                    const std::size_t offset = static_cast<std::size_t>(row) * cloud->row_step +
                                               static_cast<std::size_t>(column) * cloud->point_step;
                    const Eigen::Vector3d original = view.point(offset);
                    const Eigen::Vector3d sensor = input_cloud_is_world_ ? sensor_from_world * original : original;
                    if (!sensor.allFinite() || !sensor.cast<float>().allFinite()) continue;
                    pcl::PointXYZI point;
                    point.x = sensor.x(); point.y = sensor.y(); point.z = sensor.z(); point.intensity = 0;
                    local->push_back(point);
                    if (view.ring) {
                        PointXYZIRT ring_point{};
                        ring_point.x = point.x; ring_point.y = point.y; ring_point.z = point.z;
                        ring_point.ring = view.read<uint16_t>(cloud->data.data() + offset + view.ring->offset);
                        ring_local->push_back(ring_point);
                    }
                    offsets.push_back(offset);
                }
            }
            std::vector<int> labels;
            if (view.ring) segmentation_->Segment(ring_local, &labels, false);
            else segmentation_->Segment(local, &labels, false);
            if (labels.size() != offsets.size()) throw std::runtime_error("Invalid segmentation label count");
            for (std::size_t i = 0; i < offsets.size(); ++i) {
                auto& output = labels[i] == 1 ? ground : nonground;
                if (pose && !input_cloud_is_world_) {
                    const Eigen::Vector3d world = world_from_sensor * view.point(offsets[i]);
                    if (!world.allFinite() || !world.cast<float>().allFinite()) continue;
                    view.append(output, offsets[i], &world);
                } else view.append(output, offsets[i]);
            }
            // Publish only after the complete pair has been validated and processed.
            if (pose) pose_pub_->publish(*pose);
            ground_pub_->publish(ground);
            nonground_pub_->publish(nonground);
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping cloud/pair: %s", error.what());
        }
    }

    using Sync = message_filters::TimeSynchronizer<ground_seg::Cloud, nav_msgs::msg::Odometry>;
    bool standalone_;
    bool input_cloud_is_world_{true}, odometry_is_sensor_pose_{false};
    int sync_queue_size_{30};
    std::string sub_cloud_topic_, odometry_topic_, pub_odometry_topic_;
    std::string pub_ground_cloud_topic_, pub_no_ground_cloud_topic_, sensor_frame_id_, body_frame_id_;
    SegmentationParams params_;
    std::unique_ptr<EfficientOnlineSegmentation> segmentation_;
    message_filters::Subscriber<ground_seg::Cloud> cloud_filter_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> pose_filter_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Subscription<ground_seg::Cloud>::SharedPtr cloud_sub_;
    rclcpp::Publisher<ground_seg::Cloud>::SharedPtr ground_pub_, nonground_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
};

std::shared_ptr<rclcpp::Node> makeGroundSeparationNode(bool standalone)
{
    return std::make_shared<GroundSeparationNode>(standalone);
}

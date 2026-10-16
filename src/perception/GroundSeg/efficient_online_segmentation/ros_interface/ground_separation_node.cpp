/*
 * Copyright 2024
 * ROS2 (Humble) interface: ground-vs-nonground separation.
 *
 * This node runs the same EfficientOnlineSegmentation core, but instead of the
 * visualization topics it only splits the input cloud into two outputs:
 *   - /ground_cloud    : points labelled as ground.
 *   - /no_ground_cloud : everything else (wall + unknown).
 *
 * The original per-point intensity is preserved (Segment is called with
 * use_intensity=false), so the output clouds carry the original reflectivity.
 */

#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "core/efficient_online_segmentation.h"

// Labels produced by EfficientOnlineSegmentation (see core/efficient_online_segmentation.h):
//   0 = unknown, 1 = ground, 2 = wall.
static constexpr int kGroundLabel = 1;

// ###########################################################################

class GroundSeparationNode : public rclcpp::Node
{
public:
    GroundSeparationNode()
    : Node("ground_separation")
    {
        common_original_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        custom_original_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
        ground_common_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        nonground_common_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        ground_custom_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
        nonground_custom_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());

        LoadAlgorithmParams();
        const bool is_params_legal = params_.UpdateInternalParams();
        if (!is_params_legal) {
            RCLCPP_ERROR(get_logger(),
                "###################### ERROR! ######################\n"
                "## Parameters illegal, please check!!!\n"
                "###################### ERROR! ######################");
            rclcpp::shutdown();
            return;
        }
        efficient_sgmtt_.ResetParameters(params_);

        source_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_cloud_topic_, 1,
            std::bind(&GroundSeparationNode::pointCloudCallback,
                      this, std::placeholders::_1));
        ground_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pub_ground_cloud_topic_, 1);
        no_ground_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pub_no_ground_cloud_topic_, 1);
    }

private:
    // ####################### Parameter loading #######################

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
        this->declare_parameter<std::string>("sub_cloud_topic", "/rslidar_points");
        this->declare_parameter<std::string>("pub_ground_cloud_topic", "/ground_cloud");
        this->declare_parameter<std::string>("pub_no_ground_cloud_topic", "/no_ground_cloud");
        this->declare_parameter<std::string>("base_link_frame_id", "base_link");
        this->declare_parameter<std::string>("sensor_frame_id", "rslidar");

        // LiDAR parameters (angles in degree, converted below).
        this->declare_parameter<int>("kLidarRows", 96);
        this->declare_parameter<int>("kLidarCols", 900);
        this->declare_parameter<double>("kLidarHorizRes", 0.4);
        this->declare_parameter<double>("kLidarVertRes", 0.92);
        this->declare_parameter<double>("kLidarVertFovMax", 88.0);
        this->declare_parameter<double>("kLidarVertFovMin", 0.0);
        this->declare_parameter<double>("kLidarProjectionError", 0.5);

        // Basic segmentation parameters.
        this->declare_parameter<int>("kNumSectors", 360);
        this->declare_parameter<double>("kGroundYInterceptTolerance", 0.5);
        this->declare_parameter<double>("kGroundPointLineDistThres", 0.1);
        this->declare_parameter<int>("kWallLineMinBinNum", 3);
        this->declare_parameter<double>("kWallPointLineDistThres", 0.1);

        // Extrinsics (from base(ground) to sensor).
        this->declare_parameter<std::vector<double>>(
            "kExtrinsicTrans", std::vector<double>{0.0, 0.0, 2.07});
        this->declare_parameter<std::vector<double>>(
            "kExtrinsicRot", std::vector<double>{1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0});

        // Identify ground / wall (angles in degree, converted to slope below).
        this->declare_parameter<double>("kGroundSameLineTolerance", 2.0);
        this->declare_parameter<double>("kGroundSlopeTolerance", 10.0);
        this->declare_parameter<double>("kWallSameLineTolerance", 10.0);
        this->declare_parameter<double>("kWallSlopeTolerance", 75.0);

        // Read plain params (no conversion).
        sub_cloud_topic_ = this->get_parameter("sub_cloud_topic").as_string();
        pub_ground_cloud_topic_ = this->get_parameter("pub_ground_cloud_topic").as_string();
        pub_no_ground_cloud_topic_ = this->get_parameter("pub_no_ground_cloud_topic").as_string();
        base_link_frame_id_ = this->get_parameter("base_link_frame_id").as_string();
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

    // ####################### Split helper #######################

    template <typename PointT>
    void SplitCloud(const typename pcl::PointCloud<PointT>::Ptr& in,
                    typename pcl::PointCloud<PointT>::Ptr& ground,
                    typename pcl::PointCloud<PointT>::Ptr& nonground)
    {
        ground->clear();
        nonground->clear();
        ground->reserve(in->size());
        nonground->reserve(in->size());
        for (std::size_t i = 0; i < in->size(); ++i) {
            if (labels_[i] == kGroundLabel) {
                ground->push_back(in->points[i]);
            } else {
                nonground->push_back(in->points[i]);
            }
        }
    }

    // ####################### Callback #######################

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Check cloud msg fields (once).
        static bool need_check_fields = true;
        static bool ring_field_exists = false;
        if (need_check_fields) {
            for (const auto& field : msg->fields) {
                if (field.name == "ring") {
                    ring_field_exists = true;
                    RCLCPP_INFO(get_logger(), "########## `ring` field exists -> ring path.");
                    break;
                }
            }
            if (!ring_field_exists) {
                RCLCPP_INFO(get_logger(),
                    "########## no `ring` field -> elevation-angle path.");
            }
            need_check_fields = false;
        }

        // Run ground segmentation. use_intensity=false keeps the original intensity.
        if (ring_field_exists) {
            pcl::fromROSMsg(*msg, *custom_original_cloud_);
            efficient_sgmtt_.Segment(custom_original_cloud_, &labels_, false);
            SplitCloud<PointXYZIRT>(custom_original_cloud_,
                                    ground_custom_cloud_, nonground_custom_cloud_);
        } else {
            pcl::fromROSMsg(*msg, *common_original_cloud_);
            efficient_sgmtt_.Segment(common_original_cloud_, &labels_, false);
            SplitCloud<pcl::PointXYZI>(common_original_cloud_,
                                       ground_common_cloud_, nonground_common_cloud_);
        }

        // Publish base_link -> sensor tf (so the result can also be viewed in base_link).
        {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = msg->header.stamp;
            tf_msg.header.frame_id = base_link_frame_id_;
            tf_msg.child_frame_id = sensor_frame_id_;
            Eigen::Quaternionf rot_quat(params_.kBaseToSensor.rotation());
            Eigen::Vector3f trans = params_.kBaseToSensor.translation();
            tf_msg.transform.rotation.x = rot_quat.x();
            tf_msg.transform.rotation.y = rot_quat.y();
            tf_msg.transform.rotation.z = rot_quat.z();
            tf_msg.transform.rotation.w = rot_quat.w();
            tf_msg.transform.translation.x = trans.x();
            tf_msg.transform.translation.y = trans.y();
            tf_msg.transform.translation.z = trans.z();
            tf_broadcaster_.sendTransform(tf_msg);
        }

        // Publish ground cloud.
        {
            sensor_msgs::msg::PointCloud2 out;
            if (ring_field_exists) {
                pcl::toROSMsg(*ground_custom_cloud_, out);
            } else {
                pcl::toROSMsg(*ground_common_cloud_, out);
            }
            out.header = msg->header;
            out.header.frame_id = sensor_frame_id_;
            ground_cloud_publisher_->publish(out);
        }

        // Publish non-ground cloud.
        {
            sensor_msgs::msg::PointCloud2 out;
            if (ring_field_exists) {
                pcl::toROSMsg(*nonground_custom_cloud_, out);
            } else {
                pcl::toROSMsg(*nonground_common_cloud_, out);
            }
            out.header = msg->header;
            out.header.frame_id = sensor_frame_id_;
            no_ground_cloud_publisher_->publish(out);
        }
    }

    // ####################### Members #######################

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_cloud_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_{*this};

    std::string sub_cloud_topic_;
    std::string pub_ground_cloud_topic_;
    std::string pub_no_ground_cloud_topic_;
    std::string base_link_frame_id_;
    std::string sensor_frame_id_;

    SegmentationParams params_;
    EfficientOnlineSegmentation efficient_sgmtt_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr common_original_cloud_;
    pcl::PointCloud<PointXYZIRT>::Ptr custom_original_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_common_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_common_cloud_;
    pcl::PointCloud<PointXYZIRT>::Ptr ground_custom_cloud_;
    pcl::PointCloud<PointXYZIRT>::Ptr nonground_custom_cloud_;
    std::vector<int> labels_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundSeparationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
 * Copyright 2021 Guanhua WANG
 * ROS2 (Humble) interface.
 */

#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "core/efficient_online_segmentation.h"

// ###########################################################################

class EfficientOnlineSegmentationNode : public rclcpp::Node
{
public:
    EfficientOnlineSegmentationNode()
    : Node("efficient_online_segmentation")
    {
        // System variables.
        common_original_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        custom_original_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());

        // Load parameters (defaults match segmentation_params.yaml).
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

        // ROS interface.
        source_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_cloud_topic_, 1,
            std::bind(&EfficientOnlineSegmentationNode::pointCloudCallback,
                      this, std::placeholders::_1));
        segmted_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pub_cloud_topic_, 1);
        range_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            pub_rangeimage_topic_, 1);
        extracted_lines_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            pub_extractedlines_topic_, 1);
        transformed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/EOS_transformed_cloud", 1);
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
        this->declare_parameter<std::string>("sub_cloud_topic", "/velodyne_points_0");
        this->declare_parameter<std::string>("pub_cloud_topic", "/EOS_segmted_cloud");
        this->declare_parameter<std::string>("pub_rangeimage_topic", "/EOS_range_image");
        this->declare_parameter<std::string>("pub_extractedlines_topic", "/EOS_extracted_lines");
        this->declare_parameter<std::string>("base_link_frame_id", "base_link");
        this->declare_parameter<std::string>("sensor_frame_id", "velodyne");

        // LiDAR parameters (angles in degree, converted below).
        this->declare_parameter<int>("kLidarRows", 32);
        this->declare_parameter<int>("kLidarCols", 2169);
        this->declare_parameter<double>("kLidarHorizRes", 0.166);
        this->declare_parameter<double>("kLidarVertRes", 1.333);
        this->declare_parameter<double>("kLidarVertFovMax", 10.67);
        this->declare_parameter<double>("kLidarVertFovMin", -30.67);
        this->declare_parameter<double>("kLidarProjectionError", 0.5);

        // Basic segmentation parameters.
        this->declare_parameter<int>("kNumSectors", 360);
        this->declare_parameter<double>("kGroundYInterceptTolerance", 0.5);
        this->declare_parameter<double>("kGroundPointLineDistThres", 0.1);
        this->declare_parameter<int>("kWallLineMinBinNum", 3);
        this->declare_parameter<double>("kWallPointLineDistThres", 0.1);

        // Extrinsics (from base(ground) to sensor).
        this->declare_parameter<std::vector<double>>(
            "kExtrinsicTrans", std::vector<double>{0.0, 0.0, 1.832});
        this->declare_parameter<std::vector<double>>(
            "kExtrinsicRot", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        // Identify ground / wall (angles in degree, converted to slope below).
        this->declare_parameter<double>("kGroundSameLineTolerance", 2.0);
        this->declare_parameter<double>("kGroundSlopeTolerance", 10.0);
        this->declare_parameter<double>("kWallSameLineTolerance", 10.0);
        this->declare_parameter<double>("kWallSlopeTolerance", 75.0);

        // Read plain params (no conversion).
        sub_cloud_topic_ = this->get_parameter("sub_cloud_topic").as_string();
        pub_cloud_topic_ = this->get_parameter("pub_cloud_topic").as_string();
        pub_rangeimage_topic_ = this->get_parameter("pub_rangeimage_topic").as_string();
        pub_extractedlines_topic_ = this->get_parameter("pub_extractedlines_topic").as_string();
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

    // ####################### Callback #######################

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Check cloud msg fields.
        static bool need_check_fields = true;
        static bool ring_field_exists = false;
        static bool time_field_exists = false;
        if (need_check_fields) {
            for (const auto& field : msg->fields) {
                if (field.name == "ring") {
                    ring_field_exists = true;
                    std::cout << "########## Congrats, `ring` field exists!" << std::endl;
                    continue;
                }
                if (field.name == "time" || field.name == "t") {
                    time_field_exists = true;
                    std::cout << "########## Congrats, `time` field exists!" << std::endl;
                    continue;
                }
            }
            if (!ring_field_exists) {
                std::cout << "########## Could not found `ring` field. " << std::endl;
            }
            if (!time_field_exists) {
                std::cout << "########## Could not found `time` field. " << std::endl;
            }
            need_check_fields = false;
        }

        // Run ground-segmentation algorithm.
        if (ring_field_exists) {
            pcl::fromROSMsg(*msg, *custom_original_cloud_);
            efficient_sgmtt_.Segment(custom_original_cloud_, &labels_, true);
        }
        else {
            pcl::fromROSMsg(*msg, *common_original_cloud_);
            efficient_sgmtt_.Segment(common_original_cloud_, &labels_, true);
        }

        // Publish robot tf.
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

        // Visualize results.
        if (segmted_cloud_publisher_->get_subscription_count() != 0) {
            sensor_msgs::msg::PointCloud2 cloudMsgTemp;
            if (common_original_cloud_->size() > 0) {
                pcl::toROSMsg(*common_original_cloud_, cloudMsgTemp);
            }
            else {
                pcl::toROSMsg(*custom_original_cloud_, cloudMsgTemp);
            }
            cloudMsgTemp.header = msg->header;
            cloudMsgTemp.header.frame_id = sensor_frame_id_;
            segmted_cloud_publisher_->publish(cloudMsgTemp);
        }

        if (transformed_cloud_publisher_->get_subscription_count() != 0) {
            sensor_msgs::msg::PointCloud2 cloudMsgTemp;
            if (ring_field_exists) {
                pcl::toROSMsg(*efficient_sgmtt_.GetTransformedCustomCloud(), cloudMsgTemp);
            }
            else {
                pcl::toROSMsg(*efficient_sgmtt_.GetTransformedCommonCloud(), cloudMsgTemp);
            }
            cloudMsgTemp.header = msg->header;
            cloudMsgTemp.header.frame_id = base_link_frame_id_;
            transformed_cloud_publisher_->publish(cloudMsgTemp);
        }

        if (range_image_publisher_->get_subscription_count() != 0) {
            cv::Mat imgTmp1, imgTmp2;
            imgTmp1 = efficient_sgmtt_.GetRangeImage();
            cv::normalize(imgTmp1, imgTmp2, 255, 0, cv::NORM_MINMAX);
            imgTmp2.convertTo(imgTmp1, CV_8UC1);  // from CV_32FC1 to CV_8UC1
            cv::flip(imgTmp1, imgTmp2, 0);
            cv::applyColorMap(imgTmp2, imgTmp1, cv::COLORMAP_RAINBOW);
            cv::resize(imgTmp1, imgTmp2, cv::Size(imgTmp1.cols / 3, imgTmp1.rows));
            sensor_msgs::msg::Image::SharedPtr imgMsgTemp =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgTmp2).toImageMsg();
            range_image_publisher_->publish(*imgMsgTemp);
        }

        if (extracted_lines_publisher_->get_subscription_count() != 0) {
            visualization_msgs::msg::MarkerArray lines_array;
            const auto& geometry_lines = efficient_sgmtt_.GetExtractedLines();
            if (!geometry_lines.empty()) {
                float kEdgeScale = 0.05;
                visualization_msgs::msg::Marker edge;
                edge.header = msg->header;
                edge.header.frame_id = base_link_frame_id_;
                edge.action = visualization_msgs::msg::Marker::ADD;
                edge.ns = "EOS_lines";
                edge.id = 0;
                edge.type = visualization_msgs::msg::Marker::LINE_STRIP;
                edge.scale.x = kEdgeScale;
                edge.scale.y = kEdgeScale;
                edge.scale.z = kEdgeScale;
                edge.color.r = 0.0;
                edge.color.g = 1.0;
                edge.color.b = 1.0;
                edge.color.a = 1.0;
                geometry_msgs::msg::Point pStart;
                geometry_msgs::msg::Point pEnd;
                int id = 0;
                for (const auto& curr_line : geometry_lines) {
                    edge.points.clear();
                    edge.id = id;
                    pStart.x = curr_line.start_point.x;
                    pStart.y = curr_line.start_point.y;
                    pStart.z = curr_line.start_point.z;
                    edge.points.push_back(pStart);
                    pEnd.x = curr_line.end_point.x;
                    pEnd.y = curr_line.end_point.y;
                    pEnd.z = curr_line.end_point.z;
                    edge.points.push_back(pEnd);
                    if (curr_line.label == LineLabel::GROUND) {
                        edge.scale.x = kEdgeScale;
                        edge.scale.y = kEdgeScale;
                        edge.scale.z = kEdgeScale;
                        edge.color.r = 1.0;
                        edge.color.g = 0.8;
                        edge.color.b = 0.0;
                    }
                    else if (curr_line.label == LineLabel::WALL) {
                        edge.scale.x = 2 * kEdgeScale;
                        edge.scale.y = 2 * kEdgeScale;
                        edge.scale.z = 2 * kEdgeScale;
                        edge.color.r = 0.0;
                        edge.color.g = 1.0;
                        edge.color.b = 0.0;
                    }
                    lines_array.markers.push_back(edge);
                    id++;
                }
            }
            extracted_lines_publisher_->publish(lines_array);
        }
    }

    // ####################### Members #######################

    // Ros tools.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmted_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr range_image_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr extracted_lines_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_{*this};

    std::string base_link_frame_id_;
    std::string sensor_frame_id_;
    std::string sub_cloud_topic_;
    std::string pub_cloud_topic_;
    std::string pub_rangeimage_topic_;
    std::string pub_extractedlines_topic_;

    // System variables.
    SegmentationParams params_;
    EfficientOnlineSegmentation efficient_sgmtt_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr common_original_cloud_;
    pcl::PointCloud<PointXYZIRT>::Ptr custom_original_cloud_;
    std::vector<int> labels_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EfficientOnlineSegmentationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

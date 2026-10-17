#include <gtest/gtest.h>

#include "core/efficient_online_segmentation.h"

TEST(SegmentationStartup, ConstructWithDefaultParameters)
{
    // Both ROS nodes construct the core before loading their YAML parameters.
    EfficientOnlineSegmentation segmentation;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> labels{42};
    segmentation.Segment(cloud, &labels, false);
    EXPECT_TRUE(labels.empty());
}

TEST(SegmentationStartup, ResetToGazeboParametersAndProcessBothPointTypes)
{
    EfficientOnlineSegmentation segmentation;
    SegmentationParams params;
    params.kLidarRows = 50;
    params.kLidarCols = 360;
    params.kNumSectors = 180;
    params.kLidarHorizRes = M_PI / 180.0;
    params.kLidarVertRes = 1.273 * M_PI / 180.0;
    params.kLidarVertFovMin = -7.2 * M_PI / 180.0;
    params.kLidarVertFovMax = 55.2 * M_PI / 180.0;
    params.kExtrinsicTrans.z() = 0.25;
    ASSERT_TRUE(params.UpdateInternalParams());
    segmentation.ResetParameters(params);

    pcl::PointCloud<pcl::PointXYZI>::Ptr common(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZIRT>::Ptr custom(new pcl::PointCloud<PointXYZIRT>);
    for (int i = 0; i < 20; ++i) {
        pcl::PointXYZI point;
        point.x = 3.0f + i * 0.1f;
        point.y = 0.0f;
        point.z = -0.25f;
        point.intensity = 37.0f;
        common->push_back(point);
        PointXYZIRT ring_point{};
        ring_point.x = point.x;
        ring_point.y = point.y;
        ring_point.z = point.z;
        ring_point.intensity = point.intensity;
        ring_point.ring = i;
        custom->push_back(ring_point);
    }

    std::vector<int> labels;
    segmentation.Segment(common, &labels, false);
    EXPECT_EQ(labels.size(), common->size());
    for (const auto& point : *common) {
        EXPECT_FLOAT_EQ(point.intensity, 37.0f);
    }
    segmentation.Segment(custom, &labels, false);
    EXPECT_EQ(labels.size(), custom->size());
    for (const auto& point : *custom) {
        EXPECT_FLOAT_EQ(point.intensity, 37.0f);
    }

    // Empty frames after valid frames must not retain the previous results.
    common->clear();
    segmentation.Segment(common, &labels, false);
    EXPECT_TRUE(labels.empty());
    EXPECT_TRUE(segmentation.GetTransformedCommonCloud()->empty());
    custom->clear();
    segmentation.Segment(custom, &labels, false);
    EXPECT_TRUE(labels.empty());
    EXPECT_TRUE(segmentation.GetTransformedCustomCloud()->empty());
}

TEST(SegmentationStartup, SinglePointVisualizationDoesNotWritePastCloud)
{
    EfficientOnlineSegmentation segmentation;
    pcl::PointCloud<pcl::PointXYZI>::Ptr common(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZIRT>::Ptr custom(new pcl::PointCloud<PointXYZIRT>);
    pcl::PointXYZI point;
    point.x = 3.0f;
    point.y = 0.0f;
    point.z = 0.0f;
    point.intensity = 37.0f;
    common->push_back(point);
    PointXYZIRT ring_point{};
    ring_point.x = point.x;
    ring_point.intensity = point.intensity;
    custom->push_back(ring_point);
    std::vector<int> labels;
    segmentation.Segment(common, &labels, true);
    EXPECT_EQ(labels.size(), 1u);
    EXPECT_FLOAT_EQ(common->front().intensity, 180.0f);
    segmentation.Segment(custom, &labels, true);
    EXPECT_EQ(labels.size(), 1u);
    EXPECT_FLOAT_EQ(custom->front().intensity, 180.0f);
}

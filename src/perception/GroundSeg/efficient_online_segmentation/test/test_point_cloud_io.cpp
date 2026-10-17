#include <gtest/gtest.h>
#include "ros_interface/point_cloud_io.h"

TEST(PointCloudIO, PreservesFieldsPaddingAndBothByteOrders)
{
    for (bool bigendian : {false, true}) {
        ground_seg::Cloud cloud;
        cloud.header.frame_id = "odom";
        cloud.width = 1; cloud.height = 2; cloud.point_step = 32; cloud.row_step = 40;
        cloud.is_bigendian = bigendian;
        cloud.data.resize(80, 0xab);
        for (int i = 0; i < 3; ++i) {
            ground_seg::Field f;
            f.name = std::array<const char*, 3>{{"x", "y", "z"}}[i];
            f.offset = 8*i; f.datatype = ground_seg::Field::FLOAT64; f.count = 1;
            cloud.fields.push_back(f);
        }
        ground_seg::CloudView view(cloud);
        view.write<double>(cloud.data.data() + 40, 1.25);
        view.write<double>(cloud.data.data() + 48, -2.5);
        view.write<double>(cloud.data.data() + 56, 3.0);
        EXPECT_NEAR(view.point(40).x(), 1.25, 1e-12);
        auto out = view.empty("odom");
        view.append(out, 40);
        EXPECT_EQ(out.height, 1u);
        EXPECT_EQ(out.width, 1u);
        EXPECT_EQ(out.row_step, 32u);
        EXPECT_EQ(out.fields, cloud.fields);
        EXPECT_EQ(out.data, std::vector<uint8_t>(cloud.data.begin() + 40, cloud.data.begin() + 72));
        const Eigen::Vector3d world(7, 8, 9);
        auto transformed = view.empty("world");
        view.append(transformed, 40, &world);
        ground_seg::CloudView transformed_view(transformed);
        EXPECT_NEAR((transformed_view.point(0) - world).norm(), 0, 1e-12);
        EXPECT_EQ(transformed.data.back(), 0xab);
    }
}

TEST(PointCloudIO, RejectsMalformedLayoutsBeforeReadingMemory)
{
    ground_seg::Cloud cloud;
    EXPECT_THROW(ground_seg::CloudView view(cloud), std::invalid_argument);
    cloud.point_step = 12; cloud.width = 1; cloud.height = 1; cloud.row_step = 12;
    for (int i = 0; i < 3; ++i) {
        ground_seg::Field f;
        f.name = std::array<const char*, 3>{{"x", "y", "z"}}[i];
        f.offset = 4*i; f.datatype = ground_seg::Field::FLOAT32; f.count = 1;
        cloud.fields.push_back(f);
    }
    EXPECT_THROW(ground_seg::CloudView view(cloud), std::invalid_argument);
    cloud.data.resize(12);
    EXPECT_NO_THROW(ground_seg::CloudView view(cloud));
    cloud.fields[2].offset = 12;
    EXPECT_THROW(ground_seg::CloudView view(cloud), std::invalid_argument);
}

#include <gtest/gtest.h>
#include <limits>
#include <plan_manage/warm_start_path.h>

using Eigen::Vector3d;
using scan_planner::resampleWarmStartPath;

TEST(WarmStartPath, PreservesDetourAndMeasuredStart)
{
  const std::vector<Vector3d> route{{0.1, 0.1, 0.0}, {1.0, 1.0, 0.0},
                                   {2.0, 1.0, 0.0}, {3.0, 0.0, 0.0}};
  std::vector<Vector3d> sampled;
  ASSERT_TRUE(resampleWarmStartPath(route, 0.2, 100, sampled));
  EXPECT_TRUE(sampled.front().isApprox(route.front()));
  EXPECT_TRUE(sampled.back().isApprox(route.back()));
  double max_y = 0.0;
  for (const auto &point : sampled)
  {
    EXPECT_GE(point.y(), 0.0);
    max_y = std::max(max_y, point.y());
  }
  EXPECT_GT(max_y, 0.9);
}

TEST(WarmStartPath, HandlesDuplicatesAndShortRemainingRoutes)
{
  std::vector<Vector3d> sampled;
  ASSERT_TRUE(resampleWarmStartPath({Vector3d::Zero(), Vector3d::Zero(),
                                    Vector3d(0.01, 0.0, 0.0)}, 0.2, 100, sampled));
  EXPECT_EQ(sampled.size(), 7u);
  for (const auto &point : sampled)
    EXPECT_TRUE(point.allFinite());
}

TEST(WarmStartPath, RejectsDegenerateOrUnboundedSampling)
{
  std::vector<Vector3d> sampled;
  EXPECT_FALSE(resampleWarmStartPath({}, 0.2, 100, sampled));
  EXPECT_FALSE(resampleWarmStartPath({Vector3d::Zero(), Vector3d::Zero()}, 0.2, 100, sampled));
  EXPECT_FALSE(resampleWarmStartPath({Vector3d::Zero(), Vector3d::UnitX()}, 0.0, 100, sampled));
  EXPECT_FALSE(resampleWarmStartPath({Vector3d::Zero(), Vector3d::UnitX()}, 1e-12, 100, sampled));
  EXPECT_FALSE(resampleWarmStartPath({Vector3d::Zero(),
      Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)}, 0.2, 100, sampled));
  EXPECT_TRUE(sampled.empty());
}

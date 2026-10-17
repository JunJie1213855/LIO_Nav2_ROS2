#include <gtest/gtest.h>
#include "plan_manage/car_path_tracker.h"

using namespace scan_planner;

static std::vector<CarPathSample> line(double angle = 0.0)
{
  std::vector<CarPathSample> path;
  for (int i = 0; i <= 100; ++i)
  {
    CarPathSample p;
    p.position = Eigen::Vector2d(std::cos(angle), std::sin(angle)) * (i * 0.03);
    p.time = i * 0.06;
    p.speed = 0.5;
    path.push_back(p);
  }
  return path;
}

TEST(CarPathTracker, CorrectsLateralErrorAndReachesAngledGoal)
{
  for (double angle : {0.0, M_PI / 4.0, M_PI / 2.0})
  {
    CarPathTracker tracker;
    const auto path = line(angle);
    ASSERT_TRUE(tracker.setPath(path));
    Eigen::Vector2d position(0.0, 0.35);
    double yaw = 0.0, previous_progress = 0.0;
    bool reached = false;
    for (int i = 0; i < 3000; ++i)
    {
      const auto cmd = tracker.update(position, yaw, 0.02);
      ASSERT_FALSE(cmd.tracking_lost);
      EXPECT_GE(cmd.linear, 0.0);
      EXPECT_LE(cmd.linear, 0.5);
      EXPECT_LE(std::abs(cmd.angular), 1.0);
      EXPECT_GE(cmd.progress_time + 1e-9, previous_progress);
      previous_progress = cmd.progress_time;
      position += 0.02 * cmd.linear * Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
      yaw += 0.02 * cmd.angular;
      if (cmd.reached_end) { reached = true; break; }
    }
    EXPECT_TRUE(reached) << "angle=" << angle;
    EXPECT_LT((position - path.back().position).norm(), 0.11);
  }
}

TEST(CarPathTracker, TracksCurveWithUnicycleKinematics)
{
  CarPathTracker tracker;
  std::vector<CarPathSample> path;
  for (int i = 0; i <= 200; ++i)
  {
    const double theta = i * (M_PI / 2.0) / 200.0;
    CarPathSample p;
    p.position = Eigen::Vector2d(std::sin(theta), 1.0 - std::cos(theta));
    p.time = theta / 0.5;
    p.speed = 0.5;
    path.push_back(p);
  }
  ASSERT_TRUE(tracker.setPath(path));
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double yaw = 0.0, max_error = 0.0;
  bool reached = false;
  for (int i = 0; i < 2000; ++i)
  {
    auto cmd = tracker.update(position, yaw, 0.02);
    ASSERT_FALSE(cmd.tracking_lost);
    max_error = std::max(max_error, cmd.cross_track_error);
    position += 0.02 * cmd.linear * Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
    yaw += 0.02 * cmd.angular;
    if (cmd.reached_end) { reached = true; break; }
  }
  EXPECT_TRUE(reached);
  EXPECT_LT(max_error, 0.15);
}

TEST(CarPathTracker, StopsOnLostTrackingAndDoesNotAdvanceStationaryProgress)
{
  CarPathTracker tracker;
  ASSERT_TRUE(tracker.setPath(line()));
  const auto before = tracker.update(Eigen::Vector2d::Zero(), 0.0, 0.02);
  for (int i = 0; i < 200; ++i)
    EXPECT_DOUBLE_EQ(tracker.update(Eigen::Vector2d::Zero(), 0.0, 0.02).progress_time,
                      before.progress_time);
  const auto lost = tracker.update(Eigen::Vector2d(0.0, 2.0), 0.0, 0.02);
  EXPECT_TRUE(lost.tracking_lost);
  EXPECT_DOUBLE_EQ(lost.linear, 0.0);
  EXPECT_DOUBLE_EQ(lost.angular, 0.0);
}

TEST(CarPathTracker, EmergencyHoldNeverDrivesAndRotationHasNoForwardCommand)
{
  CarPathTracker tracker;
  ASSERT_TRUE(tracker.setPath(line(M_PI / 2.0)));
  const auto rotate = tracker.update(Eigen::Vector2d::Zero(), 0.0, 0.02);
  EXPECT_TRUE(rotate.rotating);
  EXPECT_DOUBLE_EQ(rotate.linear, 0.0);
  EXPECT_GT(rotate.angular, 0.0);
  auto hold = line();
  for (auto &point : hold) point.position = Eigen::Vector2d(3.0, 3.0);
  ASSERT_TRUE(tracker.setPath(hold));
  const auto stopped = tracker.update(Eigen::Vector2d::Zero(), 0.0, 0.02);
  EXPECT_DOUBLE_EQ(stopped.linear, 0.0);
  EXPECT_DOUBLE_EQ(stopped.angular, 0.0);
  EXPECT_FALSE(stopped.reached_end);
}

TEST(CarPathTracker, RejectsInvalidConfigurationAndPath)
{
  CarTrackerConfig config;
  config.max_vx = 0.0;
  EXPECT_THROW(CarPathTracker{config}, std::invalid_argument);
  CarPathTracker tracker;
  EXPECT_FALSE(tracker.setPath({}));
  auto path = line();
  path[1].time = path[0].time;
  EXPECT_FALSE(tracker.setPath(path));
}

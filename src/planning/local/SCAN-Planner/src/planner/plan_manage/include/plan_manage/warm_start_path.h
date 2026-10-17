#ifndef SCAN_PLANNER_WARM_START_PATH_H_
#define SCAN_PLANNER_WARM_START_PATH_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace scan_planner
{
// Resample the remaining route without losing its chosen side of an obstacle.
// Degenerate paths must fail instead of underflowing an index or looping forever.
inline bool resampleWarmStartPath(const std::vector<Eigen::Vector3d> &path,
                                  double spacing, std::size_t max_points,
                                  std::vector<Eigen::Vector3d> &sampled)
{
  sampled.clear();
  if (!std::isfinite(spacing) || spacing <= 0.0 || max_points < 7)
    return false;
  std::vector<Eigen::Vector3d> points;
  std::vector<double> lengths;
  for (const auto &point : path)
  {
    if (!point.allFinite())
      return false;
    if (points.empty())
    {
      points.push_back(point);
      lengths.push_back(0.0);
    }
    else
    {
      const double distance = (point - points.back()).norm();
      if (distance > 1e-9)
      {
        points.push_back(point);
        lengths.push_back(lengths.back() + distance);
      }
    }
  }
  if (points.size() < 2 || !std::isfinite(lengths.back()))
    return false;
  const double intervals = std::max(6.0, std::ceil(lengths.back() / spacing));
  if (intervals > static_cast<double>(max_points - 1))
    return false;
  const auto count = static_cast<std::size_t>(intervals);
  std::size_t segment = 0;
  sampled.reserve(count + 1);
  for (std::size_t i = 0; i <= count; ++i)
  {
    const double distance = lengths.back() * static_cast<double>(i) / count;
    while (segment + 2 < points.size() && lengths[segment + 1] < distance)
      ++segment;
    const double ratio = (distance - lengths[segment]) /
                         (lengths[segment + 1] - lengths[segment]);
    sampled.push_back((1.0 - ratio) * points[segment] + ratio * points[segment + 1]);
  }
  sampled.front() = points.front();
  sampled.back() = points.back();
  return true;
}
}  // namespace scan_planner

#endif

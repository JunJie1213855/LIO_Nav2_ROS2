#ifndef SCAN_PLANNER_CAR_PATH_TRACKER_H_
#define SCAN_PLANNER_CAR_PATH_TRACKER_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace scan_planner
{
struct CarTrackerConfig
{
  double max_vx{0.5}, max_vyaw{1.0};
  double max_accel{0.5}, max_decel{0.8}, max_yaw_accel{2.0};
  double max_lateral_accel{0.4};
  double lookahead_min{0.25}, lookahead_max{0.6}, lookahead_time{0.6};
  double rotate_enter{0.7}, rotate_exit{0.25}, kp_rotate{1.5};
  double finish_dist{0.1}, max_tracking_error{0.8}, progress_search_distance{1.0};
};

struct CarPathSample
{
  Eigen::Vector2d position;
  double time{0.0}, speed{0.0}, arc{0.0};
};

struct CarControl
{
  double linear{0.0}, angular{0.0}, progress_time{0.0}, cross_track_error{0.0};
  bool reached_end{false}, tracking_lost{false}, rotating{false};
};

// Pure pursuit on an arc-length path. No lateral velocity command is generated.
class CarPathTracker
{
public:
  explicit CarPathTracker(const CarTrackerConfig &config = CarTrackerConfig()) : config_(config)
  {
    for (double value : {config.max_vx, config.max_vyaw, config.max_accel, config.max_decel,
                         config.max_yaw_accel, config.max_lateral_accel, config.lookahead_min,
                         config.lookahead_max, config.kp_rotate, config.finish_dist,
                         config.max_tracking_error, config.progress_search_distance})
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Car tracker limits and distances must be positive and finite");
    if (!std::isfinite(config.lookahead_time) || config.lookahead_time < 0.0 ||
        config.lookahead_min > config.lookahead_max || !std::isfinite(config.rotate_enter) ||
        !std::isfinite(config.rotate_exit) || config.rotate_exit < 0.0 ||
        config.rotate_exit >= config.rotate_enter || config.rotate_enter >= M_PI / 2.0)
      throw std::invalid_argument("Invalid car lookahead or rotation hysteresis parameters");
  }

  bool setPath(std::vector<CarPathSample> path)
  {
    path_.clear();
    progress_arc_ = 0.0;
    rotating_ = false;
    if (path.size() < 2)
      return false;
    double arc = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
      if (!path[i].position.allFinite() || !std::isfinite(path[i].time) ||
          !std::isfinite(path[i].speed) ||
          path[i].speed < 0.0 || path[i].time < 0.0 ||
          (i && path[i].time <= path[i - 1].time))
        return false;
      if (i)
        arc += (path[i].position - path[i - 1].position).norm();
      if (!std::isfinite(arc))
        return false;
      path[i].arc = arc;
    }
    path_ = std::move(path);
    return true;
  }

  void stop() { last_v_ = last_w_ = 0.0; }

  CarControl update(const Eigen::Vector2d &position, double yaw, double dt)
  {
    CarControl out;
    if (path_.empty() || !position.allFinite() || !std::isfinite(yaw) ||
        !std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
    {
      stop();
      return out;
    }
    const double goal_distance = (path_.back().position - position).norm();
    // Constant B-splines are emergency holds, never commands to drive to a point.
    if (path_.back().arc < 1e-5)
    {
      out.reached_end = goal_distance <= config_.finish_dist;
      out.rotating = !out.reached_end;
      out.progress_time = out.reached_end ? path_.back().time : 0.0;
      stop();
      return out;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    double best_arc = progress_arc_;
    const double search_end = progress_arc_ + config_.progress_search_distance;
    for (std::size_t i = 1; i < path_.size(); ++i)
    {
      const auto &a = path_[i - 1];
      const auto &b = path_[i];
      if (b.arc < progress_arc_ || b.arc - a.arc < 1e-9)
        continue;
      if (a.arc > search_end)
        break;
      const Eigen::Vector2d delta = b.position - a.position;
      const double lower = std::clamp((progress_arc_ - a.arc) / (b.arc - a.arc), 0.0, 1.0);
      const double upper = std::clamp((search_end - a.arc) / (b.arc - a.arc), lower, 1.0);
      const double ratio = std::clamp((position - a.position).dot(delta) / delta.squaredNorm(),
                                      lower, upper);
      const double distance = (position - a.position - ratio * delta).norm();
      if (distance < best_distance)
      {
        best_distance = distance;
        best_arc = a.arc + ratio * (b.arc - a.arc);
      }
    }
    progress_arc_ = best_arc;
    out.cross_track_error = best_distance;
    out.progress_time = atArc(progress_arc_).time;
    if (best_distance > config_.max_tracking_error)
    {
      out.tracking_lost = true;
      stop();
      return out;
    }
    const double remaining = path_.back().arc - progress_arc_;
    if (remaining <= 2.0 * config_.finish_dist && goal_distance <= config_.finish_dist)
    {
      out.reached_end = true;
      out.progress_time = path_.back().time;
      stop();
      return out;
    }

    const double lookahead = std::clamp(config_.lookahead_min + config_.lookahead_time * last_v_,
                                       config_.lookahead_min, config_.lookahead_max);
    const auto target = atArc(std::min(progress_arc_ + lookahead, path_.back().arc));
    const Eigen::Vector2d delta = target.position - position;
    const double x = std::cos(yaw) * delta.x() + std::sin(yaw) * delta.y();
    const double y = -std::sin(yaw) * delta.x() + std::cos(yaw) * delta.y();
    const double heading_error = std::atan2(y, x);
    if (rotating_)
      rotating_ = std::abs(heading_error) > config_.rotate_exit;
    else
      rotating_ = std::abs(heading_error) > config_.rotate_enter;
    if (rotating_)
    {
      last_v_ = 0.0;
      last_w_ = slew(std::clamp(config_.kp_rotate * heading_error,
                               -config_.max_vyaw, config_.max_vyaw), last_w_,
                     config_.max_yaw_accel * dt);
      out.angular = last_w_;
      out.rotating = true;
      return out;
    }

    const double curvature = 2.0 * y / std::max(delta.squaredNorm(), 1e-6);
    // Regulate the arc the car will execute, not reference a/v^2 near a zero-speed endpoint.
    const double limit_curvature = std::abs(curvature);
    double speed = std::min(config_.max_vx, std::max(0.08, target.speed));
    if (limit_curvature > 1e-6)
      speed = std::min({speed, config_.max_vyaw / limit_curvature,
                        std::sqrt(config_.max_lateral_accel / limit_curvature)});
    speed *= std::max(0.0, std::cos(heading_error)) / (1.0 + best_distance);
    if (remaining < 2.0 * lookahead)
      speed = std::min({speed, std::sqrt(2.0 * config_.max_decel * goal_distance),
                        0.8 * goal_distance});
    speed = slew(speed, last_v_, (speed > last_v_ ? config_.max_accel : config_.max_decel) * dt);
    if (std::abs(curvature) > 1e-6)
      speed = std::min(speed, config_.max_vyaw / std::abs(curvature));
    const double desired_w = std::clamp(speed * curvature, -config_.max_vyaw, config_.max_vyaw);
    last_w_ = slew(desired_w, last_w_, config_.max_yaw_accel * dt);
    // Slow down during yaw acceleration rather than turning a wider arc than requested.
    if (std::abs(curvature) > 1e-5)
      speed = last_w_ * curvature > 0.0 ? std::min(speed, std::abs(last_w_ / curvature)) : 0.0;
    last_v_ = std::max(0.0, speed);
    out.linear = last_v_;
    out.angular = last_w_;
    return out;
  }

private:
  static double slew(double value, double previous, double step)
  { return std::clamp(value, previous - step, previous + step); }

  CarPathSample atArc(double arc) const
  {
    auto end = std::lower_bound(path_.begin(), path_.end(), arc,
        [](const CarPathSample &sample, double value) { return sample.arc < value; });
    if (end == path_.begin()) return path_.front();
    if (end == path_.end()) return path_.back();
    const auto &a = *(end - 1);
    const double ratio = (arc - a.arc) / std::max(end->arc - a.arc, 1e-9);
    CarPathSample out;
    out.position = a.position + ratio * (end->position - a.position);
    out.time = a.time + ratio * (end->time - a.time);
    out.speed = a.speed + ratio * (end->speed - a.speed);
    return out;
  }

  CarTrackerConfig config_;
  std::vector<CarPathSample> path_;
  double progress_arc_{0.0}, last_v_{0.0}, last_w_{0.0};
  bool rotating_{false};
};
}  // namespace scan_planner
#endif

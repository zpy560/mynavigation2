#ifndef NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_START_SPEED_GUARD_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_START_SPEED_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace nav2_regulated_modules
{
namespace detail
{

struct StartPathPoint
{
  StartPathPoint(const double point_x, const double point_y)
  : x(point_x), y(point_y)
  {
  }

  double x;
  double y;
};

inline bool requiresStartPrebraking(const double measured_speed, const double speed_limit)
{
  return !std::isfinite(measured_speed) || std::abs(measured_speed) > speed_limit;
}

// 从当前进度向前选取最近的非零长度路径段，避免弯道仍使用起点切线。
inline bool calculateStartRecoveryErrors(const std::vector<StartPathPoint> & path, const std::size_t nearest_index, const double robot_x, const double robot_y, const double vehicle_yaw, const int direction_sign, double & lateral_error, double & heading_error, double * progress = nullptr)
{
  if (path.size() < 2 || !std::isfinite(robot_x) || !std::isfinite(robot_y) || !std::isfinite(vehicle_yaw) || (direction_sign != 1 && direction_sign != -1))
  {
    return false;
  }

  constexpr double pi = 3.14159265358979323846;
  const std::size_t first_segment = std::min(nearest_index > 0 ? nearest_index - 1 : 0, path.size() - 2);
  double best_distance_squared = std::numeric_limits<double>::infinity();
  bool found = false;
  double prefix_length = 0.0;
  auto consider_segment = [&](const std::size_t index)
  {
    const auto & start = path[index];
    const auto & end = path[index + 1];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (!std::isfinite(length_squared) || length_squared <= 1e-12)
    {
      return;
    }
    const double offset_x = robot_x - start.x;
    const double offset_y = robot_y - start.y;
    const double projection = std::clamp((offset_x * dx + offset_y * dy) / length_squared, 0.0, 1.0);
    const double closest_x = start.x + projection * dx;
    const double closest_y = start.y + projection * dy;
    const double distance_squared = (robot_x - closest_x) * (robot_x - closest_x) + (robot_y - closest_y) * (robot_y - closest_y);
    if (!std::isfinite(distance_squared) || distance_squared >= best_distance_squared)
    {
      return;
    }
    best_distance_squared = distance_squared;
    lateral_error = std::abs(dx * offset_y - dy * offset_x) / std::sqrt(length_squared);
    const double path_yaw = std::atan2(dy, dx);
    const double motion_yaw = vehicle_yaw + (direction_sign < 0 ? pi : 0.0);
    const double yaw_difference = path_yaw - motion_yaw;
    heading_error = std::abs(std::atan2(std::sin(yaw_difference), std::cos(yaw_difference)));
    if (progress)
    {
      *progress = prefix_length + projection * std::sqrt(length_squared);
    }
    found = true;
  };

  for (std::size_t index = 0; index + 1 < path.size(); ++index)
  {
    if (index >= first_segment)
    {
      consider_segment(index);
    }
    prefix_length += std::hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
  }
  // 终点附近若剩余路径段全为重复点，回退到最后一个可用切线。
  if (!found)
  {
    prefix_length = 0.0;
    for (std::size_t index = 0; index < first_segment; ++index)
    {
      consider_segment(index);
      prefix_length += std::hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
    }
  }
  return found && (!progress || std::isfinite(*progress));
}

// 每条新路径统一限速；距离与误差同时达标后单向解除。
class FixedPathStartSpeedGuard
{
public:
  void reset()
  {
    evaluated_ = false;
    active_ = false;
    stable_cycles_ = 0;
    initial_progress_ = std::numeric_limits<double>::quiet_NaN();
  }

  bool evaluateStart()
  {
    if (!evaluated_)
    {
      evaluated_ = true;
      active_ = true;
    }
    return active_;
  }

  bool observe(const double lateral_error, const double heading_error, const double lateral_tolerance, const double heading_tolerance, const int required_stable_cycles, const double progress, const double protected_distance)
  {
    if (!active_)
    {
      return false;
    }
    if (!std::isfinite(progress) || progress < 0.0)
    {
      resetStability();
      return false;
    }
    if (!std::isfinite(initial_progress_))
    {
      initial_progress_ = progress;
    }
    if (progress < std::max(initial_progress_ + protected_distance, protected_distance) || !std::isfinite(lateral_error) || !std::isfinite(heading_error) || lateral_error > lateral_tolerance || heading_error > heading_tolerance)
    {
      stable_cycles_ = 0;
      return false;
    }
    ++stable_cycles_;
    if (stable_cycles_ < required_stable_cycles)
    {
      return false;
    }
    active_ = false;
    return true;
  }

  void resetStability()
  {
    stable_cycles_ = 0;
  }

  bool active() const
  {
    return active_;
  }

private:
  bool evaluated_ = false;
  bool active_ = false;
  int stable_cycles_ = 0;
  double initial_progress_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace detail
}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__DETAIL__FIXED_PATH_START_SPEED_GUARD_HPP_

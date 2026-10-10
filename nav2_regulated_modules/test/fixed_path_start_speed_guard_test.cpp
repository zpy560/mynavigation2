#include <cmath>
#include <iostream>
#include <vector>

#include "nav2_regulated_modules/detail/fixed_path_start_speed_guard.hpp"

namespace
{

bool check(const bool condition, const char * message)
{
  if (!condition)
  {
    std::cerr << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  using nav2_regulated_modules::detail::FixedPathStartSpeedGuard;
  using nav2_regulated_modules::detail::StartPathPoint;
  using nav2_regulated_modules::detail::calculateStartRecoveryErrors;
  constexpr double pi = 3.14159265358979323846;
  constexpr double release_yaw = pi / 9.0;
  using nav2_regulated_modules::detail::requiresStartPrebraking;
  if (!check(requiresStartPrebraking(1.5, 0.3) && requiresStartPrebraking(-1.5, 0.3) && requiresStartPrebraking(0.21, 0.2) && requiresStartPrebraking(std::numeric_limits<double>::quiet_NaN(), 0.3) && !requiresStartPrebraking(0.3, 0.3) && !requiresStartPrebraking(-0.3, 0.3), "moving tasks and invalid feedback must prebrake; boundary speeds must pass"))
  {
    return 1;
  }
  FixedPathStartSpeedGuard guard;
  if (!check(guard.evaluateStart(), "every new path must trigger the start cap"))
  {
    return 1;
  }
  guard.reset();
  if (!check(guard.evaluateStart(), "a new path must trigger the start cap"))
  {
    return 1;
  }
  for (int cycle = 0; cycle < 20; ++cycle)
  {
    if (!check(!guard.observe(0.0, 0.0, 0.20, release_yaw, 5, cycle == 0 ? 0.0 : 0.499, 0.5), "small errors must not release before the protected distance"))
    {
      return 1;
    }
  }
  for (int cycle = 0; cycle < 4; ++cycle)
  {
    if (!check(!guard.observe(0.20, release_yaw, 0.20, release_yaw, 5, 0.5, 0.5), "release must wait for five stable cycles"))
    {
      return 1;
    }
  }
  if (!check(!guard.observe(0.201, 0.0, 0.20, release_yaw, 5, 0.5, 0.5), "lateral deviation must reset the stable count"))
  {
    return 1;
  }
  if (!check(!guard.observe(0.10, release_yaw + 0.001, 0.20, release_yaw, 5, 0.5, 0.5), "heading deviation must reset the stable count"))
  {
    return 1;
  }
  for (int cycle = 0; cycle < 4; ++cycle)
  {
    if (!check(!guard.observe(0.20, release_yaw, 0.20, release_yaw, 5, 0.5, 0.5), "cap released too early"))
    {
      return 1;
    }
  }
  if (!check(guard.observe(0.20, release_yaw, 0.20, release_yaw, 5, 0.5, 0.5) && !guard.active(), "cap must release on the fifth stable cycle"))
  {
    return 1;
  }
  if (!check(!guard.evaluateStart() && !guard.active(), "released cap must not re-arm on the same path"))
  {
    return 1;
  }
  guard.reset();
  if (!check(guard.evaluateStart(), "a new path must reset the start cap"))
  {
    return 1;
  }
  guard.observe(0.0, 0.0, 0.20, release_yaw, 1, 0.8, 0.5);
  if (!check(!guard.observe(0.0, 0.0, 0.20, release_yaw, 1, 1.299, 0.5), "starting ahead must still protect another 0.5 m"))
  {
    return 1;
  }
  if (!check(guard.observe(0.0, 0.0, 0.20, release_yaw, 1, 1.3, 0.5), "starting ahead must release after another 0.5 m"))
  {
    return 1;
  }
  guard.reset();
  guard.evaluateStart();
  guard.observe(0.0, 0.0, 0.20, release_yaw, 2, 0.0, 0.5);
  guard.observe(0.0, 0.0, 0.20, release_yaw, 2, 0.5, 0.5);
  guard.observe(0.0, 0.0, 0.20, release_yaw, 2, std::numeric_limits<double>::quiet_NaN(), 0.5);
  if (!check(!guard.observe(0.0, 0.0, 0.20, release_yaw, 2, 0.5, 0.5), "invalid progress must reset stability"))
  {
    return 1;
  }
  guard.observe(0.0, 0.0, 0.20, release_yaw, 2, 0.49, 0.5);
  if (!check(!guard.observe(0.0, 0.0, 0.20, release_yaw, 2, 0.5, 0.5), "returning into the protected zone must reset stability"))
  {
    return 1;
  }

  std::vector<StartPathPoint> curve;
  curve.emplace_back(0.0, 0.0);
  curve.emplace_back(1.0, 0.0);
  curve.emplace_back(1.0, 1.0);
  double lateral_error = 0.0;
  double heading_error = 0.0;
  double progress = 0.0;
  if (!check(calculateStartRecoveryErrors(curve, 1, 1.1, 0.4, pi / 2.0, 1, lateral_error, heading_error, &progress) && std::abs(lateral_error - 0.1) < 1e-9 && heading_error < 1e-9 && std::abs(progress - 1.4) < 1e-9, "forward recovery must project onto the nearest curved segment"))
  {
    return 1;
  }
  if (!check(calculateStartRecoveryErrors(curve, 1, 1.1, 0.4, -pi / 2.0, -1, lateral_error, heading_error) && std::abs(lateral_error - 0.1) < 1e-9 && heading_error < 1e-9, "reverse recovery must compare the vehicle backward axis"))
  {
    return 1;
  }
  std::vector<StartPathPoint> repeated_end;
  repeated_end.emplace_back(0.0, 0.0);
  repeated_end.emplace_back(1.0, 0.0);
  repeated_end.emplace_back(1.0, 0.0);
  if (!check(calculateStartRecoveryErrors(repeated_end, 2, 0.5, 0.1, 0.0, 1, lateral_error, heading_error) && std::abs(lateral_error - 0.1) < 1e-9, "a repeated terminal point must fall back to a valid segment"))
  {
    return 1;
  }
  std::vector<StartPathPoint> invalid_path;
  invalid_path.emplace_back(0.0, 0.0);
  invalid_path.emplace_back(0.0, 0.0);
  if (!check(!calculateStartRecoveryErrors(invalid_path, 0, 0.0, 0.0, 0.0, 1, lateral_error, heading_error), "a path without a valid tangent must not release the cap"))
  {
    return 1;
  }
  return 0;
}

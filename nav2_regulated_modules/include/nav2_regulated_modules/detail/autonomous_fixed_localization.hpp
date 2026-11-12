#ifndef NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_LOCALIZATION_HPP_
#define NAV2_REGULATED_MODULES__DETAIL__AUTONOMOUS_FIXED_LOCALIZATION_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

namespace nav2_regulated_modules::detail
{
// 只放宽已起步且远离段终点的跟踪；新任务和终点验收不使用延迟窗口。
inline double fixedLocalizationTimeout(const bool initialized, const bool terminal, const double goal_distance, const double strict_timeout, const double tracking_timeout, const double terminal_distance, const double maximum_speed, const double capture_margin)
{
  const double strict_distance = std::max(terminal_distance, maximum_speed * tracking_timeout + capture_margin);
  return initialized && !terminal && std::isfinite(goal_distance) && goal_distance > strict_distance ? tracking_timeout : strict_timeout;
}

struct FixedLocalizationPauseTiming
{
  std::optional<double> paused_since;
  std::optional<double> invalid_since;

  void reset()
  {
    paused_since.reset();
    invalid_since.reset();
  }

  // 有效数据清除连续失效计时，但暂停总时长继续覆盖恢复稳定等待。
  bool update(const bool valid, const double time, const double invalid_timeout)
  {
    if (!std::isfinite(time))
    {
      return true;
    }
    if (!paused_since)
    {
      paused_since = time;
    }
    if (time < *paused_since)
    {
      return true;
    }
    if (valid)
    {
      invalid_since.reset();
      return false;
    }
    if (!invalid_since)
    {
      invalid_since = time;
    }
    return time < *invalid_since || time - *invalid_since >= invalid_timeout;
  }

  double duration(const double time) const
  {
    return paused_since ? std::max(0.0, time - *paused_since) : 0.0;
  }
};
}

#endif

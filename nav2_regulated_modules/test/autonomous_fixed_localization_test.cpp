#include "nav2_regulated_modules/detail/autonomous_fixed_localization.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void require(const bool value, const char * message)
{
  if (!value)
  {
    throw std::runtime_error(message);
  }
}
}

int main()
{
  using nav2_regulated_modules::detail::fixedLocalizationTimeout;
  using nav2_regulated_modules::detail::FixedLocalizationPauseTiming;
  require(fixedLocalizationTimeout(true, false, 2.0, 0.3, 0.6, 0.5, 0.5, 0.02) == 0.6, "ordinary tracking tolerates bounded TF delay");
  require(fixedLocalizationTimeout(false, false, 2.0, 0.3, 0.6, 0.5, 0.5, 0.02) == 0.3, "initial localization must be fresh");
  require(fixedLocalizationTimeout(true, true, 2.0, 0.3, 0.6, 0.5, 0.5, 0.02) == 0.3, "terminal and rotation review stays strict");
  require(fixedLocalizationTimeout(true, false, 0.5, 0.3, 0.6, 0.5, 0.5, 0.02) == 0.3, "terminal zone boundary stays strict");
  require(fixedLocalizationTimeout(true, false, 0.8, 0.3, 0.6, 0.5, 1.5, 0.02) == 0.3, "higher speed expands the strict terminal zone");
  require(fixedLocalizationTimeout(true, false, std::numeric_limits<double>::quiet_NaN(), 0.3, 0.6, 0.5, 0.5, 0.02) == 0.3, "invalid geometry cannot relax localization");
  FixedLocalizationPauseTiming pause;
  require(!pause.update(false, 10.0, 0.5), "short failure starts a pause");
  require(!pause.update(false, 10.4, 0.5), "failure below limit retains the Action");
  require(!pause.update(true, 10.45, 0.5), "fresh data clears only consecutive failure timing");
  require(!pause.update(true, 10.61, 0.5) && pause.duration(10.61) > 0.6, "recorded recovery beyond total 0.5s does not force cancellation");
  require(!pause.update(false, 10.7, 0.5), "a new interruption starts its own invalid interval");
  require(!pause.update(false, 11.19, 0.5), "prior pause history does not consume the next failure window");
  require(pause.update(false, 11.21, 0.5), "continuous actual loss still requires full recovery");
  pause.reset();
  require(!pause.paused_since && !pause.invalid_since, "completed recovery clears timing");
  require(!pause.update(false, 20.0, 0.5) && pause.update(false, 19.0, 0.5), "backward monotonic time cannot resume motion");
  pause.reset();
  require(pause.update(false, std::numeric_limits<double>::quiet_NaN(), 0.5), "invalid time is rejected");
  std::cout << "PASS autonomous_fixed bounded TF delay and consecutive-loss timing\n";
}

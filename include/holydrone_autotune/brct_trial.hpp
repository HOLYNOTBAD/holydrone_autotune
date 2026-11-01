#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
namespace brct {
// Clock supplied by caller; flight timing uses a monotonic clock, not /clock.
struct Pulse {
  int repetition = -1;
  int phase = -1;
  double sign = 0;
  bool finished = false;
};
inline Pulse pulse(double elapsed, double settle, double duration, int repetitions) {
  const std::array<double, 7> lengths{{settle, duration, duration, settle, duration, duration, 1.0}};
  const std::array<double, 7> signs{{0, 1, -1, 0, -1, 1, 0}};
  for (int r = 0; r < repetitions; ++r) {
    for (int p = 0; p < 7; ++p) {
      if (elapsed < lengths[p]) return {r, p, signs[p], false};
      elapsed -= lengths[p];
    }
  }
  return {-1, -1, 0, true};
}
inline bool matches(double actual, double requested) {
  return std::isfinite(actual) && std::abs(actual-requested) <= 1e-7 + 1e-5*std::abs(requested);
}
}  // namespace brct

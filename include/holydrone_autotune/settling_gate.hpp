#pragma once
#include <cmath>

namespace brct {
inline bool freshSample(double sample_age, double receipt_age, double sample_limit, double wall_limit, double lead_tolerance = 0) {
  return std::isfinite(sample_age) && std::isfinite(receipt_age) &&
    sample_age >= -lead_tolerance && sample_age < sample_limit && receipt_age >= 0 && receipt_age < wall_limit;
}

// 用连续的新反馈计时：重复样本、暂停和丢包都不能累积稳定时间。
class SettlingGate {
  double since_ = -1, last_ = -1;
 public:
  void reset() { since_ = last_ = -1; }
  void observe(double stamp, bool stable, double max_gap) {
    if (!std::isfinite(stamp) || stamp < 0 || !stable) { reset(); return; }
    if (last_ >= 0 && stamp <= last_) { reset(); return; }
    if (last_ < 0 || stamp-last_ > max_gap) since_ = stamp;
    last_ = stamp;
  }
  bool ready(double duration) const {
    return since_ >= 0 && last_-since_ >= duration;
  }
};
}

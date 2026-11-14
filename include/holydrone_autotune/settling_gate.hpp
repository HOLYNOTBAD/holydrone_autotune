#pragma once
#include <cmath>
#include <Eigen/Core>

namespace brct {
inline bool freshSample(double sample_age, double receipt_age, double sample_limit, double wall_limit, double lead_tolerance = 0) {
  return std::isfinite(sample_age) && std::isfinite(receipt_age) &&
    sample_age >= -lead_tolerance && sample_age < sample_limit && receipt_age >= 0 && receipt_age < wall_limit;
}

// 用于回中和零角速度交接判稳。先滤各轴再求模，避免零均值高频波动在求模后变成正偏置。
// 使用源时间戳；断流/倒退后从当前测量重新开始，不沿用旧的“静止”状态。
class AngularRateFilter {
  double last_ = -1;
  Eigen::Vector3d value_ = Eigen::Vector3d::Zero();
 public:
  void reset() { last_ = -1; }
  double observe(double stamp, const Eigen::Vector3d& rate, double tau, double max_gap) {
    const double dt = stamp-last_;
    if (last_ < 0 || dt <= 0 || dt > max_gap) value_ = rate;
    else value_ += -std::expm1(-dt/tau)*(rate-value_);
    last_ = stamp;
    return value_.norm();
  }
};

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

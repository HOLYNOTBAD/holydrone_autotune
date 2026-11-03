#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace brct {
inline double bound(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }

// ENU：高度和垂直速度向上为正。输出是归一化总推力，不是 PWM。
struct AltitudeControl {
  double kp = 2.0, kd = 2.5, ki = 0.4;
  double acceleration_limit = 3.0, integral_limit = 1.0;
  double min_thrust = 0.05, max_thrust = 0.8;
  double integral = 0;

  double update(double hover, double error, double vz, double cos_tilt, double dt) {
    for (double x : {hover, error, vz, cos_tilt, dt})
      if (!std::isfinite(x)) throw std::runtime_error("invalid altitude feedback");
    if (cos_tilt <= 0 || hover <= 0 || hover > 1 || dt < 0)
      throw std::runtime_error("invalid thrust/tilt/time");
    const double next = bound(integral + ki*error*std::min(dt, .05), -integral_limit, integral_limit);
    const double raw_acc = kp*error - kd*vz + next;
    const double acc = bound(raw_acc, -acceleration_limit, acceleration_limit);
    const double raw_thrust = hover*(1.0+acc/9.80665)/cos_tilt;
    // 加速度或推力限幅时，仅允许能使输出脱离饱和的积分方向。
    const bool high = raw_acc > acceleration_limit || raw_thrust > max_thrust;
    const bool low = raw_acc < -acceleration_limit || raw_thrust < min_thrust;
    if ((!high && !low) || (high && error < 0) || (low && error > 0)) integral = next;
    return bound(raw_thrust, min_thrust, max_thrust);
  }
};
}  // namespace brct

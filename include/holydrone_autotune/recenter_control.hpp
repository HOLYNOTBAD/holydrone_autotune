#pragma once
#include <Eigen/Geometry>
#include <holydrone_autotune/altitude_control.hpp>

namespace brct {
// 全程使用 ENU 位姿、FLU 机体系角速度；不切换 PX4 Offboard 控制类型。
// 小角度位置/速度反馈只用于回中，激励期间保持原始的开环速率输入。
struct RecenterControl {
  double position_gain = .8, velocity_gain = 1.8, attitude_gain = 4.0;
  double max_tilt_deg = 12, max_rate = 2.0;

  Eigen::Vector3d rates(const Eigen::Quaterniond& attitude, const Eigen::Vector3d& position_error,
                        const Eigen::Vector3d& velocity, double yaw) const {
    Eigen::Vector2d a = position_gain*position_error.head<2>() - velocity_gain*velocity.head<2>();
    const double limit = 9.80665*std::tan(max_tilt_deg*std::acos(-1.0)/180);
    if (a.norm() > limit) a *= limit/a.norm();
    // 推力方向为机体 +Z：正 pitch 向机体 +X 加速，正 roll 向 -Y 加速。
    const double ax = std::cos(yaw)*a.x()+std::sin(yaw)*a.y();
    const double ay = -std::sin(yaw)*a.x()+std::cos(yaw)*a.y();
    const double roll = -std::atan2(ay, 9.80665);
    const double pitch = std::atan2(ax, std::hypot(9.80665, ay));
    const Eigen::Quaterniond desired = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    const Eigen::Quaterniond error = attitude.normalized().conjugate()*desired;
    Eigen::Vector3d rate = 2*attitude_gain*(error.w() < 0 ? -1 : 1)*error.vec();
    if (rate.norm() > max_rate) rate *= max_rate/rate.norm();
    return rate;
  }
};
}  // namespace brct

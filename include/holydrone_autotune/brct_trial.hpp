/**
 * @file brct_trial.hpp
 * @brief 提供参数读回容差判断。
 *
 * @details 独立于 ROS；激励时序由 FSM 根据运动反馈推进。
 */
#pragma once
#include <cmath>
namespace brct {
/** @brief 允许 1e-7 绝对误差加 1e-5 相对误差；非有限读回值不通过。 */
inline bool matches(double actual, double requested) {
  return std::isfinite(actual) && std::abs(actual-requested) <= 1e-7 + 1e-5*std::abs(requested);
}
}  // namespace brct

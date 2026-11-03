/**
 * @file test_brct_trial.cpp
 * @brief 验证读回容差与高度控制。
 *
 * @details 纯 C++ 单元测试，无需 ROS Master、仿真或飞控。
 */
#include <holydrone_autotune/brct_trial.hpp>
#include <holydrone_autotune/altitude_control.hpp>
#include <gtest/gtest.h>
#include <holydrone_autotune/settling_gate.hpp>
TEST(Trial, ReadbackTolerance) {
  EXPECT_TRUE(brct::matches(.003000000026, .003));
  EXPECT_FALSE(brct::matches(.0031, .003));
  EXPECT_FALSE(brct::matches(NAN, .003));
  EXPECT_FALSE(brct::matches(INFINITY, .003));
}
TEST(Altitude, HoverTiltAndFeedbackDirection) {
  brct::AltitudeControl c;
  EXPECT_NEAR(c.update(.34, 0, 0, 1, .005), .34, 1e-8);
  EXPECT_NEAR(c.update(.34, 0, 0, std::cos(M_PI/6), .005), .34/std::cos(M_PI/6), 1e-8);
  EXPECT_GT(c.update(.34, .3, -.2, 1, .005), .34);
  EXPECT_LT(c.update(.34, -.3, .2, 1, .005), .34);
}
TEST(Altitude, LimitsAndNoWindup) {
  brct::AltitudeControl c;
  c.max_thrust = .4;
  for (int i = 0; i < 1000; ++i) EXPECT_LE(c.update(.34, 10, -5, .71, .005), .4);
  EXPECT_EQ(c.integral, 0);
  EXPECT_THROW(c.update(.34, NAN, 0, 1, .005), std::runtime_error);
  EXPECT_THROW(c.update(.34, 0, 0, -1, .005), std::runtime_error);
}
TEST(Altitude, VerticalPlantRecoversRepeatedTiltPulses) {
  brct::AltitudeControl c;
  double z = 3, vz = 0, worst = 0;
  // 以不同于捕获推力的实际悬停推力模拟建模误差，并施加反复 30 度倾斜。
  for (int i = 0; i < 12000; ++i) {
    const double tilt = i%700 < 50 ? std::cos(M_PI/6) : 1;
    const double thrust = c.update(.34, 3-z, vz, tilt, .005);
    vz += (9.80665*(thrust*tilt/.35-1))*.005;
    z += vz*.005;
    worst = std::max(worst, std::abs(z-3));
  }
  EXPECT_LT(worst, .3);
  EXPECT_NEAR(z, 3, .05);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Settling, RequiresContinuousFreshStableFeedback) {
  brct::SettlingGate gate;
  // 姿态水平但角速度振荡：任一峰值越界就重新计时。
  for (int i = 0; i < 200; ++i) {
    gate.observe(i*.01, i%5 != 0, .1);
    EXPECT_FALSE(gate.ready(.3));
  }
  gate.reset();
  for (int i = 0; i <= 31; ++i) gate.observe(3+i*.01, true, .1);
  EXPECT_TRUE(gate.ready(.3));
  gate.observe(3.31, true, .1);  // 重复时间戳不能保持已达标状态。
  EXPECT_FALSE(gate.ready(.3));
  gate.observe(4, true, .1); gate.observe(4.5, true, .1);
  EXPECT_FALSE(gate.ready(.3));  // 丢包区间不算稳定时间。
  gate.observe(4.51, false, .1);
  EXPECT_FALSE(gate.ready(.3));
  gate.observe(NAN, true, .1);
  EXPECT_FALSE(gate.ready(.3));
}

TEST(Settling, SimulationPauseIsNotStaleFlightState) {
  EXPECT_TRUE(brct::freshSample(.016, .103, .1, .5));
  EXPECT_FALSE(brct::freshSample(.103, .103, .1, .5));
  EXPECT_FALSE(brct::freshSample(.016, .6, .1, .5));
  EXPECT_FALSE(brct::freshSample(-.01, .01, .1, .5));
  EXPECT_FALSE(brct::freshSample(NAN, .01, .1, .5));
}

TEST(Settling, SharedSimulationClockAllowsOnlyBoundedMessageOrderingLead) {
  EXPECT_TRUE(brct::freshSample(-.004, .001, .1, .5, .008));
  EXPECT_FALSE(brct::freshSample(-.009, .001, .1, .5, .008));
  EXPECT_FALSE(brct::freshSample(-.004, .001, .1, .5));  // 实机不启用此容差。
  EXPECT_FALSE(brct::freshSample(.11, .001, .1, .5, .008));
}

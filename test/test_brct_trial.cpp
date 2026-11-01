#include <holydrone_autotune/brct_trial.hpp>
#include <gtest/gtest.h>
TEST(Trial, SequenceAndRepetition) {
  const double times[] = {0.1, .8, 1.1, 1.5, 2.1, 2.3, 2.8, 3.6};
  const double signs[] = {0, 1, -1, 0, -1, 1, 0, 0};
  for (int i = 0; i < 8; ++i) {
    auto p = brct::pulse(times[i], .75, .25, 2);
    EXPECT_EQ(p.sign, signs[i]); EXPECT_EQ(p.phase, i%7); EXPECT_EQ(p.repetition, i/7);
  }
  EXPECT_TRUE(brct::pulse(7.01, .75, .25, 2).finished);
}
TEST(Trial, ReadbackTolerance) {
  EXPECT_TRUE(brct::matches(.003000000026, .003));
  EXPECT_FALSE(brct::matches(.0031, .003));
  EXPECT_FALSE(brct::matches(NAN, .003));
  EXPECT_FALSE(brct::matches(INFINITY, .003));
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

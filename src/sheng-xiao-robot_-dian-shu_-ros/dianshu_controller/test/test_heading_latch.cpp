#include <gtest/gtest.h>
#include "dianshu_controller/heading_latch.hpp"
using dianshu_controller::HeadingLatch;
TEST(HeadingLatch, PositivePiBranchDoesNotReverse) {
 HeadingLatch h;
 EXPECT_GT(h.step(3.13, .1, .25, .4, .04), 0);
 EXPECT_GT(h.step(-3.13, .1, .25, .4, .04), 0);
 EXPECT_GT(h.step(-2.9, .1, .25, .4, .04), 0);
}
TEST(HeadingLatch, NegativePiBranchDoesNotReverse) {
 HeadingLatch h;
 EXPECT_LT(h.step(-3.13, .1, .25, .4, .04), 0);
 EXPECT_LT(h.step(3.13, .1, .25, .4, .04), 0);
}
TEST(HeadingLatch, TurnsSmoothlyAndStopsWithoutReversing) {
 HeadingLatch h;
 double error=3.14, previous=0;
 for (int i=0;i<1000;i++) {
  const double w=h.step(error,.1,.25,.4,.04);
  EXPECT_GE(w,0); EXPECT_LE(w,.25); EXPECT_LE(std::abs(w-previous),.040001);
  error-=w*.1;previous=w;
 }
 EXPECT_NEAR(error,.04,1e-6); EXPECT_NEAR(previous,0,1e-6);
}
TEST(HeadingLatch, OvershootStopsRatherThanTakingLongWay) {
 HeadingLatch h;
 h.step(.1,.1,.25,.4,.04);
 EXPECT_DOUBLE_EQ(h.step(-.02,.1,.25,.4,.04),0);
 EXPECT_DOUBLE_EQ(h.step(-.03,.1,.25,.4,.04),0);
}
TEST(HeadingLatch, NewGoalResetCanChooseOppositeDirection) {
 HeadingLatch h;
 h.step(2,.1,.25,.4,.04);h.reset();
 EXPECT_LT(h.step(-2,.1,.25,.4,.04),0);
}

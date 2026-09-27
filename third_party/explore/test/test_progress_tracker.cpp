#include <gtest/gtest.h>
#include <explore/progress_tracker.h>

TEST(ProgressTracker, SlowMovementOutlivesBothOldTimeouts) {
  explore::ProgressTracker p;
  p.reset(0, 0, 0, 10);
  for (int t = 1; t <= 180; ++t) {
    p.update(t, t * 0.01, 0, 10 - t * 0.01, 12 - t * 0.01, 0.15);
    EXPECT_FALSE(p.stalled(t, 30));
    EXPECT_FALSE(p.expired(t, 0));
  }
  EXPECT_GT(p.last_progress, 160);
}
TEST(ProgressTracker, StationaryReplanningIsNotProgress) {
  explore::ProgressTracker p;
  p.reset(0, 0, 0, 10);
  for (int t = 1; t <= 31; ++t)
    EXPECT_FALSE(p.update(t, 0, 0, 10, 50 - t, 0.15));
  EXPECT_TRUE(p.stalled(31, 30));
}
TEST(ProgressTracker, OscillationDoesNotResetHighWaterMark) {
  explore::ProgressTracker p;
  p.reset(0, 0, 0, 10);
  for (int t = 1; t <= 31; ++t) {
    double x = (t % 2) ? -0.5 : 0.0;
    EXPECT_FALSE(p.update(t, x, 0, 10 - x, 10 - x, 0.15));
  }
  EXPECT_TRUE(p.stalled(31, 30));
}
TEST(ProgressTracker, DetourCanProgressAlongPathWithoutEuclideanImprovement) {
  explore::ProgressTracker p;
  p.reset(0, 0, 0, 2);
  p.update(1, 0, 0, 2, 10, 0.15);
  EXPECT_TRUE(p.update(20, -0.2, 0, 2.2, 9.8, 0.15));
  EXPECT_FALSE(p.stalled(40, 30));
  EXPECT_TRUE(p.stalled(50, 30));
}
TEST(ProgressTracker, OptionalTotalLimitAndClockReset) {
  explore::ProgressTracker p;
  p.reset(100, 0, 0, 2);
  EXPECT_FALSE(p.expired(1000, 0));
  EXPECT_TRUE(p.expired(200, 100));
  p.update(1, 0, 0, 2, 2, 0.15);
  EXPECT_FALSE(p.stalled(1, 30));
}

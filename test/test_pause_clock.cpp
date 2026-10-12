#include <gtest/gtest.h>

#include "rsf_rviz_plugins/pause_clock.hpp"

namespace
{
rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

int64_t ns(const rclcpp::Time & time)
{
  return time.nanoseconds();
}
}

TEST(PauseClock, FollowsNowWhileRunning)
{
  rsf_rviz_plugins::PauseClock clock;
  EXPECT_FALSE(clock.paused());
  EXPECT_EQ(ns(clock.motionTime(at(10.0))), ns(at(10.0)));
}

TEST(PauseClock, FreezesWhilePaused)
{
  rsf_rviz_plugins::PauseClock clock;
  clock.setPaused(true, at(10.0));
  EXPECT_TRUE(clock.paused());
  EXPECT_EQ(ns(clock.motionTime(at(15.0))), ns(at(10.0)));
}

TEST(PauseClock, ResumesFromFrozenTime)
{
  rsf_rviz_plugins::PauseClock clock;
  clock.setPaused(true, at(10.0));
  clock.setPaused(false, at(15.0));
  EXPECT_FALSE(clock.paused());
  EXPECT_EQ(ns(clock.motionTime(at(17.0))), ns(at(12.0)));
}

TEST(PauseClock, AccumulatesRepeatedPauses)
{
  rsf_rviz_plugins::PauseClock clock;
  clock.setPaused(true, at(10.0));
  clock.setPaused(false, at(15.0));
  clock.setPaused(true, at(18.0));
  clock.setPaused(false, at(20.0));
  EXPECT_EQ(ns(clock.motionTime(at(21.0))), ns(at(14.0)));
}

TEST(PauseClock, IgnoresSettingTheSameState)
{
  rsf_rviz_plugins::PauseClock clock;
  clock.setPaused(true, at(10.0));
  clock.setPaused(true, at(12.0));
  EXPECT_EQ(ns(clock.motionTime(at(15.0))), ns(at(10.0)));
  clock.setPaused(false, at(15.0));
  clock.setPaused(false, at(16.0));
  EXPECT_EQ(ns(clock.motionTime(at(17.0))), ns(at(12.0)));
}

TEST(PauseClock, ResetDropsAccumulatedPause)
{
  rsf_rviz_plugins::PauseClock clock;
  clock.setPaused(true, at(10.0));
  clock.setPaused(false, at(15.0));
  clock.reset(at(3.0));
  EXPECT_EQ(ns(clock.motionTime(at(4.0))), ns(at(4.0)));
}

TEST(PauseClock, ResetWhilePausedFreezesAtResetTime)
{
  rsf_rviz_plugins::PauseClock clock;
  clock.setPaused(true, at(10.0));
  clock.reset(at(3.0));
  EXPECT_TRUE(clock.paused());
  EXPECT_EQ(ns(clock.motionTime(at(4.0))), ns(at(3.0)));
}

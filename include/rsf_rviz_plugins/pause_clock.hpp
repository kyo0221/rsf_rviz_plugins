#pragma once

#include <cstdint>
#include <rclcpp/time.hpp>

namespace rsf_rviz_plugins
{
class PauseClock
{
public:
  void setPaused(bool paused, const rclcpp::Time & now)
  {
    if (paused == paused_) {
      return;
    }
    if (paused) {
      pause_start_ = now.nanoseconds();
    } else {
      paused_total_ += now.nanoseconds() - pause_start_;
    }
    paused_ = paused;
  }

  bool paused() const
  {
    return paused_;
  }

  rclcpp::Time motionTime(const rclcpp::Time & now) const
  {
    const int64_t base = paused_ ? pause_start_ : now.nanoseconds();
    return rclcpp::Time(base - paused_total_, now.get_clock_type());
  }

  void reset(const rclcpp::Time & now)
  {
    pause_start_ = now.nanoseconds();
    paused_total_ = 0;
  }

private:
  bool paused_{false};
  int64_t pause_start_{0};
  int64_t paused_total_{0};
};
}

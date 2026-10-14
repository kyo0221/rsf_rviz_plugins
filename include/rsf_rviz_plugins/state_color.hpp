#pragma once

#include <atomic>
#include <string>
#include <QColor>

namespace rsf_rviz_plugins
{
inline const QColor kGood(46, 204, 113);
inline const QColor kWarn(241, 196, 15);
inline const QColor kCaution(243, 156, 18);
inline const QColor kBad(231, 76, 60);
inline std::atomic<int> odom_type_right{-1};
inline std::atomic<int> speed_center{-1};
inline std::atomic<int> odom_type_width{0};
inline std::atomic<int> nav_status_width{0};
inline constexpr int kTopRowCenter = 60;
inline constexpr int kSpeedHalf = 100;
inline const QColor kPrimary(174, 174, 174);

inline QColor stateColor(const std::string & text)
{
  if (text.rfind("GNSS", 0) == 0 || text == "good") {
    return kGood;
  }
  if (text == "normal") {
    return kWarn;
  }
  if (text == "abnormal") {
    return kBad;
  }
  return kCaution;
}
}

#pragma once

#include <string>
#include <QColor>

namespace rsf_rviz_plugins
{
inline const QColor kGood(46, 204, 113);
inline const QColor kWarn(241, 196, 15);
inline const QColor kCaution(243, 156, 18);
inline const QColor kBad(231, 76, 60);

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

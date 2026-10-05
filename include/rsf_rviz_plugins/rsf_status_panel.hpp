#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <QColor>
#include <QLabel>
#include <QTimer>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

namespace rsf_rviz_plugins
{
class RsfStatusPanel : public rviz_common::Panel
{
public:
  explicit RsfStatusPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void refresh();
  struct Row
  {
    QLabel * value;
    std::string text;
    QColor color;
    std::chrono::steady_clock::time_point stamp;
  };

  void subscribe();
  void set(const std::string & key, const std::string & text, const QColor & color);

  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::map<std::string, std::string> topics_;
  std::mutex mutex_;
  std::map<std::string, Row> rows_;
  QTimer * timer_;
};
}

#pragma once

#include <string>
#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rsf_rviz_plugins
{
class WaypointNavigationPanel : public rviz_common::Panel
{
public:
  explicit WaypointNavigationPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  QPushButton * button(const QString & label, const std::string & service);

  rclcpp::Node::SharedPtr node_;
  QLabel * status_;
};
}

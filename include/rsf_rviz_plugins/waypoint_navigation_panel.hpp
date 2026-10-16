#pragma once

#include <map>
#include <memory>
#include <string>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
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
  void call(const QString & label, const std::string & service);
  void startFrom(int index);
  void onStatus(const std_msgs::msg::String::ConstSharedPtr & msg);
  void setStatus(const QString & text);

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QSpinBox * start_index_;
  QLabel * total_;
  QLabel * nav_status_;
  QLabel * status_;
};
}

#ifndef RSF_RVIZ_PLUGINS__WAYPOINT_NAVIGATION_PANEL_HPP_
#define RSF_RVIZ_PLUGINS__WAYPOINT_NAVIGATION_PANEL_HPP_

#include <memory>
#include <QLabel>
#include <QTimer>
#include <QWidget>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rsf_rviz_plugins
{
class WaypointNavigationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointNavigationPanel(QWidget * parent = nullptr);

private Q_SLOTS:
  void callStart();
  void spinRos();

private:
  using Trigger = std_srvs::srv::Trigger;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  QTimer * timer_;
  QLabel * status_;
};
}

#endif

#include "rsf_rviz_plugins/waypoint_navigation_panel.hpp"

#include <chrono>

#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace rsf_rviz_plugins
{
WaypointNavigationPanel::WaypointNavigationPanel(QWidget * parent)
: rviz_common::Panel(parent), timer_(new QTimer(this)), status_(new QLabel("Ready", this))
{
  node_ = std::make_shared<rclcpp::Node>("waypoint_navigation_panel");
  start_client_ = node_->create_client<Trigger>("/waypoint_navigator/start");
  executor_.add_node(node_);

  auto * layout = new QVBoxLayout;
  auto * start_button = new QPushButton("Start Waypoint Navigation", this);
  layout->addWidget(start_button);
  layout->addWidget(status_);
  setLayout(layout);

  connect(start_button, &QPushButton::clicked, this, &WaypointNavigationPanel::callStart);
  connect(timer_, &QTimer::timeout, this, &WaypointNavigationPanel::spinRos);
  timer_->start(20);
}

void WaypointNavigationPanel::callStart()
{
  if (!start_client_->service_is_ready()) {
    status_->setText("Start failed: service unavailable");
    return;
  }
  status_->setText("Starting waypoint navigation...");
  auto request = std::make_shared<Trigger::Request>();
  start_client_->async_send_request(request,
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      QString result = QString::fromStdString(response->message);
      if (result.isEmpty()) {
        result = response->success ? "Started" : "Failed";
      }
      QMetaObject::invokeMethod(this, [this, result, success = response->success]() {
        status_->setText(success ? result : "Start failed: " + result);
      }, Qt::QueuedConnection);
    });
}

void WaypointNavigationPanel::spinRos()
{
  executor_.spin_some(std::chrono::milliseconds(0));
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::WaypointNavigationPanel, rviz_common::Panel)

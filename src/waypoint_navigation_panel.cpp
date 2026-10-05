#include "rsf_rviz_plugins/waypoint_navigation_panel.hpp"

#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rsf_rviz_plugins
{
WaypointNavigationPanel::WaypointNavigationPanel(QWidget * parent)
: rviz_common::Panel(parent), status_(new QLabel("Ready", this))
{
}

void WaypointNavigationPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  auto * layout = new QVBoxLayout;
  auto * step = new QHBoxLayout;
  layout->addWidget(button("Start Waypoint Navigation", "start"));
  layout->addWidget(button("Save Waypoint", "save_waypoint"));
  step->addWidget(button("Prev Waypoint", "prev_waypoint"));
  step->addWidget(button("Next Waypoint", "next_waypoint"));
  layout->addLayout(step);
  layout->addWidget(status_);
  setLayout(layout);
}

QPushButton * WaypointNavigationPanel::button(const QString & label, const std::string & service)
{
  using Trigger = std_srvs::srv::Trigger;
  auto client = node_->create_client<Trigger>("/waypoint_navigator/" + service);
  auto * button = new QPushButton(label, this);
  connect(button, &QPushButton::clicked, this, [this, client, label]() {
    if (!client->service_is_ready()) {
      status_->setText(label + " failed: service unavailable");
      return;
    }
    status_->setText(label + " requested...");
    client->async_send_request(std::make_shared<Trigger::Request>(),
      [this, label](rclcpp::Client<Trigger>::SharedFuture future) {
        const auto response = future.get();
        const auto message = QString::fromStdString(response->message);
        const auto text = response->success ?
          (message.isEmpty() ? label + " succeeded" : message) :
          label + " failed" + (message.isEmpty() ? "" : ": " + message);
        QMetaObject::invokeMethod(this, [this, text]() {status_->setText(text);}, Qt::QueuedConnection);
      });
  });
  return button;
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::WaypointNavigationPanel, rviz_common::Panel)

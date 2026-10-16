#include "rsf_rviz_plugins/waypoint_navigation_panel.hpp"

#include <algorithm>
#include <sstream>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rsf_rviz_plugins
{
namespace
{
const std::string kNavigator = "/waypoint_navigator";
const std::vector<std::string> kServices = {"start", "save_waypoint", "prev_waypoint", "next_waypoint"};
}

WaypointNavigationPanel::WaypointNavigationPanel(QWidget * parent)
: rviz_common::Panel(parent), start_index_(new QSpinBox(this)), total_(new QLabel("/ -", this)),
  nav_status_(new QLabel("No Data", this)), status_(new QLabel("Ready", this))
{
}

void WaypointNavigationPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  for (const auto & service : kServices) {
    clients_[service] = node_->create_client<std_srvs::srv::Trigger>(kNavigator + "/" + service);
  }
  parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, kNavigator);
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kNavigator + "/status", rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {onStatus(msg);});

  auto * layout = new QVBoxLayout;
  auto * from = new QHBoxLayout;
  from->addWidget(new QLabel("Start from waypoint", this));
  from->addWidget(start_index_);
  from->addWidget(total_);
  from->addStretch();
  layout->addLayout(from);
  auto * start = new QPushButton("Start Waypoint Navigation", this);
  connect(start, &QPushButton::clicked, this, [this]() {startFrom(start_index_->value());});
  layout->addWidget(start);
  layout->addWidget(button("Save Waypoint", "save_waypoint"));
  auto * step = new QHBoxLayout;
  step->addWidget(button("Prev Waypoint", "prev_waypoint"));
  step->addWidget(button("Next Waypoint", "next_waypoint"));
  layout->addLayout(step);
  layout->addWidget(nav_status_);
  layout->addWidget(status_);
  setLayout(layout);
}

QPushButton * WaypointNavigationPanel::button(const QString & label, const std::string & service)
{
  auto * button = new QPushButton(label, this);
  connect(button, &QPushButton::clicked, this, [this, label, service]() {call(label, service);});
  return button;
}

void WaypointNavigationPanel::call(const QString & label, const std::string & service)
{
  using Trigger = std_srvs::srv::Trigger;
  auto client = clients_.at(service);
  if (!client->service_is_ready()) {
    setStatus(label + " failed: service unavailable");
    return;
  }
  setStatus(label + " requested...");
  client->async_send_request(std::make_shared<Trigger::Request>(),
    [this, label](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      const auto message = QString::fromStdString(response->message);
      setStatus(response->success ?
        (message.isEmpty() ? label + " succeeded" : message) :
        label + " failed" + (message.isEmpty() ? "" : ": " + message));
    });
}

void WaypointNavigationPanel::startFrom(int index)
{
  const auto label = QString("Start from waypoint %1").arg(index);
  if (!parameters_->service_is_ready()) {
    setStatus(label + " failed: waypoint_navigator unavailable");
    return;
  }
  setStatus(label + " requested...");
  parameters_->set_parameters(
    {rclcpp::Parameter("start_index", index)},
    [this, label](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      if (results.empty() || !results.front().successful) {
        setStatus(label + " failed: could not set start_index");
        return;
      }
      call(label, "start");
    });
}

void WaypointNavigationPanel::onStatus(const std_msgs::msg::String::ConstSharedPtr & msg)
{
  std::istringstream in(msg->data);
  std::string state;
  int index = 0;
  int total = 0;
  if (!(in >> state >> index >> total)) {
    return;
  }
  const auto text = QString("%1  %2 / %3").arg(QString::fromStdString(state)).arg(index).arg(total);
  const int next = state == "finished" ? 0 : index;
  QMetaObject::invokeMethod(this, [this, text, next, total]() {
      nav_status_->setText(text);
      total_->setText(QString("/ %1").arg(total));
      start_index_->setRange(0, std::max(0, total - 1));
      if (!start_index_->hasFocus()) {
        start_index_->setValue(next);
      }
    }, Qt::QueuedConnection);
}

void WaypointNavigationPanel::setStatus(const QString & text)
{
  QMetaObject::invokeMethod(this, [this, text]() {status_->setText(text);}, Qt::QueuedConnection);
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::WaypointNavigationPanel, rviz_common::Panel)

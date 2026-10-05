#include "rsf_rviz_plugins/rsf_status_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <QGridLayout>
#include <QString>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include "rsf_rviz_plugins/state_color.hpp"

namespace rsf_rviz_plugins
{
namespace
{
constexpr auto kTimeout = std::chrono::seconds(2);

const QColor kStale(127, 140, 141);

const std::vector<std::string> kStringRows = {"Odom Type", "Odom State", "Fix Type", "Fix State"};
const std::vector<std::string> kRows = {
  "Odom Type", "Odom State", "Fix Type", "Fix State", "GNSS Quality", "Satellites", "HDOP",
  "Position", "Device"};

const std::vector<std::pair<std::string, std::string>> kTopics = {
  {"Odom Type Topic", "/rsf/rsf_odom_type"},
  {"Odom State Topic", "/rsf/rsf_odom_state"},
  {"Fix Type Topic", "/rsf/rsf_fix_type"},
  {"Fix State Topic", "/rsf/rsf_fix_state"},
  {"GPGGA Topic", "/rsf/gpgga"},
  {"NavSatFix Topic", "/rsf/nav_sat_fix"},
  {"Diagnostics Topic", "/rsf/diagnostics"},
};

std::pair<std::string, QColor> gpsQuality(uint32_t qual)
{
  switch (qual) {
    case 0: return {"0: No Fix", kBad};
    case 1: return {"1: GPS", kCaution};
    case 2: return {"2: DGPS", kCaution};
    case 4: return {"4: RTK Fix", kGood};
    case 5: return {"5: RTK Float", kWarn};
    case 6: return {"6: Dead Reckoning", kBad};
    default: return {std::to_string(qual) + ": Other", kCaution};
  }
}

std::string format(const char * fmt, double a, double b = 0.0, double c = 0.0)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c);
  return buf;
}
}

RsfStatusPanel::RsfStatusPanel(QWidget * parent)
: rviz_common::Panel(parent), timer_(new QTimer(this))
{
  for (const auto & [key, topic] : kTopics) {
    topics_[key] = topic;
  }

  auto * layout = new QGridLayout;
  layout->setColumnStretch(1, 1);
  int r = 0;
  for (const auto & label : kRows) {
    auto * name = new QLabel(QString::fromStdString(label), this);
    auto * value = new QLabel(this);
    value->setAlignment(Qt::AlignCenter);
    value->setMargin(3);
    QFont font = value->font();
    font.setBold(true);
    value->setFont(font);
    layout->addWidget(name, r, 0);
    layout->addWidget(value, r, 1);
    rows_[label] = Row{value, "", kStale, {}};
    ++r;
  }
  layout->setRowStretch(r, 1);
  setLayout(layout);

  connect(timer_, &QTimer::timeout, this, &RsfStatusPanel::refresh);
  refresh();
}

void RsfStatusPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  subscribe();
  timer_->start(200);
}

void RsfStatusPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  for (auto & [key, topic] : topics_) {
    QString value;
    if (config.mapGetString(QString::fromStdString(key), &value) && !value.isEmpty()) {
      topic = value.toStdString();
    }
  }
  if (node_) {
    subscribe();
  }
}

void RsfStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  for (const auto & [key, topic] : topics_) {
    config.mapSetValue(QString::fromStdString(key), QString::fromStdString(topic));
  }
}

void RsfStatusPanel::set(const std::string & key, const std::string & text, const QColor & color)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & row = rows_.at(key);
  row.text = text;
  row.color = color;
  row.stamp = std::chrono::steady_clock::now();
}

void RsfStatusPanel::subscribe()
{
  using std_msgs::msg::String;
  subs_.clear();
  const auto qos = rclcpp::SensorDataQoS();

  for (const auto & label : kStringRows) {
    subs_.push_back(node_->create_subscription<String>(
      topics_.at(label + " Topic"), qos,
      [this, label](String::ConstSharedPtr msg) {
        set(label, msg->data, stateColor(msg->data));
      }));
  }

  subs_.push_back(node_->create_subscription<nmea_msgs::msg::Gpgga>(
    topics_.at("GPGGA Topic"), qos,
    [this](nmea_msgs::msg::Gpgga::ConstSharedPtr msg) {
      const auto [text, color] = gpsQuality(msg->gps_qual);
      set("GNSS Quality", text, color);
      set("Satellites", std::to_string(msg->num_sats),
        msg->num_sats >= 10 ? kGood : msg->num_sats >= 5 ? kWarn : kBad);
      set("HDOP", format("%.2f", msg->hdop),
        msg->hdop <= 1.0 ? kGood : msg->hdop <= 2.0 ? kWarn : kBad);
    }));

  subs_.push_back(node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    topics_.at("NavSatFix Topic"), qos,
    [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
      if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        set("Position", "No Fix", kBad);
        return;
      }
      const double sigma = std::sqrt(
        std::max(msg->position_covariance[0], msg->position_covariance[4]));
      set("Position", format("%.7f, %.7f (%.2f m)", msg->latitude, msg->longitude, sigma),
        sigma <= 0.1 ? kGood : sigma <= 1.0 ? kWarn : kCaution);
    }));

  subs_.push_back(node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    topics_.at("Diagnostics Topic"), qos,
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      for (const auto & status : msg->status) {
        std::string text = status.message;
        for (const auto & kv : status.values) {
          if (kv.key == "device_temperature") {
            text += " / " + kv.value + " C";
          } else if (kv.key == "cpu_usage") {
            text += " / CPU " + kv.value + "%";
          }
        }
        using diagnostic_msgs::msg::DiagnosticStatus;
        set("Device", text,
          status.level == DiagnosticStatus::OK ? kGood :
          status.level == DiagnosticStatus::WARN ? kWarn : kBad);
      }
    }));
}

void RsfStatusPanel::refresh()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  for (auto & [key, row] : rows_) {
    const bool stale = row.text.empty() || now - row.stamp > kTimeout;
    const QColor color = stale ? kStale : row.color;
    row.value->setText(stale ? "No Data" : QString::fromStdString(row.text));
    row.value->setStyleSheet(
      QString("QLabel { background-color: %1; color: %2; border-radius: 4px; }")
      .arg(color.name(), qGray(color.rgb()) > 160 ? "black" : "white"));
  }
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::RsfStatusPanel, rviz_common::Panel)

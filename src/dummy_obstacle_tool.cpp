#include "rsf_rviz_plugins/dummy_obstacle_tool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace rsf_rviz_plugins
{
namespace
{
constexpr int kRays = 7200;

double hit(bool box, double half_length, double half_width, double x, double y, double yaw, double angle)
{
  const double inf = std::numeric_limits<double>::infinity();
  if (!box) {
    const double along = x * std::cos(angle) + y * std::sin(angle);
    const double across = x * std::sin(angle) - y * std::cos(angle);
    const double discriminant = half_width * half_width - across * across;
    if (discriminant < 0.0 || along + std::sqrt(discriminant) < 0.0) {
      return inf;
    }
    return std::max(0.0, along - std::sqrt(discriminant));
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double o[2] = {-x * c - y * s, x * s - y * c};
  const double d[2] = {std::cos(angle - yaw), std::sin(angle - yaw)};
  const double h[2] = {half_length, half_width};
  double near = -inf;
  double far = inf;
  for (int k = 0; k < 2; ++k) {
    if (std::abs(d[k]) < 1e-9) {
      if (std::abs(o[k]) > h[k]) {
        return inf;
      }
      continue;
    }
    const double t1 = (-h[k] - o[k]) / d[k];
    const double t2 = (h[k] - o[k]) / d[k];
    near = std::max(near, std::min(t1, t2));
    far = std::min(far, std::max(t1, t2));
  }
  return far < 0.0 || far < near ? inf : std::max(0.0, near);
}
}

DummyObstacleTool::DummyObstacleTool(const QString & name, const std::string & ns, char shortcut)
: name_(name), ns_(ns)
{
  shortcut_key_ = shortcut;
}

void DummyObstacleTool::onInitialize()
{
  PoseTool::onInitialize();
  setName(name_);
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/" + ns_ + "/scan", rclcpp::SensorDataQoS());
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/" + ns_ + "/markers", 1);
  QObject::connect(&timer_, &QTimer::timeout, [this]() {publish();});
  timer_.start(50);
}

int DummyObstacleTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.rightDown()) {
    clear();
    publish();
    return Render | Finished;
  }
  return PoseTool::processMouseEvent(event);
}

bool DummyObstacleTool::toMap(double & x, double & y, double & yaw)
{
  Ogre::Vector3 origin;
  Ogre::Quaternion rotation;
  if (!context_->getFrameManager()->getTransform(std::string("map"), origin, rotation)) {
    return false;
  }
  const auto inverse = rotation.Inverse();
  const auto point = inverse * (Ogre::Vector3(x, y, 0.0) - origin);
  const auto heading = inverse * Ogre::Vector3(std::cos(yaw), std::sin(yaw), 0.0);
  x = point.x;
  y = point.y;
  yaw = std::atan2(heading.y, heading.x);
  return true;
}

rclcpp::Time DummyObstacleTool::now()
{
  const auto now = node_->now();
  if (now < last_time_) {
    clear();
  }
  last_time_ = now;
  return now;
}

void DummyObstacleTool::publish()
{
  const auto stamp = now();
  const auto list = bodies(stamp);

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.emplace_back().action = visualization_msgs::msg::Marker::DELETEALL;
  for (const auto & body : list) {
    auto & marker = markers.markers.emplace_back();
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = ns_;
    marker.id = static_cast<int>(markers.markers.size());
    marker.type = body.box ? visualization_msgs::msg::Marker::CUBE :
      visualization_msgs::msg::Marker::CYLINDER;
    marker.pose.position.x = body.x;
    marker.pose.position.y = body.y;
    marker.pose.position.z = body.box ? 0.4 : 0.85;
    marker.pose.orientation.z = std::sin(body.yaw / 2.0);
    marker.pose.orientation.w = std::cos(body.yaw / 2.0);
    marker.scale.x = 2.0 * body.half_length;
    marker.scale.y = 2.0 * body.half_width;
    marker.scale.z = body.box ? 0.8 : 1.7;
    marker.color.r = body.r;
    marker.color.g = body.g;
    marker.color.b = body.b;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  }
  marker_pub_->publish(markers);

  Ogre::Vector3 map_origin, sensor_origin;
  Ogre::Quaternion map_rotation, sensor_rotation;
  if (!context_->getFrameManager()->getTransform(std::string("map"), map_origin, map_rotation) ||
    !context_->getFrameManager()->getTransform(
      std::string("rsf_hokuyo3d"), sensor_origin, sensor_rotation))
  {
    return;
  }
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "rsf_hokuyo3d";
  scan.header.stamp = stamp;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / kRays;
  scan.angle_max = scan.angle_min + (kRays - 1) * scan.angle_increment;
  scan.range_min = 0.0;
  scan.range_max = 30.0;
  scan.scan_time = 0.05;
  scan.ranges.assign(kRays, std::numeric_limits<float>::infinity());

  const auto to_sensor = sensor_rotation.Inverse() * map_rotation;
  const auto offset = sensor_rotation.Inverse() * (map_origin - sensor_origin);
  for (const auto & body : list) {
    const auto center = to_sensor * Ogre::Vector3(body.x, body.y, 0.0) + offset;
    const auto heading = to_sensor * Ogre::Vector3(std::cos(body.yaw), std::sin(body.yaw), 0.0);
    const double yaw = std::atan2(heading.y, heading.x);
    for (int i = 0; i < kRays; ++i) {
      const double range = hit(
        body.box, body.half_length, body.half_width, center.x, center.y, yaw,
        scan.angle_min + i * scan.angle_increment);
      if (range < scan.range_max && range < scan.ranges[i]) {
        scan.ranges[i] = range;
      }
    }
  }
  scan_pub_->publish(scan);
}
}

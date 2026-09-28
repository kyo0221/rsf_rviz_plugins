#include "rsf_rviz_plugins/dummy_pedestrian_tool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace rsf_rviz_plugins
{
DummyPedestrianTool::DummyPedestrianTool()
{
  shortcut_key_ = 'p';
}

void DummyPedestrianTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Dummy Pedestrian");
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/dummy_pedestrians/scan", rclcpp::SensorDataQoS());
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/dummy_pedestrians/markers", 1);
  QObject::connect(&timer_, &QTimer::timeout, [this]() {publish();});
  timer_.start(50);
}

int DummyPedestrianTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.rightDown()) {
    pedestrians_.clear();
    publish();
    return Render | Finished;
  }
  return PoseTool::processMouseEvent(event);
}

void DummyPedestrianTool::onPoseSet(double x, double y, double theta)
{
  publish();
  pedestrians_.push_back({x, y, theta, node_->now()});
  publish();
}

void DummyPedestrianTool::publish()
{
  const auto now = node_->now();
  const auto frame = context_->getFixedFrame().toStdString();
  if (frame != frame_ || now < last_time_) {
    pedestrians_.clear();
  }
  frame_ = frame;
  last_time_ = now;

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  Ogre::Vector3 origin;
  Ogre::Quaternion rotation;
  const bool have_tf = context_->getFrameManager()->getTransform(
    std::string("rsf_hokuyo3d"), origin, rotation);
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "rsf_hokuyo3d";
  scan.header.stamp = now;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 1440;
  scan.angle_max = scan.angle_min + 1439 * scan.angle_increment;
  scan.range_min = 0.0;
  scan.range_max = 30.0;
  scan.scan_time = 0.05;
  scan.ranges.assign(1440, std::numeric_limits<float>::infinity());

  for (const auto & pedestrian : pedestrians_) {
    const double distance = 0.8 * (now - pedestrian.start).seconds();
    const double x = pedestrian.x + distance * std::cos(pedestrian.theta);
    const double y = pedestrian.y + distance * std::sin(pedestrian.theta);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = now;
    marker.ns = "pedestrians";
    marker.id = static_cast<int>(markers.markers.size());
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.85;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = 0.6;
    marker.scale.z = 1.7;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(marker);

    if (!have_tf) {
      continue;
    }
    const auto local = rotation.Inverse() * (Ogre::Vector3(x, y, 0.0) - origin);
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + i * scan.angle_increment;
      const double along = local.x * std::cos(angle) + local.y * std::sin(angle);
      const double across = local.x * std::sin(angle) - local.y * std::cos(angle);
      const double discriminant = 0.09 - across * across;
      if (discriminant < 0.0 || along + std::sqrt(discriminant) < 0.0) {
        continue;
      }
      const double range = std::max(0.0, along - std::sqrt(discriminant));
      if (range < scan.range_max && range < scan.ranges[i]) {
        scan.ranges[i] = range;
      }
    }
  }
  marker_pub_->publish(markers);
  if (have_tf) {
    scan_pub_->publish(scan);
  }
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::DummyPedestrianTool, rviz_common::Tool)

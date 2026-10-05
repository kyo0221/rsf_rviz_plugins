#include "rsf_rviz_plugins/dummy_pedestrian_tool.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace rsf_rviz_plugins
{
DummyPedestrianTool::DummyPedestrianTool()
: DummyObstacleTool("2D Dummy Pedestrian", "dummy_pedestrians", 'p')
{
}

void DummyPedestrianTool::onPoseSet(double x, double y, double theta)
{
  if (toMap(x, y, theta)) {
    pedestrians_.push_back({x, y, theta, now()});
    publish();
  }
}

std::vector<DummyObstacleTool::Body> DummyPedestrianTool::bodies(const rclcpp::Time & now)
{
  std::vector<Body> bodies;
  for (const auto & pedestrian : pedestrians_) {
    const double distance = 0.8 * (now - pedestrian.start).seconds();
    bodies.push_back({
      pedestrian.x + distance * std::cos(pedestrian.yaw),
      pedestrian.y + distance * std::sin(pedestrian.yaw),
      pedestrian.yaw, 0.3, 0.3, false, 1.0f, 0.5f, 0.0f});
  }
  return bodies;
}

void DummyPedestrianTool::clear()
{
  pedestrians_.clear();
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::DummyPedestrianTool, rviz_common::Tool)

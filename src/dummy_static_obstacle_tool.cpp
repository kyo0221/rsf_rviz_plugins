#include "rsf_rviz_plugins/dummy_static_obstacle_tool.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace rsf_rviz_plugins
{
DummyStaticObstacleTool::DummyStaticObstacleTool()
: DummyObstacleTool("2D Static Obstacle", "dummy_static_obstacles", 'o')
{
}

void DummyStaticObstacleTool::onPoseSet(double x, double y, double theta)
{
  if (toMap(x, y, theta)) {
    obstacles_.push_back({x, y, theta, 0.4, 0.3, true, 0.5f, 0.5f, 0.5f});
    publish();
  }
}

std::vector<DummyObstacleTool::Body> DummyStaticObstacleTool::bodies(const rclcpp::Time &)
{
  return obstacles_;
}

void DummyStaticObstacleTool::clear()
{
  obstacles_.clear();
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::DummyStaticObstacleTool, rviz_common::Tool)

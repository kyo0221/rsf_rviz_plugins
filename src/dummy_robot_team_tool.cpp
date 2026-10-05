#include "rsf_rviz_plugins/dummy_robot_team_tool.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rsf_rviz_plugins
{
DummyRobotTeamTool::DummyRobotTeamTool()
: DummyObstacleTool("2D Dummy Robot Team", "dummy_robot_team", 't')
{
}

void DummyRobotTeamTool::onInitialize()
{
  DummyObstacleTool::onInitialize();
  std::string route;
  try {
    route = ament_index_cpp::get_package_share_directory("rsf_waypoint_manager") +
      "/waypoints/tsudanuma2-3_wp.yaml";
  } catch (const std::exception &) {
  }
  route_property_ = new rviz_common::properties::StringProperty(
    "Waypoints File", QString::fromStdString(route), "Waypoint yaml the team follows.",
    getPropertyContainer());
}

bool DummyRobotTeamTool::loadRoute()
{
  const auto file = route_property_->getStdString();
  if (file == route_file_ && lengths_.size() > 1) {
    return true;
  }
  teams_.clear();
  xs_.clear();
  ys_.clear();
  lengths_.clear();
  route_file_ = file;
  try {
    for (const auto & waypoint : YAML::LoadFile(file)["waypoints"]) {
      xs_.push_back(waypoint["x"].as<double>());
      ys_.push_back(waypoint["y"].as<double>());
      lengths_.push_back(
        lengths_.empty() ? 0.0 :
        lengths_.back() + std::hypot(xs_.back() - xs_[xs_.size() - 2], ys_.back() - ys_[ys_.size() - 2]));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load %s: %s", file.c_str(), e.what());
  }
  return lengths_.size() > 1;
}

DummyObstacleTool::Body DummyRobotTeamTool::bodyAt(double s, double offset, bool leader) const
{
  const size_t i = std::min<size_t>(
    std::upper_bound(lengths_.begin(), lengths_.end(), s) - lengths_.begin(), lengths_.size() - 1);
  const size_t j = std::max<size_t>(i, 1);
  const double yaw = std::atan2(ys_[j] - ys_[j - 1], xs_[j] - xs_[j - 1]);
  const double t = s - lengths_[j - 1];
  const double x = xs_[j - 1] + t * std::cos(yaw) - offset * std::sin(yaw);
  const double y = ys_[j - 1] + t * std::sin(yaw) + offset * std::cos(yaw);
  if (leader) {
    return {x, y, yaw, 0.4, 0.3, true, 0.2f, 0.4f, 1.0f};
  }
  return {x, y, yaw, 0.3, 0.3, false, 0.3f, 0.9f, 0.3f};
}

void DummyRobotTeamTool::onPoseSet(double x, double y, double theta)
{
  if (!loadRoute() || !toMap(x, y, theta)) {
    return;
  }
  double best = std::numeric_limits<double>::infinity();
  double start = 0.0;
  for (size_t i = 1; i < lengths_.size(); ++i) {
    const double dx = xs_[i] - xs_[i - 1];
    const double dy = ys_[i] - ys_[i - 1];
    const double length = lengths_[i] - lengths_[i - 1];
    const double t = length > 0.0 ? std::clamp(
      ((x - xs_[i - 1]) * dx + (y - ys_[i - 1]) * dy) / (length * length), 0.0, 1.0) :
      0.0;
    const double distance = std::hypot(xs_[i - 1] + t * dx - x, ys_[i - 1] + t * dy - y);
    if (distance < best) {
      best = distance;
      start = lengths_[i - 1] + t * length;
    }
  }
  std::uniform_real_distribution<double> speed(0.4, 1.0);
  std::uniform_real_distribution<double> offset(-0.4, 0.4);
  Team team{start, speed(rng_), {}, now()};
  for (int i = std::uniform_int_distribution<int>(1, 2)(rng_); i > 0; --i) {
    team.offsets.push_back(offset(rng_));
  }
  RCLCPP_INFO(node_->get_logger(), "dummy robot team: s=%.1f speed=%.2f followers=%zu",
    start, team.speed, team.offsets.size());
  teams_.push_back(team);
  publish();
}

std::vector<DummyObstacleTool::Body> DummyRobotTeamTool::bodies(const rclcpp::Time & now)
{
  std::vector<Body> bodies;
  for (const auto & team : teams_) {
    const double s = std::min(team.start + team.speed * (now - team.stamp).seconds(), lengths_.back());
    bodies.push_back(bodyAt(s, 0.0, true));
    for (size_t i = 0; i < team.offsets.size(); ++i) {
      bodies.push_back(bodyAt(s - 1.2 - 0.9 * i, team.offsets[i], false));
    }
  }
  return bodies;
}

void DummyRobotTeamTool::clear()
{
  teams_.clear();
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::DummyRobotTeamTool, rviz_common::Tool)

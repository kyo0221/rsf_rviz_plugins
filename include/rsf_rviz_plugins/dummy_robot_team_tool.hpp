#pragma once

#include <random>
#include <string>
#include <vector>
#include <rviz_common/properties/string_property.hpp>
#include "rsf_rviz_plugins/dummy_obstacle_tool.hpp"

namespace rsf_rviz_plugins
{
class DummyRobotTeamTool : public DummyObstacleTool
{
public:
  DummyRobotTeamTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;
  std::vector<Body> bodies(const rclcpp::Time & now) override;
  void clear() override;

private:
  struct Team
  {
    double start, speed;
    std::vector<double> offsets;
    rclcpp::Time stamp;
  };

  bool loadRoute();
  Body bodyAt(double s, double offset, bool leader) const;

  rviz_common::properties::StringProperty * route_property_;
  std::string route_file_;
  std::vector<double> xs_, ys_, lengths_;
  std::vector<Team> teams_;
  std::mt19937 rng_{std::random_device{}()};
};
}

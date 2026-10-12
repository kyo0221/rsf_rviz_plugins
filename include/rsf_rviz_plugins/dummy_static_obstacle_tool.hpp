#pragma once

#include <vector>
#include "rsf_rviz_plugins/dummy_obstacle_tool.hpp"

namespace rsf_rviz_plugins
{
class DummyStaticObstacleTool : public DummyObstacleTool
{
public:
  DummyStaticObstacleTool();

protected:
  void onPoseSet(double x, double y, double theta) override;
  std::vector<Body> bodies(const rclcpp::Time & now) override;
  void clear() override;

private:
  std::vector<Body> obstacles_;
};
}

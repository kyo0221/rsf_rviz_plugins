#pragma once

#include <vector>
#include "rsf_rviz_plugins/dummy_obstacle_tool.hpp"

namespace rsf_rviz_plugins
{
class DummyPedestrianTool : public DummyObstacleTool
{
public:
  DummyPedestrianTool();

protected:
  void onPoseSet(double x, double y, double theta) override;
  std::vector<Body> bodies(const rclcpp::Time & now) override;
  void clear() override;

private:
  struct Pedestrian
  {
    double x, y, yaw;
    rclcpp::Time start;
  };

  std::vector<Pedestrian> pedestrians_;
};
}

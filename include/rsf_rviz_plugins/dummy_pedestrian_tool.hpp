#pragma once

#include <string>
#include <vector>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rsf_rviz_plugins
{
class DummyPedestrianTool : public rviz_default_plugins::tools::PoseTool
{
public:
  DummyPedestrianTool();
  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  struct Pedestrian
  {
    double x, y, theta;
    rclcpp::Time start;
  };

  void publish();
  QTimer timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::vector<Pedestrian> pedestrians_;
  std::string frame_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};
}

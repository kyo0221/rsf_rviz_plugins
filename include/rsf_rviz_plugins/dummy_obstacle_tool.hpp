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
class DummyObstacleTool : public rviz_default_plugins::tools::PoseTool
{
public:
  DummyObstacleTool(const QString & name, const std::string & ns, char shortcut);
  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  struct Body
  {
    double x, y, yaw, half_length, half_width;
    bool box;
    float r, g, b;
  };

  virtual std::vector<Body> bodies(const rclcpp::Time & now) = 0;
  virtual void clear() = 0;
  bool toMap(double & x, double & y, double & yaw);
  rclcpp::Time now();
  void publish();

  rclcpp::Node::SharedPtr node_;

private:
  QString name_;
  std::string ns_;
  QTimer timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};
}

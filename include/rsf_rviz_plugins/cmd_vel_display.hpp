#pragma once

#include <chrono>
#include <mutex>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <QString>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

namespace rsf_rviz_plugins
{
class CmdVelDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  CmdVelDisplay();
  ~CmdVelDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updateTopic();

private:
  void draw(const QString & value);

  rclcpp::Node::SharedPtr node_;
  rviz_common::properties::RosTopicProperty * topic_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::mutex mutex_;
  bool received_ = false;
  double speed_ = 0.0;
  std::chrono::steady_clock::time_point stamp_;
  QString shown_;
  int drawn_width_ = 0;
  Ogre::Overlay * overlay_ = nullptr;
  Ogre::PanelOverlayElement * panel_ = nullptr;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};
}

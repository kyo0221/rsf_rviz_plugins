#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <std_msgs/msg/string.hpp>

namespace rsf_rviz_plugins
{
class OdomTypeDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  OdomTypeDisplay();
  ~OdomTypeDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updateTopics();

private:
  struct Field
  {
    rviz_common::properties::RosTopicProperty * topic;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    std::string text;
    std::chrono::steady_clock::time_point stamp;
  };

  void subscribe(Field & field);
  void draw(const std::string & type, const std::string & state);

  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  Field type_;
  Field state_;
  std::string shown_;
  int drawn_width_ = 0;
  Ogre::Overlay * overlay_ = nullptr;
  Ogre::PanelOverlayElement * panel_ = nullptr;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};
}

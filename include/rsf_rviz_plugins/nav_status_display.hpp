#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <QColor>
#include <QString>
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <std_msgs/msg/string.hpp>

namespace rsf_rviz_plugins
{
class NavStatusDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  NavStatusDisplay();
  ~NavStatusDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updateTopics();

private:
  void onLog(const nav2_msgs::msg::BehaviorTreeLog & msg);
  bool compose(QString & text, QColor & color);
  void draw(const QString & text, const QColor & color);

  rclcpp::Node::SharedPtr node_;
  rviz_common::properties::RosTopicProperty * log_topic_;
  rviz_common::properties::RosTopicProperty * status_topic_;
  rviz_common::properties::RosTopicProperty * speed_topic_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_sub_;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> behavior_subs_;

  std::mutex mutex_;
  std::string status_;
  double speed_limit_ = 0.0;
  std::string reason_;
  int recovery_count_ = 0;
  std::string behavior_;
  std::string previous_;
  QString label_;
  std::chrono::steady_clock::time_point behavior_end_;

  QString shown_;
  int drawn_width_ = 0;
  Ogre::Overlay * overlay_ = nullptr;
  Ogre::PanelOverlayElement * panel_ = nullptr;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};
}

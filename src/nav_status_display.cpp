#include "rsf_rviz_plugins/nav_status_display.hpp"

#include <algorithm>
#include <sstream>
#include <QFont>
#include <QImage>
#include <QPainter>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlayManager.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/render_system.hpp>
#include "rsf_rviz_plugins/state_color.hpp"

namespace rsf_rviz_plugins
{
namespace
{
constexpr int kWidth = 720;
constexpr int kHeight = 44;
constexpr auto kRecoveryHold = std::chrono::milliseconds(1500);
const QColor kSuccess("#8DF08B");
const QColor kWarning("#EEF08B");
const QColor kDanger("#F08B8B");
const QColor kInfo("#8BD0F0");
const QColor kNeutral("#303538");
const QColor kDarkText("#0F1417");
const QColor kLightText("#DFE3E7");

QString behaviorLabel(const std::string & name, const std::string & previous)
{
  if (name == "backup") {
    return "Retreat 1/3: back up";
  }
  if (name == "spin") {
    return previous == "backup" ? "Retreat 2/3: turn around" : "Spin";
  }
  if (name == "drive_on_heading") {
    return "Retreat 3/3: drive forward";
  }
  return "Wait";
}
}

NavStatusDisplay::NavStatusDisplay()
{
  log_topic_ = new rviz_common::properties::RosTopicProperty(
    "Behavior Tree Log Topic", "/behavior_tree_log", "nav2_msgs/msg/BehaviorTreeLog", "",
    this, SLOT(updateTopics()));
  status_topic_ = new rviz_common::properties::RosTopicProperty(
    "Waypoint Status Topic", "/waypoint_navigator/status", "std_msgs/msg/String", "",
    this, SLOT(updateTopics()));
  speed_topic_ = new rviz_common::properties::RosTopicProperty(
    "Speed Limit Topic", "/speed_limit", "nav2_msgs/msg/SpeedLimit", "",
    this, SLOT(updateTopics()));
}

NavStatusDisplay::~NavStatusDisplay()
{
  if (overlay_) {
    auto & manager = Ogre::OverlayManager::getSingleton();
    overlay_->remove2D(panel_);
    manager.destroyOverlayElement(panel_);
    manager.destroy(overlay_);
    Ogre::MaterialManager::getSingleton().remove(material_);
    Ogre::TextureManager::getSingleton().remove(texture_);
  }
}

void NavStatusDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  log_topic_->initialize(context_->getRosNodeAbstraction());
  status_topic_->initialize(context_->getRosNodeAbstraction());
  speed_topic_->initialize(context_->getRosNodeAbstraction());
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  const std::string name = "rsf_nav_status_" + std::to_string(reinterpret_cast<uintptr_t>(this));
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
    kWidth, kHeight, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  material_ = Ogre::MaterialManager::getSingleton().create(
    name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto * pass = material_->getTechnique(0)->getPass(0);
  pass->createTextureUnitState(name);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthCheckEnabled(false);
  pass->setLightingEnabled(false);

  auto & manager = Ogre::OverlayManager::getSingleton();
  panel_ = static_cast<Ogre::PanelOverlayElement *>(manager.createOverlayElement("Panel", name));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setDimensions(kWidth, kHeight);
  panel_->setMaterialName(name);
  overlay_ = manager.create(name);
  overlay_->add2D(panel_);
  overlay_->setZOrder(500);
}

void NavStatusDisplay::onEnable()
{
  behavior_subs_.clear();
  log_sub_.reset();
  status_sub_.reset();
  speed_sub_.reset();
  if (!log_topic_->isEmpty()) {
    log_sub_ = node_->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      log_topic_->getTopicStd(), rclcpp::QoS(50),
      [this](nav2_msgs::msg::BehaviorTreeLog::ConstSharedPtr msg) {onLog(*msg);});
  }
  if (!status_topic_->isEmpty()) {
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      status_topic_->getTopicStd(), rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::istringstream next(msg->data), previous(status_);
        std::string next_state, previous_state;
        int next_index = -1, previous_index = -1;
        next >> next_state >> next_index;
        previous >> previous_state >> previous_index;
        if (next_index != previous_index) {
          recovery_count_ = 0;
          reason_.clear();
        }
        status_ = msg->data;
      });
  }
  if (!speed_topic_->isEmpty()) {
    speed_sub_ = node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_topic_->getTopicStd(), rclcpp::QoS(1),
      [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_limit_ = msg->percentage ? 0.0 : msg->speed_limit;
      });
  }
  for (const std::string name : {"backup", "spin", "drive_on_heading", "wait"}) {
    behavior_subs_.push_back(node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/" + name + "/_action/status", rclcpp::QoS(1).transient_local(),
      [this, name](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        const bool executing = std::any_of(
          msg->status_list.begin(), msg->status_list.end(), [](const auto & status) {
            return status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
          });
        std::lock_guard<std::mutex> lock(mutex_);
        if (executing && behavior_ != name) {
          label_ = behaviorLabel(name, previous_);
          behavior_ = name;
        } else if (!executing && behavior_ == name) {
          previous_ = name;
          behavior_.clear();
          behavior_end_ = std::chrono::steady_clock::now();
        }
      }));
  }
}

void NavStatusDisplay::onDisable()
{
  behavior_subs_.clear();
  log_sub_.reset();
  status_sub_.reset();
  speed_sub_.reset();
  overlay_->hide();
}

void NavStatusDisplay::updateTopics()
{
  if (isEnabled()) {
    onEnable();
  }
}

void NavStatusDisplay::onLog(const nav2_msgs::msg::BehaviorTreeLog & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & event : msg.event_log) {
    const auto & name = event.node_name;
    if (name == "RecoveryFallback" && event.previous_status == "IDLE" &&
      event.current_status == "RUNNING")
    {
      ++recovery_count_;
    } else if (event.current_status == "FAILURE" && name == "FollowPath") {
      reason_ = "stuck";
    } else if (event.current_status == "FAILURE" && name.rfind("ComputePath", 0) == 0) {
      reason_ = "no path";
    }
  }
}

bool NavStatusDisplay::compose(QString & text, QColor & color)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!behavior_.empty() || std::chrono::steady_clock::now() - behavior_end_ < kRecoveryHold) {
    text = label_ + QString(" · #%1").arg(recovery_count_);
    if (!reason_.empty()) {
      text += " " + QString::fromStdString(reason_);
    }
    color = kWarning;
    return true;
  }
  if (status_.empty()) {
    return false;
  }
  std::istringstream stream(status_);
  std::string state;
  int index = 0;
  int total = 0;
  stream >> state >> index >> total;
  const QString wp = QString("WP %1/%2").arg(index).arg(total - 1);
  if (state == "running") {
    text = "Running " + wp;
    if (speed_limit_ > 0.0) {
      text += QString(" · limit %1 m/s").arg(speed_limit_, 0, 'f', 1);
    }
    color = kSuccess;
  } else if (state == "stopped") {
    text = QString("Stopped WP %1 · waiting").arg(index);
    color = kInfo;
  } else if (state == "paused") {
    text = QString("Paused WP %1").arg(index);
    color = kNeutral;
  } else if (state == "failed") {
    text = QString("Failed WP %1").arg(index);
    color = kDanger;
  } else if (state == "finished") {
    text = "Finished";
    color = kSuccess;
  } else {
    text = "Idle";
    color = kNeutral;
  }
  return true;
}

void NavStatusDisplay::update(float, float)
{
  QString text;
  QColor color;
  if (!compose(text, color)) {
    overlay_->hide();
    return;
  }
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
    context_->getViewManager()->getRenderPanel()->getRenderWindow());
  const QString key = text + color.name();
  if (key != shown_) {
    shown_ = key;
    draw(text, color);
  }
  const int right = odom_type_right;
  panel_->setPosition(
    right >= 0 ? right + 8 : (static_cast<int>(viewport->getActualWidth()) - drawn_width_) / 2, 8);
  overlay_->show();
}

void NavStatusDisplay::draw(const QString & text, const QColor & color)
{
  QImage image(kWidth, kHeight, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  QFont font("Roboto");
  font.setPixelSize(18);
  font.setBold(true);
  painter.setFont(font);
  drawn_width_ = std::min(kWidth, painter.fontMetrics().horizontalAdvance(text) + 40);
  painter.setPen(Qt::NoPen);
  painter.setBrush(color);
  painter.drawRoundedRect(0, 0, drawn_width_, kHeight, kHeight / 2, kHeight / 2);
  painter.setPen(color == kNeutral ? kLightText : kDarkText);
  painter.drawText(0, 0, drawn_width_, kHeight, Qt::AlignCenter, text);
  texture_->getBuffer()->blitFromMemory(
    Ogre::PixelBox(kWidth, kHeight, 1, Ogre::PF_A8R8G8B8, image.bits()));
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::NavStatusDisplay, rviz_common::Display)

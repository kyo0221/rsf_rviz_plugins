#include "rsf_rviz_plugins/cmd_vel_display.hpp"

#include <cmath>
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

namespace rsf_rviz_plugins
{
namespace
{
constexpr int kSize = 140;
constexpr int kMargin = 16;
constexpr auto kTimeout = std::chrono::seconds(1);
}

CmdVelDisplay::CmdVelDisplay()
{
  topic_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "/cmd_vel", "geometry_msgs/msg/Twist", "", this, SLOT(updateTopic()));
}

CmdVelDisplay::~CmdVelDisplay()
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

void CmdVelDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  topic_->initialize(context_->getRosNodeAbstraction());
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  const std::string name = "rsf_cmd_vel_" + std::to_string(reinterpret_cast<uintptr_t>(this));
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
    kSize, kSize, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
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
  panel_->setDimensions(kSize, kSize);
  panel_->setMaterialName(name);
  overlay_ = manager.create(name);
  overlay_->add2D(panel_);
  overlay_->setZOrder(500);
}

void CmdVelDisplay::onEnable()
{
  sub_.reset();
  if (topic_->isEmpty()) {
    return;
  }
  sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    topic_->getTopicStd(), rclcpp::SystemDefaultsQoS(),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      received_ = true;
      speed_ = msg->linear.x;
      stamp_ = std::chrono::steady_clock::now();
    });
}

void CmdVelDisplay::onDisable()
{
  sub_.reset();
  overlay_->hide();
}

void CmdVelDisplay::updateTopic()
{
  if (isEnabled()) {
    onEnable();
  }
}

void CmdVelDisplay::update(float, float)
{
  double speed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) {
      overlay_->hide();
      return;
    }
    if (std::chrono::steady_clock::now() - stamp_ > kTimeout) {
      speed_ = 0.0;
    }
    speed = speed_;
  }
  const QString value = QString::number(std::abs(speed) < 0.005 ? 0.0 : speed, 'f', 2);
  if (value != shown_) {
    shown_ = value;
    draw(value);
  }
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
    context_->getViewManager()->getRenderPanel()->getRenderWindow());
  panel_->setPosition(kMargin, viewport->getActualHeight() - kSize - kMargin);
  overlay_->show();
}

void CmdVelDisplay::draw(const QString & value)
{
  QImage image(kSize, kSize, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(QColor(200, 200, 200, 230), 4));
  painter.setBrush(QColor(60, 60, 60, 200));
  painter.drawEllipse(4, 4, kSize - 8, kSize - 8);

  painter.setPen(Qt::white);
  QFont font = painter.font();
  font.setBold(true);
  font.setPixelSize(38);
  painter.setFont(font);
  painter.drawText(0, 30, kSize, 52, Qt::AlignCenter, value);
  font.setBold(false);
  font.setPixelSize(18);
  painter.setFont(font);
  painter.drawText(0, 82, kSize, 26, Qt::AlignCenter, "m/s");

  texture_->getBuffer()->blitFromMemory(
    Ogre::PixelBox(kSize, kSize, 1, Ogre::PF_A8R8G8B8, image.bits()));
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::CmdVelDisplay, rviz_common::Display)

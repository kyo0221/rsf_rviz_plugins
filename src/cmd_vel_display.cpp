#include "rsf_rviz_plugins/cmd_vel_display.hpp"

#include <cmath>
#include <QFont>
#include <QFontDatabase>
#include <QImage>
#include <QPainter>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlayManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include "rsf_rviz_plugins/state_color.hpp"
#include <rviz_rendering/render_system.hpp>

namespace rsf_rviz_plugins
{
namespace
{
constexpr int kWidth = 550;
constexpr int kHeight = 100;
constexpr int kTop = 10;
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
  const std::string fonts =
    ament_index_cpp::get_package_share_directory("rsf_rviz_plugins") + "/assets/font/Quicksand/";
  QFontDatabase::addApplicationFont(QString::fromStdString(fonts + "Quicksand-Regular.ttf"));
  QFontDatabase::addApplicationFont(QString::fromStdString(fonts + "Quicksand-Bold.ttf"));

  const std::string name = "rsf_cmd_vel_" + std::to_string(reinterpret_cast<uintptr_t>(this));
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
  top_overlay_bottom = 0;
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
      top_overlay_bottom = 0;
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
  panel_->setPosition((static_cast<int>(viewport->getActualWidth()) - kWidth) / 2, kTop);
  top_overlay_bottom = kTop + kHeight;
  overlay_->show();
}

void CmdVelDisplay::draw(const QString & value)
{
  QImage image(kWidth, kHeight, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  const QRectF background(0, 0, kWidth, kHeight);
  QColor fill(0, 0, 0);
  fill.setAlphaF(0.3);
  painter.setPen(Qt::NoPen);
  painter.setBrush(fill);
  painter.drawRoundedRect(background, background.height() / 2, background.height() / 2);

  const QColor color(174, 174, 174);
  painter.setFont(QFont("Quicksand", 40));
  const QRect value_rect = painter.fontMetrics().boundingRect(value);
  painter.setPen(color);
  painter.drawText(
    QPointF(
      background.center().x() - value_rect.width() / 2.0,
      background.center().y() + value_rect.bottom()),
    value);

  painter.setFont(QFont("Quicksand", 8, QFont::DemiBold));
  const QString unit = "m/s";
  const QRect unit_rect = painter.fontMetrics().boundingRect(unit);
  painter.drawText(
    QPointF(
      background.width() / 2.0 - unit_rect.width() / 2.0,
      background.height() / 2.0 + unit_rect.height() + 15.0),
    unit);

  texture_->getBuffer()->blitFromMemory(
    Ogre::PixelBox(kWidth, kHeight, 1, Ogre::PF_A8R8G8B8, image.bits()));
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::CmdVelDisplay, rviz_common::Display)

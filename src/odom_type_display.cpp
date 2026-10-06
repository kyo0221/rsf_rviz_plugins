#include "rsf_rviz_plugins/odom_type_display.hpp"

#include <vector>
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
constexpr int kWidth = 640;
constexpr int kHeight = 44;
constexpr auto kTimeout = std::chrono::seconds(1);
}

OdomTypeDisplay::OdomTypeDisplay()
{
  type_.topic = new rviz_common::properties::RosTopicProperty(
    "Type Topic", "/rsf/rsf_odom_type", "std_msgs/msg/String", "", this, SLOT(updateTopics()));
  state_.topic = new rviz_common::properties::RosTopicProperty(
    "State Topic", "/rsf/rsf_odom_state", "std_msgs/msg/String", "", this, SLOT(updateTopics()));
}

OdomTypeDisplay::~OdomTypeDisplay()
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

void OdomTypeDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  type_.topic->initialize(context_->getRosNodeAbstraction());
  state_.topic->initialize(context_->getRosNodeAbstraction());
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  const std::string name = "rsf_odom_type_" + std::to_string(reinterpret_cast<uintptr_t>(this));
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

void OdomTypeDisplay::onEnable()
{
  subscribe(type_);
  subscribe(state_);
}

void OdomTypeDisplay::onDisable()
{
  odom_type_right = -1;
  type_.sub.reset();
  state_.sub.reset();
  overlay_->hide();
}

void OdomTypeDisplay::updateTopics()
{
  if (isEnabled()) {
    onEnable();
  }
}

void OdomTypeDisplay::subscribe(Field & field)
{
  field.sub.reset();
  if (field.topic->isEmpty()) {
    return;
  }
  field.sub = node_->create_subscription<std_msgs::msg::String>(
    field.topic->getTopicStd(), rclcpp::SensorDataQoS(),
    [this, &field](std_msgs::msg::String::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      field.text = msg->data;
      field.stamp = std::chrono::steady_clock::now();
    });
}

void OdomTypeDisplay::update(float, float)
{
  std::string type;
  std::string state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (now - type_.stamp > kTimeout) {
      type_.text.clear();
    }
    if (now - state_.stamp > kTimeout) {
      state_.text.clear();
    }
    type = type_.text;
    state = state_.text;
  }
  if (type.empty() && state.empty()) {
    odom_type_right = -1;
    overlay_->hide();
    return;
  }
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
    context_->getViewManager()->getRenderPanel()->getRenderWindow());
  panel_->setPosition((viewport->getActualWidth() - kWidth) / 2, 8);
  if (type + "\n" + state != shown_) {
    shown_ = type + "\n" + state;
    draw(type, state);
  }
  odom_type_right = (viewport->getActualWidth() + drawn_width_) / 2;
  overlay_->show();
}

void OdomTypeDisplay::draw(const std::string & type, const std::string & state)
{
  QImage image(kWidth, kHeight, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  QFont font = painter.font();
  font.setPixelSize(18);
  font.setBold(true);
  painter.setFont(font);

  std::vector<QString> cells;
  for (const auto & text : {type, state}) {
    if (!text.empty()) {
      cells.push_back(QString::fromStdString(text));
    }
  }
  int width = 8 * (static_cast<int>(cells.size()) - 1);
  for (const auto & cell : cells) {
    width += painter.fontMetrics().horizontalAdvance(cell) + 40;
  }
  drawn_width_ = width;
  int x = (kWidth - width) / 2;
  for (const auto & cell : cells) {
    const int w = painter.fontMetrics().horizontalAdvance(cell) + 40;
    const QColor color = stateColor(cell.toStdString());
    painter.setPen(Qt::NoPen);
    painter.setBrush(color);
    painter.drawRoundedRect(x, 0, w, kHeight, 10, 10);
    painter.setPen(qGray(color.rgb()) > 160 ? Qt::black : Qt::white);
    painter.drawText(x, 0, w, kHeight, Qt::AlignCenter, cell);
    x += w + 8;
  }
  texture_->getBuffer()->blitFromMemory(
    Ogre::PixelBox(kWidth, kHeight, 1, Ogre::PF_A8R8G8B8, image.bits()));
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::OdomTypeDisplay, rviz_common::Display)

#include "rsf_rviz_plugins/view_switch_tool.hpp"

#include <QTimer>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

namespace rsf_rviz_plugins
{
ViewSwitchTool::ViewSwitchTool(const QString & name, const QString & view_class)
: name_(name), view_class_(view_class)
{
}

void ViewSwitchTool::onInitialize()
{
  setName(name_);
}

void ViewSwitchTool::activate()
{
  auto view_manager = context_->getViewManager();
  view_manager->setCurrentViewControllerType(view_class_);
  auto view = view_manager->getCurrent();
  view->subProp("Target Frame")->setValue("base_footprint");
  view->reset();
  view->subProp("Scale")->setValue(40.0);
  auto tool_manager = context_->getToolManager();
  QTimer::singleShot(0, [tool_manager]() {
    tool_manager->setCurrentTool(tool_manager->getDefaultTool());
  });
}

BirdEyeViewTool::BirdEyeViewTool()
: ViewSwitchTool("Bird's Eye View", "rviz_default_plugins/TopDownOrtho")
{
}

ThirdPersonViewTool::ThirdPersonViewTool()
: ViewSwitchTool("Third Person View", "rviz_default_plugins/ThirdPersonFollower")
{
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::BirdEyeViewTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::ThirdPersonViewTool, rviz_common::Tool)

#include "rsf_rviz_plugins/dummy_pause_tool.hpp"

#include <QTimer>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include "rsf_rviz_plugins/dummy_obstacle_tool.hpp"

namespace rsf_rviz_plugins
{
DummyPauseTool::DummyPauseTool()
{
  shortcut_key_ = ' ';
}

void DummyPauseTool::onInitialize()
{
  refreshName();
}

void DummyPauseTool::activate()
{
  auto & clock = DummyObstacleTool::pauseClock();
  const auto now = context_->getRosNodeAbstraction().lock()->get_raw_node()->now();
  clock.setPaused(!clock.paused(), now);
  refreshName();
  auto tool_manager = context_->getToolManager();
  tool_manager->refreshTool(this);
  QTimer::singleShot(0, [tool_manager]() {
    tool_manager->setCurrentTool(tool_manager->getDefaultTool());
  });
}

void DummyPauseTool::refreshName()
{
  setName(DummyObstacleTool::pauseClock().paused() ? "Resume Dummies" : "Pause Dummies");
}
}

PLUGINLIB_EXPORT_CLASS(rsf_rviz_plugins::DummyPauseTool, rviz_common::Tool)

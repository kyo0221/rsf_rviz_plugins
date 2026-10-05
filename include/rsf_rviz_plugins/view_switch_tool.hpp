#pragma once

#include <QString>
#include <rviz_common/tool.hpp>

namespace rsf_rviz_plugins
{
class ViewSwitchTool : public rviz_common::Tool
{
public:
  ViewSwitchTool(const QString & name, const QString & view_class);
  void onInitialize() override;
  void activate() override;
  void deactivate() override {}

private:
  QString name_;
  QString view_class_;
};

class BirdEyeViewTool : public ViewSwitchTool
{
public:
  BirdEyeViewTool();
};

class ThirdPersonViewTool : public ViewSwitchTool
{
public:
  ThirdPersonViewTool();
};
}

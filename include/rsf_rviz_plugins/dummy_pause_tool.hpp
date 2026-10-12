#pragma once

#include <rviz_common/tool.hpp>

namespace rsf_rviz_plugins
{
class DummyPauseTool : public rviz_common::Tool
{
public:
  DummyPauseTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override {}

private:
  void refreshName();
};
}

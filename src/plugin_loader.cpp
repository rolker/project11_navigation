#include "project11_navigation/plugin_loader.h"

namespace project11_navigation
{

  PluginLoader::PluginLoader():
    task_to_task_loader_("project11_navigation", "project11_navigation::TaskToTaskWorkflow"),
    task_to_twist_loader_("project11_navigation", "project11_navigation::TaskToTwistWorkflow")
  {
    
  }

} //namespace project11_navigation

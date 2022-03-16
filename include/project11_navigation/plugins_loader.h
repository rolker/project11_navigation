#ifndef PROJECT11_NAVIGATION_PLUGIN_LOADER_H
#define PROJECT11_NAVIGATION_PLUGIN_LOADER_H

#include "project11_navigation/interfaces/task_to_task_workflow.h"
#include "project11_navigation/interfaces/task_to_twist_workflow.h"
#include <pluginlib/class_loader.h>


namespace project11_navigation
{

class PluginsLoader
{
public:
  PluginsLoader();

  boost::shared_ptr<TaskToTaskWorkflow> getTaskToTaskPlugin(std::string name);
  boost::shared_ptr<TaskToTwistWorkflow> getTaskToTwistPlugin(std::string name);

  void configure(std::shared_ptr<Context> context);

private:
  template<typename T> struct PluginType
  {
    PluginType(std::string package, std::string base_class): loader(package, base_class){}
    pluginlib::ClassLoader<T> loader;
    std::map<std::string, boost::shared_ptr<T> > plugins;
    void load(std::string name, std::string type)
    {
      try
      {
        plugins[name] =  loader.createInstance(type);
      }
      catch(const std::exception& e)
      {
        ROS_FATAL("Failed to load the %s plugin of type %s, are you sure it is properly registered and that the containing library is built? Exception: %s", name.c_str(), type.c_str(), e.what());
        exit(1);
      }
    }
  };

  PluginType<TaskToTaskWorkflow> task_to_task_plugins_;
  PluginType<TaskToTwistWorkflow> task_to_twist_plugins_;
};

} // namespace project11_navigation

#endif

#include "project11_navigation/plugins_loader.h"

namespace project11_navigation
{

  PluginsLoader::PluginsLoader():
    task_to_task_plugins_("project11_navigation", "project11_navigation::TaskToTaskWorkflow"),
    task_to_twist_plugins_("project11_navigation", "project11_navigation::TaskToTwistWorkflow")
  {
    XmlRpc::XmlRpcValue plugins_param;
    if(ros::param::get("~plugins", plugins_param))
    {
      if(plugins_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        for(auto plugin_param: plugins_param)
        {
          if(plugin_param.second.getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            std::string plugin_type = plugin_param.second;
            if(task_to_task_plugins_.loader.isClassAvailable(plugin_type))
              task_to_task_plugins_.load(plugin_param.first, plugin_type);
            else if(task_to_twist_plugins_.loader.isClassAvailable(plugin_type))
              task_to_twist_plugins_.load(plugin_param.first, plugin_type);
            else
              ROS_WARN_STREAM("Could not load " << plugin_param.first << " plugin of type " << plugin_type);
          }
        }

      }
      else
        ROS_WARN("Unexpected type for plugins parameter. Excpected a struct.");
    }
    else
      ROS_WARN("Parameter 'plugins' not found.");

  }

  void PluginsLoader::configure(std::shared_ptr<Context> context)
  {
    for(auto p: task_to_task_plugins_.plugins)
      p.second->configure(p.first, context);
    for(auto p: task_to_twist_plugins_.plugins)
      p.second->configure(p.first, context);
  }

  boost::shared_ptr<TaskToTaskWorkflow> PluginsLoader::getTaskToTaskPlugin(std::string name)
  {
    if(task_to_task_plugins_.plugins.find(name) != task_to_task_plugins_.plugins.end())
      return task_to_task_plugins_.plugins[name];
    return boost::shared_ptr<TaskToTaskWorkflow>();
  }

  boost::shared_ptr<TaskToTwistWorkflow> PluginsLoader::getTaskToTwistPlugin(std::string name)
  {
    if(task_to_twist_plugins_.plugins.find(name) != task_to_twist_plugins_.plugins.end())
      return task_to_twist_plugins_.plugins[name];
    return boost::shared_ptr<TaskToTwistWorkflow>();
  }

} //namespace project11_navigation

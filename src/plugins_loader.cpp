#include "project11_navigation/plugins_loader.h"

namespace project11_navigation
{

  PluginsLoader::PluginsLoader()
  {
    addType<TaskToTaskWorkflow>("TaskToTaskWorkflow");
    addType<TaskToTwistWorkflow>("TaskToTwistWorkflow");
    addType<TaskListToTaskListWorkflow>("TaskListToTaskListWorkflow");
    addType<TaskListToTwistWorkflow>("TaskListToTwistWorkflow");
    //addType<TaskWrapper>("TaskWrapper");

    XmlRpc::XmlRpcValue plugins_param;
    if(ros::param::get("~plugins", plugins_param))
    {
      if(plugins_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        loadPlugins<TaskToTaskWorkflow>(plugins_param);
        loadPlugins<TaskToTwistWorkflow>(plugins_param);
        loadPlugins<TaskListToTaskListWorkflow>(plugins_param);
        loadPlugins<TaskListToTwistWorkflow>(plugins_param);
        //loadPlugins<TaskWrapper>(plugins_param);
      }
      else
        ROS_WARN("Unexpected type for plugins parameter. Excpected a struct.");
    }
    else
      ROS_WARN("Parameter 'plugins' not found.");

  }

  void PluginsLoader::configure(std::shared_ptr<Context> context)
  {
    configureType<TaskToTaskWorkflow>(context);
    configureType<TaskToTwistWorkflow>(context);
    configureType<TaskListToTaskListWorkflow>(context);
    configureType<TaskListToTwistWorkflow>(context);
    //configureType<TaskWrapper>(context);
  }
} //namespace project11_navigation

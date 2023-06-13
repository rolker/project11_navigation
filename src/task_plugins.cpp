#include "project11_navigation/task_plugins.h"
#include <ros/ros.h>

namespace project11_navigation
{

TaskPlugins::TaskPlugins():class_loader_("project11_navigation", "project11_navigation::Task")
{
  
}

void TaskPlugins::configure(std::shared_ptr<Context> context)
{
  XmlRpc::XmlRpcValue plugins_param;
  if(ros::param::get("~plugins", plugins_param))
  {
    if(plugins_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      if(plugins_param.hasMember("Tasks"))
      {
        auto tasks = plugins_param["Tasks"];
        for(auto task_type: tasks)
        {
          if(task_type.second.getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            std::string plugin_type = task_type.second;
            if(class_loader_.isClassAvailable(plugin_type))
              task_types_[task_type.first] = plugin_type;
            else
              ROS_WARN_STREAM(task_type.first << " plugin of type " << plugin_type << " not available");
          }
        }
      }
    }
    else
      ROS_WARN("Unexpected type for plugins parameter. Excpected a struct.");
  }
  else
    ROS_WARN("Parameter 'plugins' not found.");
  for(auto t: task_types_)
    ROS_INFO_STREAM("task: " << t.first << " type: " << t.second);
}

Task::Ptr TaskPlugins::operator()(const project11_nav_msgs::TaskInformation& task_information)
{
  Task::Ptr task;
  ROS_INFO_STREAM("task type: " << task_information.type);
  auto plugin_type = task_types_.find(task_information.type);
  ROS_INFO_STREAM("found? " << bool(plugin_type != task_types_.end()));
  if(plugin_type != task_types_.end())
    try
    {
      task = class_loader_.createInstance(plugin_type->second);
    }
    catch(const std::exception& e)
    {
       ROS_WARN_STREAM("Failed to load the " << plugin_type->first << " plugin of type " << plugin_type->second << ", are you sure it is properly registered and that the containing library is built? Exception: " << e.what());
    }
  if(!task)
    task = boost::make_shared<Task>();
  task->update(task_information, false);    
  return task;
}


} // namespace project11_navigation

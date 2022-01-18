#ifndef PROJECT11_NAVIGATION_PLUGIN_LOADER_H
#define PROJECT11_NAVIGATION_PLUGIN_LOADER_H

#include "project11_navigation/interfaces/task_to_task_workflow.h"
#include "project11_navigation/interfaces/task_to_twist_workflow.h"
#include <pluginlib/class_loader.h>


namespace project11_navigation
{

struct PluginLoader
{
  PluginLoader();

  pluginlib::ClassLoader<project11_navigation::TaskToTaskWorkflow> task_to_task_loader_;
  pluginlib::ClassLoader<project11_navigation::TaskToTwistWorkflow> task_to_twist_loader_;

};

} // namespace project11_navigation

#endif

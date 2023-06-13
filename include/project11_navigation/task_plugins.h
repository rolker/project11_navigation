#ifndef PROJECT11_NAVIGATION_TASK_PLUGINS_H
#define PROJECT11_NAVIGATION_TASK_PLUGINS_H

#include "project11_navigation/task.h"
#include <pluginlib/class_loader.h>

namespace project11_navigation
{

class TaskPlugins
{
public:
  TaskPlugins();

  void configure(std::shared_ptr<Context> context);

  Task::Ptr operator()(const project11_nav_msgs::TaskInformation& task_information);

private:
  pluginlib::ClassLoader<Task> class_loader_;
  std::map<std::string, std::string> task_types_;

};

} // namespace project11_navigation

#endif

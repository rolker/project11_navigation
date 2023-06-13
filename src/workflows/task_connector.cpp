#include "project11_navigation/workflows/task_connector.h"
#include "project11_navigation/utilities.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::TaskConnector, project11_navigation::TaskListToTaskListWorkflow)

namespace project11_navigation
{

void TaskConnector::configure(std::string name, Context::Ptr context)
{
  context_ = context;
}

void TaskConnector::setGoal(const std::shared_ptr<TaskList>& input)
{
  task_list_ = input;
  connected_ = connectTasks();
}

bool TaskConnector::running()
{
  return true;
  //return false;
}

bool TaskConnector::getResult(std::shared_ptr<TaskList>& output)
{
  connected_ = connectTasks();
  if(connected_)
    output = task_list_;
  return connected_;
}

bool TaskConnector::connectTasks()
{
  if(task_list_)
  {
    auto odom = context_->getOdometry();
    geometry_msgs::PoseStamped start_position;
    start_position.header = odom.header;
    start_position.pose = odom.pose.pose;
    for(auto t: task_list_->tasksByPriority())
      t->updateTransit(start_position, start_position, context_);
    return true;
  }
  return false;
}

} // namespace project11_navigation
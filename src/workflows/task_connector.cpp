#include "project11_navigation/workflows/task_connector.h"
#include "project11_navigation/utilities.h"
#include "project11_navigation/interfaces/task_wrapper.h"

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
  return false;
}

bool TaskConnector::getResult(std::shared_ptr<TaskList>& output)
{
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
    auto current_task = task_list_->getFirstTask();
    while(current_task)
    {
      ROS_INFO_STREAM("Need transit?\n" << current_task->message());
      auto tw = context_->getTaskWrapper(current_task);
      tw->updateTransit(start_position, start_position);
      current_task = task_list_->getNextTask(current_task);
    }
    return true;
  }
  return false;
}

} // namespace project11_navigation
#include "project11_navigation/workflows/task_manager.h"
#include "project11_navigation/context.h"
#include "project11_navigation/plugins_loader.h"
#include <queue>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::TaskManager, project11_navigation::TaskListToTwistWorkflow)

namespace project11_navigation
{

void TaskManager::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  task_connector_ = context_->pluginsLoader()->getPlugin<TaskListToTaskListWorkflow>("task_connector");
  executive_ = context_->pluginsLoader()->getPlugin<TaskToTwistWorkflow>("executive");
}

void TaskManager::setGoal(const std::shared_ptr<TaskList>& input)
{
  task_connector_->setGoal(input);
  task_connector_->getResult(task_list_);
  if(task_list_)
  {
    auto t = task_list_->getFirstTask();
    while(t)
    {
      ROS_INFO_STREAM("Task:\n" << t->message());
      t = task_list_->getNextTask(t);
    }
  }
  else
    ROS_INFO_STREAM("No task list!");

  updateCurrentTask();
}

bool TaskManager::running()
{
  updateCurrentTask();
  while(current_task_ && !executive_->running())
    updateCurrentTask();
  if(task_list_)
    return !task_list_->allDone();
  return false;
}

bool TaskManager::getResult(geometry_msgs::TwistStamped& output)
{
  updateCurrentTask();
  return executive_->getResult(output);
}

void TaskManager::updateCurrentTask()
{
  auto old_task = current_task_;
  std::map<int, std::vector<std::shared_ptr<Task> > > todo_list;
  if(task_list_)
    for(auto t: task_list_->tasks())
      if(!t->done())
        todo_list[t->message().priority].push_back(t);

  std::shared_ptr<Task> new_task;
  if(!todo_list.empty())
    new_task = todo_list.begin()->second.front();

  if(current_task_ != new_task)
  {
    executive_->setGoal(new_task);
    current_task_ = new_task;
  }
  if(old_task != current_task_)
  {
    if(old_task)
      ROS_INFO_STREAM("old task:\n" << old_task->message());
    else
      ROS_INFO_STREAM("old task: none");
    if(current_task_)
      ROS_INFO_STREAM("new task:\n" << current_task_->message());
    else
      ROS_INFO_STREAM("new task: none");
  }
}

}  // namespace project11_navigation
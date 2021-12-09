#include "project11_navigation/workflows/task_manager.h"
#include <queue>

namespace project11_navigation
{

void TaskManager::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  executive_ = std::make_shared<ExecuteTask>();
  executive_->configure("executive", context);
}

void TaskManager::setGoal(const std::shared_ptr<TaskList>& input)
{
  task_list_ = input;
  updateCurrentTask();
}

bool TaskManager::running()
{
  return executive_->running();
}

bool TaskManager::getResult(geometry_msgs::TwistStamped& output)
{
  updateCurrentTask();
  return executive_->getResult(output);
}

void TaskManager::updateCurrentTask()
{
  std::priority_queue<std::shared_ptr<Task> > todo_list;
  if(task_list_)
    for(auto t: task_list_->tasks())
      if(!t->done())
        todo_list.push(t);
  std::shared_ptr<Task> new_task;
  if(!todo_list.empty())
    new_task = todo_list.top();

  if(current_task_ != new_task)
  {
    executive_->setGoal(new_task);
    current_task_ = new_task;
  }
}

}  // namespace project11_navigation
#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/workflows/nav_core.h>
#include <project11_navigation/interfaces/task_wrapper.h>

namespace project11_navigation
{

ExecuteTask::~ExecuteTask()
{

}

void ExecuteTask::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  task_handlers_["transit"] = std::make_shared<NavCore>();
  task_handlers_["transit"]->configure("nav_core", context);
}

void ExecuteTask::setGoal(const std::shared_ptr<Task>& input)
{
  if(input != current_task_)
  {
    current_task_ = input;
    current_handler_.reset();
    updateCurrentHandler();
  }
}

bool ExecuteTask::running()
{
  if (current_handler_ && current_handler_->running())
    return true;
  return updateCurrentHandler();
}

bool ExecuteTask::getResult(geometry_msgs::TwistStamped& output)
{
  if(current_handler_)
    return current_handler_->getResult(output);

  return false;
}

bool ExecuteTask::updateCurrentHandler()
{
  if(current_task_)
  {
    auto tw = context_->getTaskWrapper(current_task_);
    if(tw)
    {
      current_nav_task_ = tw->getCurrentNavigationTask();
      current_handler_= task_handlers_[current_nav_task_->message().type];
      if(current_handler_)
        current_handler_->setGoal(current_nav_task_);
      else
      {
        current_nav_task_->setStatus("Skipped by ExecuteTask");
        current_nav_task_->setDone();
      }
    }
    else
      current_handler_.reset();
  }
  else
    current_handler_.reset();
  return current_handler_ && current_handler_->running();
}

}  // namespace project11_navigation
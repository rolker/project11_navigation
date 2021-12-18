#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/workflows/nav_core.h>

namespace project11_navigation
{

ExecuteTask::~ExecuteTask()
{

}

void ExecuteTask::configure(std::string name, Context::Ptr context)
{
  task_handlers_["transit"] = std::make_shared<NavCore>();
  task_handlers_["transit"]->configure("nav_core", context);
}

void ExecuteTask::setGoal(const std::shared_ptr<Task>& input)
{
  if(input != current_task_)
  {
    current_task_ = input;
    if(current_task_)
    {
      current_handler_= task_handlers_[input->message().type];
      if(current_handler_)
        current_handler_->setGoal(input);
      else
      {
        auto transit_to = current_task_->getFirstChildOfTypeAndID("transit","transit_to");
        if(transit_to && !transit_to->done())
        {
          current_handler_= task_handlers_["transit"];
          current_handler_->setGoal(transit_to);
        }
        else
        {
          current_task_->setStatus("Skipped by ExecuteTask");
          current_task_->setDone();
        }
      }
    }
    else
      current_handler_.reset();
  }
}
bool ExecuteTask::running()
{
  if (current_handler_)
    return current_handler_->running();
  return false;
}

bool ExecuteTask::getResult(geometry_msgs::TwistStamped& output)
{
  if(current_handler_)
    return current_handler_->getResult(output);

  return false;
}

}  // namespace project11_navigation
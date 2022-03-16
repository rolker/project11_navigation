#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/workflows/nav_core.h>
#include <project11_navigation/interfaces/task_wrapper.h>
#include <project11_navigation/workflows/task_to_twist_stack.h>
#include <project11_navigation/plugins_loader.h>

namespace project11_navigation
{

ExecuteTask::ExecuteTask()
{
  
}

ExecuteTask::~ExecuteTask()
{

}

void ExecuteTask::configure(std::string name, Context::Ptr context)
{
  context_ = context;

  auto loader = context_->pluginsLoader();

  task_handlers_["transit"] = loader->getTaskToTwistPlugin("transit");
  task_handlers_["hover"] = loader->getTaskToTwistPlugin("hover");

  task_handlers_["survey_line"] = boost::make_shared<TaskToTwistStack>();
  task_handlers_["survey_line"]->configure("survey_line", context);

}

void ExecuteTask::setGoal(const std::shared_ptr<Task>& input)
{
  if(input != current_task_)
  {
    current_task_ = input;
    updateCurrentHandler();
  }
}

bool ExecuteTask::running()
{
  if(current_task_)
  {
    while (!current_task_->done() && !updateCurrentHandler())
    {
    }
    return !current_task_->done();
  }
  return false;
}

bool ExecuteTask::getResult(geometry_msgs::TwistStamped& output)
{
  if(running())
    return current_handler_->getResult(output);

  return false;
}

bool ExecuteTask::updateCurrentHandler()
{
  auto old_handler = current_handler_;
  if(current_task_)
  {
    auto tw = context_->getTaskWrapper(current_task_);
    if(tw)
    {
      auto old_nav_task = current_nav_task_;
      current_nav_task_ = tw->getCurrentNavigationTask();
      if(current_nav_task_)
      {
        if(current_nav_task_ != old_nav_task)
        {
          ROS_INFO_STREAM("Current nav task:" << current_nav_task_->message());
          current_handler_= task_handlers_[current_nav_task_->message().type];
          if(current_handler_)
            current_handler_->setGoal(current_nav_task_);
          else
          {
            current_nav_task_->setStatus("Skipped by ExecuteTask");
            current_nav_task_->setDone();
          }
        }
      }
      else
      {
        current_handler_.reset();
        current_task_->setDone();  
      }
    }
    else
    {
      current_handler_.reset();
      current_task_->setStatus("Skipped by ExecuteTask");
      current_task_->setDone();
    }
  }
  else
    current_handler_.reset();
  if(old_handler && old_handler != current_handler_)
    old_handler->setGoal(std::shared_ptr<Task>());
  return current_handler_ && current_handler_->running();
}

}  // namespace project11_navigation
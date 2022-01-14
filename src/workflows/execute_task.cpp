#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/workflows/nav_core.h>
#include <project11_navigation/interfaces/task_wrapper.h>

namespace project11_navigation
{

ExecuteTask::ExecuteTask():
  task_handler_loader_("project11_navigation", "project11_navigation::TaskToTwistWorkflow")
{
  
}

ExecuteTask::~ExecuteTask()
{

}

void ExecuteTask::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  task_handlers_["transit"] = boost::make_shared<NavCore>();
  task_handlers_["transit"]->configure("nav_core", context);

  std::string hover_handler = "hover/Hover";
  try
  {
    auto handler = task_handler_loader_.createInstance(hover_handler);
    handler->configure(task_handler_loader_.getName(hover_handler), context);
    task_handlers_["hover"] = handler;
  }
  catch(const std::exception& e)
  {
    ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", hover_handler.c_str(), e.what());
    exit(1);
  }
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
  return updateCurrentHandler();
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
      current_handler_.reset();
  }
  else
    current_handler_.reset();
  if(old_handler && old_handler != current_handler_)
    old_handler->setGoal(std::shared_ptr<Task>());
  return current_handler_ && current_handler_->running();
}

}  // namespace project11_navigation
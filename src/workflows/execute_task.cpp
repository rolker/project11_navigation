#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/workflows/nav_core.h>
#include <project11_navigation/workflows/task_to_twist_stack.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::ExecuteTask, project11_navigation::TaskToTwistWorkflow)

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

  XmlRpc::XmlRpcValue parameters;
  if(ros::param::get("~"+name, parameters))
  {
    if(parameters.hasMember("workflows"))
      for(auto h: parameters["workflows"])
        task_workflows_[h.first] = std::string(h.second);
  }
}

void ExecuteTask::setGoal(const boost::shared_ptr<Task>& input)
{
  if(input != current_task_)
  {
    current_task_ = input;
    updateCurrentWorkflow();
  }
}

bool ExecuteTask::running()
{
  if(current_task_)
  {
    while (!current_task_->done() && !updateCurrentWorkflow())
    {
    }
    return !current_task_->done();
  }
  return false;
}

bool ExecuteTask::getResult(geometry_msgs::TwistStamped& output)
{
  if(running())
    return current_workflow_->getResult(output);

  return false;
}

bool ExecuteTask::updateCurrentWorkflow()
{
  auto old_workflow = current_workflow_;
  if(current_task_)
  {
    auto old_nav_task = current_nav_task_;
    current_nav_task_ = current_task_->getCurrentNavigationTask();
    if(current_nav_task_)
    {
      if(current_nav_task_ != old_nav_task)
      {
        ROS_INFO_STREAM("Current nav task:" << current_nav_task_->message());
        std::string task_type = current_nav_task_->message().type;
        if(task_workflows_.find(task_type) != task_workflows_.end())
          current_workflow_ = context_->pluginsLoader()->getPlugin<TaskToTwistWorkflow>(task_workflows_[task_type]);
        else
          current_workflow_ = context_->pluginsLoader()->getPlugin<TaskToTwistWorkflow>(task_type);
        if(current_workflow_)
        {
          if(old_workflow && old_workflow != current_workflow_)
            old_workflow->setGoal(boost::shared_ptr<Task>());
          current_workflow_->setGoal(current_nav_task_);
        }
        else
        {
          auto status = current_nav_task_->status();
          status["skipped"] = "Skipped by ExecuteTask";
          current_nav_task_->setStatus(status);
          current_nav_task_->setDone();
        }
      }
    }
    else
    {
      current_workflow_.reset();
      current_task_->setDone();  
    }
  }
  else
    current_workflow_.reset();

  return current_workflow_ && current_workflow_->running();
}

}  // namespace project11_navigation
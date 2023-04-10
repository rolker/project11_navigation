#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/workflows/nav_core.h>
#include <project11_navigation/interfaces/task_wrapper.h>
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
    if(parameters.hasMember("handlers"))
      for(auto h: parameters["handlers"])
        task_handlers_[h.first] = std::string(h.second);
  }
  display_pub_ = ros::NodeHandle("~").advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
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
      visualization_msgs::MarkerArray marker_array;
      if(current_nav_task_)
        for(auto m: current_nav_task_->markerArray().markers)
          marker_array.markers.push_back(m);
      display_pub_.publish(marker_array);
      if(current_nav_task_)
      {
        if(current_nav_task_ != old_nav_task)
        {
          ROS_INFO_STREAM("Current nav task:" << current_nav_task_->message());
          std::string task_type = current_nav_task_->message().type;
          if(task_handlers_.find(task_type) != task_handlers_.end())
            current_handler_ = context_->pluginsLoader()->getPlugin<TaskToTwistWorkflow>(task_handlers_[task_type]);
          else
            current_handler_ = context_->pluginsLoader()->getPlugin<TaskToTwistWorkflow>(task_type);
          if(current_handler_)
          {
            if(old_handler && old_handler != current_handler_)
              old_handler->setGoal(std::shared_ptr<Task>());
            current_handler_->setGoal(current_nav_task_);
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
        current_handler_.reset();
        current_task_->setDone();  
      }
    }
    else
    {
      current_handler_.reset();
      auto status = current_task_->status();
      status["skipped"] = "Skipped by ExecuteTask";
      current_task_->setStatus(status);
      current_task_->setDone();
    }
  }
  else
    current_handler_.reset();
  return current_handler_ && current_handler_->running();
}

}  // namespace project11_navigation
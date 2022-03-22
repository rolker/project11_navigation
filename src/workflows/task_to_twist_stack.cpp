#include "project11_navigation/workflows/task_to_twist_stack.h"
#include "project11_navigation/plugins_loader.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::TaskToTwistStack, project11_navigation::TaskToTwistWorkflow)

namespace project11_navigation
{

void TaskToTwistStack::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh("~/" + name);

  std::string last_step;
  if(nh.getParam("last_step", last_step))
    last_step_ = context->pluginsLoader()->getPlugin<TaskToTwistWorkflow>(last_step);
  if(!last_step_)
    ROS_WARN_STREAM("Unable to get last step plugin: " << last_step);
  
  std::vector<std::string> steps;
  nh.getParam("steps", steps);
  for(auto step: steps)
  {
    auto sp = context_->pluginsLoader()->getPlugin<TaskToTaskWorkflow>(step);
    if(sp)
      steps_.push_back(sp);
    else
      ROS_WARN_STREAM("Unable to get step plugin: " << step);
  }
}

void TaskToTwistStack::setGoal(const std::shared_ptr<Task>& input)
{
  current_task_ = input;
  sub_tasks_.clear();
  sub_tasks_.resize(steps_.size());
  if(!steps_.empty())
    steps_.front()->setGoal(input);
  else
    last_step_->setGoal(input);
  iterate();
}

bool TaskToTwistStack::running()
{
  iterate();
  for(auto step: steps_)
    if(step->running())
      return true;
  return last_step_->running();
}

bool TaskToTwistStack::getResult(geometry_msgs::TwistStamped& output)
{
  iterate();
  return last_step_->getResult(output);
}

void TaskToTwistStack::iterate()
{
  for(int i = 0; i < steps_.size(); i++)
  {
    if(steps_[i]->getResult(sub_tasks_[i]))
      if(i < steps_.size()-1)
        steps_[i+1]->setGoal(sub_tasks_[i]);
      else
        last_step_->setGoal(sub_tasks_[i]);
    if(sub_tasks_[i] && sub_tasks_[i]->done())
      if(i==0)
        current_task_->setDone();
      else
        sub_tasks_[i-1]->setDone();
  }
}

} // namespace project11_navigation

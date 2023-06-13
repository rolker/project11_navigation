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
  ROS_INFO_STREAM("Stack "<<name<<", last step: "<< last_step);
  
  std::vector<std::string> steps;
  nh.getParam("steps", steps);
  for(auto step: steps)
  {
    ROS_INFO_STREAM("step: "<< step);
    auto sp = context_->pluginsLoader()->getPlugin<TaskToTaskWorkflow>(step);
    if(sp)
      steps_.push_back(sp);
    else
      ROS_WARN_STREAM("Unable to get step plugin: " << step);
  }
}

void TaskToTwistStack::setGoal(const boost::shared_ptr<Task>& input)
{
  if(input != current_task_)
  {
    boost::shared_ptr<Task> null_task;
    for(auto s: steps_)
      s->setGoal(null_task);
    last_step_->setGoal(null_task);
  }
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
  bool ret = false; // don't short circuit so all the steps' running() are called.
  for(auto step: steps_)
    ret |= step->running();
  ret |= last_step_->running();
  return ret;
}

bool TaskToTwistStack::getResult(geometry_msgs::TwistStamped& output)
{
  iterate();
  return last_step_->getResult(output);
}

void TaskToTwistStack::iterate()
{
  if(!current_task_)
    return;
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

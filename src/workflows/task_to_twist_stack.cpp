#include "project11_navigation/workflows/task_to_twist_stack.h"
#include "project11_navigation/plugin_loader.h"

namespace project11_navigation
{

void TaskToTwistStack::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh("~/" + name);

  std::string last_step = "path_follower/PathFollower";
  nh.param("last_step", last_step, last_step);
  
  last_step_ = context->pluginLoader()->task_to_twist_loader_.createInstance(last_step);
  last_step_->configure(name+"/"+context->pluginLoader()->task_to_twist_loader_.getName(last_step), context);
}

void TaskToTwistStack::setGoal(const std::shared_ptr<Task>& input)
{
  current_task_ = input;
  last_step_->setGoal(input);
}

bool TaskToTwistStack::running()
{
  return last_step_->running();
}

bool TaskToTwistStack::getResult(geometry_msgs::TwistStamped& output)
{
  return last_step_->getResult(output);
}

} // namespace project11_navigation

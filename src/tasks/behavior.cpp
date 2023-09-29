#include "project11_navigation/tasks/behavior.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::BehaviorTask, project11_navigation::Task)

namespace project11_navigation
{

void BehaviorTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  geometry_msgs::PoseStamped pose = from_pose;
  for(auto t: children().tasksByPriority())
  {
    t->updateTransit(pose, out_pose, context);
    pose = out_pose;
  }
  out_pose = pose;
}


boost::shared_ptr<Task> BehaviorTask::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    auto nav_task = t->getCurrentNavigationTask();
    if(nav_task)
      return nav_task;
  }
  return nullptr;
}

}

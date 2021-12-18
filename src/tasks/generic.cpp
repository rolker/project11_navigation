#include <project11_navigation/tasks/generic.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

bool GenericTask::needsTransit(const geometry_msgs::PoseStamped& from, geometry_msgs::PoseStamped& to)
{
  geometry_msgs::PoseStamped start_position;
  if(task_->getFirstPose(start_position));
  {
    if(length(vectorBetween(from.pose, start_position.pose))>10.0)
    {
      to = start_position;
      return true;
    }
  }
  return false;
}

geometry_msgs::PoseStamped GenericTask::expectedEndPose(const geometry_msgs::PoseStamped& starting_pose)
{
  if(!task_->message().poses.empty())
    return task_->message().poses.back();
  return starting_pose;
}

void GenericTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  geometry_msgs::PoseStamped in_pose;
  if(task_->getFirstPose(in_pose))
  {
    if(length(vectorBetween(from_pose.pose, in_pose.pose))>10.0)
      task_->updateTransitTo(in_pose);
  }
  else 
    in_pose = from_pose;

  if(!task_->getLastPose(out_pose))
    out_pose = in_pose;
}

} // namespace project11_navigation
#include <project11_navigation/tasks/transit.h>

namespace project11_navigation
{

bool TransitTask::needsTransit(const geometry_msgs::PoseStamped& from, geometry_msgs::PoseStamped& to)
{
  return false;
}

geometry_msgs::PoseStamped TransitTask::expectedEndPose(const geometry_msgs::PoseStamped& starting_pose)
{
  if(!task_->message().poses.empty())
    return task_->message().poses.back();
  return starting_pose;
}

void TransitTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  if(!task_->getLastPose(out_pose))
    out_pose = from_pose;
}


} // namespace project11_navigation

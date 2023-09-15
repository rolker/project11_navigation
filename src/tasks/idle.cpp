#include <project11_navigation/tasks/idle.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::IdleTask, project11_navigation::Task)

namespace project11_navigation
{

void IdleTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  out_pose = from_pose;
}

boost::shared_ptr<Task> IdleTask::getCurrentNavigationTask()
{
  return self();
}

} // namespace project11_navigation

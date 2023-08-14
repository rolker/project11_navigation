#include <project11_navigation/tasks/goto.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::GotoTask, project11_navigation::Task)

namespace project11_navigation
{

void GotoTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  auto current_pose = from_pose;
  if(getFirstPose(current_pose))
  {
    if(length(vectorBetween(from_pose.pose, current_pose.pose)) > context->getRobotCapabilities().waypoint_reached_distance)
    {
      auto transit = updateTransitTo(from_pose, current_pose);
      auto preview_planner = context->pluginsLoader()->getPlugin<TaskToTaskWorkflow>("preview");
      if(preview_planner)
      {
        preview_planner->setGoal(transit);
        boost::shared_ptr<Task> plan;
        preview_planner->getResult(plan);
      }
    }
    else
      clearTransitTo();
  }
  else
    current_pose = from_pose;

  out_pose = current_pose;
}

boost::shared_ptr<Task> GotoTask::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return self();
}


} // namespace project11_navigation

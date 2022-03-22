#include <project11_navigation/tasks/generic.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::GenericTask, project11_navigation::TaskWrapper)

namespace project11_navigation
{

void GenericTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  geometry_msgs::PoseStamped in_pose;
  if(task_->getFirstPose(in_pose))
  {
    if(length(vectorBetween(from_pose.pose, in_pose.pose))>10.0)
    {
      auto transit = task_->updateTransitTo(from_pose, in_pose);
      auto preview_planner = context_->pluginsLoader()->getPlugin<TaskToTaskWorkflow>("preview");
      if(preview_planner)
      {
        preview_planner->setGoal(transit);
        std::shared_ptr<Task> plan;
        preview_planner->getResult(plan);
      }
    }
  }
  else 
    in_pose = from_pose;

  if(!task_->getLastPose(out_pose))
    out_pose = in_pose;
}

std::shared_ptr<Task> GenericTask::getCurrentNavigationTask()
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return std::shared_ptr<Task>();
}

void GenericTask::configure(std::string name, std::shared_ptr<Context> context)
{
  
}

void GenericTask::getPreviewDisplay(visualization_msgs::MarkerArray& marker_array)
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    auto tw = context_->getTaskWrapper(t);
    if(tw)
      tw->getPreviewDisplay(marker_array);
  }
}

} // namespace project11_navigation

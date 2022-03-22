#include <project11_navigation/tasks/hover.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::HoverTask, project11_navigation::TaskWrapper)

namespace project11_navigation
{

void HoverTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
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

  out_pose = in_pose;
}

std::shared_ptr<Task> HoverTask::getCurrentNavigationTask()
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return task_;
}

void HoverTask::configure(std::string name, std::shared_ptr<Context> context)
{
  
}

void HoverTask::getPreviewDisplay(visualization_msgs::MarkerArray& marker_array)
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    auto tw = context_->getTaskWrapper(t);
    if(tw)
      tw->getPreviewDisplay(marker_array);
  }

  geometry_msgs::PoseStamped pose;
  if(task_->getFirstPose(pose))
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = marker_array.markers.size();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose = pose.pose;
    marker.color.r = .25;
    marker.color.g = .4;
    marker.color.b = .25;
    marker.color.a = .5;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker_array.markers.push_back(marker);
  }
}

} // namespace project11_navigation

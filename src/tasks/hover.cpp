#include <project11_navigation/tasks/hover.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::HoverTask, project11_navigation::Task)

namespace project11_navigation
{

void HoverTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  geometry_msgs::PoseStamped in_pose;
  if(getFirstPose(in_pose))
  {
    if(length(vectorBetween(from_pose.pose, in_pose.pose)) > context->navigatorSettings().waypoint_reached_distance)
    {
      auto transit = updateTransitTo(from_pose, in_pose);
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
    in_pose = from_pose;

  out_pose = in_pose;
}

boost::shared_ptr<Task> HoverTask::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return self();
}


void HoverTask::getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const
{
  Task::getDisplayMarkers(marker_array);

  geometry_msgs::PoseStamped pose;
  if(getFirstPose(pose))
  {
    visualization_msgs::Marker marker;
    ROS_DEBUG_STREAM_NAMED("display", "frame_id: " << pose.header.frame_id);
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = 1;
    marker.ns = message().id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose = pose.pose;
    marker.color.r = .0;
    marker.color.g = .5;
    marker.color.b = .0;
    marker.color.a = .5;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker.lifetime = ros::Duration(2.0);
    marker_array.markers.push_back(marker);
  }
}

} // namespace project11_navigation

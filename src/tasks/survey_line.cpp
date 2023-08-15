#include <project11_navigation/tasks/survey_line.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::SurveyLineTask, project11_navigation::Task)

namespace project11_navigation
{

void SurveyLineTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  if(message().poses.size() >= 2)
  {
    auto first_segment_direction_vector = normalize(vectorBetween(message().poses[0].pose, message().poses[1].pose));

    double lead_in_distance = context->navigatorSettings().survey_lead_in_distance;

    geometry_msgs::PoseStamped in_pose = message().poses[0];
    in_pose.pose.position.x -= first_segment_direction_vector.x*lead_in_distance;
    in_pose.pose.position.y -= first_segment_direction_vector.y*lead_in_distance;
    in_pose.pose.position.z -= first_segment_direction_vector.z*lead_in_distance;

    if(length(vectorBetween(from_pose.pose, in_pose.pose)) > context->navigatorSettings().waypoint_reached_distance)
      updateTransitTo(from_pose, in_pose);
    else
      clearTransitTo();
  }
  else
  {
    geometry_msgs::PoseStamped in_pose;
    if(getFirstPose(in_pose))
    {
      if(length(vectorBetween(from_pose.pose, in_pose.pose)) > context->navigatorSettings().waypoint_reached_distance)
        updateTransitTo(from_pose, in_pose);
    }
    else 
      in_pose = from_pose;
  }
  if(!getLastPose(out_pose))
    out_pose = from_pose;
}

boost::shared_ptr<Task> SurveyLineTask::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return self();
}


void SurveyLineTask::getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const
{
  Task::getDisplayMarkers(marker_array);

  geometry_msgs::PoseStamped pose;
  if(getFirstPose(pose))
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = marker_array.markers.size();
    marker.ns = "survey_line";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.color.r = .25;
    marker.color.g = .4;
    marker.color.b = .25;
    marker.color.a = .5;
    marker.scale.x = 1.0;
    for(auto p: message().poses)
      marker.points.push_back(p.pose.position);
    marker_array.markers.push_back(marker);
  }
}


} // namespace project11_navigation

#include <project11_navigation/tasks/transit.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::TransitTask, project11_navigation::Task)

namespace project11_navigation
{

void TransitTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  if(getLastPose(out_pose))
    out_pose = from_pose;
}

boost::shared_ptr<Task> TransitTask::getCurrentNavigationTask()
{
  return self();
}


void TransitTask::getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const
{
  std::vector<geometry_msgs::PoseStamped> poses;
  for(auto t: children().tasksByPriority(true))
  {
    if (t->message().type == "follow_trajectory")
    {
      poses = t->message().poses;
      break;
    }
  }

  if(poses.empty())
    poses = message().poses;

  if(!poses.empty())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = poses.front().header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = marker_array.markers.size();
    marker.ns = message().id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.color.r = .25;
    marker.color.g = .25;
    marker.color.b = .4;
    marker.color.a = .5;
    marker.scale.x = 1.0;
    for(auto p: poses)
      marker.points.push_back(p.pose.position);
    marker_array.markers.push_back(marker);
  }
}


} // namespace project11_navigation

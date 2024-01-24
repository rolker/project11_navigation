#include <project11_navigation/robot.h>
#include <project11_navigation/robot_capabilities.h>

namespace project11_navigation
{

Robot::Robot()
{
  ros::NodeHandle nh("~");
  cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

  enable_sub_ = nh.subscribe<std_msgs::Bool>("enable", 10, &Robot::enableCallback, this);
}

void Robot::sendControls(const geometry_msgs::TwistStamped& cmd_vel) const
{
  if(enabled_)
    cmd_vel_pub_.publish(cmd_vel);
}

bool Robot::enabled() const
{
  return enabled_;
}

void Robot::enableCallback(const std_msgs::BoolConstPtr& msg)
{
  enabled_ = msg->data;
}

void Robot::updateMarkers(visualization_msgs::MarkerArray& marker_array, const RobotCapabilities& robot_capabilities) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = odom_.header.frame_id;
  marker.header.stamp = odom_.header.stamp;
  marker.id = 0;
  marker.ns = odom_.header.frame_id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.pose = odom_.pose.pose;
  marker.color.r = .75;
  marker.color.g = .75;
  marker.color.b = .25;
  marker.color.a = 1.0;
  marker.scale.x = 0.2;
  for(auto p: robot_capabilities.footprint)
  {
    marker.points.push_back(p);
  }
  marker_array.markers.push_back(marker);

}

} // namespace project11_navigation

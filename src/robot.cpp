#include <project11_navigation/robot.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

Robot::Robot(Context::Ptr context)
  :context_(context)
{
  ros::NodeHandle nh("~");
  cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

  enable_sub_ = nh.subscribe<std_msgs::Bool>("enable", 10, &Robot::enableCallback, this);

  XmlRpc::XmlRpcValue value;
  if(nh.getParam("robot/turn_radius", value))
  {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
      for(int i = 0; i < value.size(); ++i)
        capabilities_.turn_radius_map[static_cast<double>(value[i]["velocity"])] = static_cast<double>(value[i]["radius"]);
    else
      nh.getParam("robot/turn_radius", capabilities_.turn_radius_map[0.0]);
  }

  readLinearAngularParameters(nh, "robot/max_velocity", capabilities_.max_velocity, capabilities_.max_velocity);
  readLinearAngularParameters(nh, "robot/min_velocity", capabilities_.min_velocity, capabilities_.min_velocity);
  readLinearAngularParameters(nh, "robot/default_velocity", capabilities_.default_velocity, capabilities_.default_velocity);

  readLinearAngularParameters(nh, "robot/max_acceleration", capabilities_.max_acceleration, capabilities_.max_acceleration);
  readLinearAngularParameters(nh, "robot/max_deceleration", capabilities_.max_deceleration, capabilities_.max_deceleration);

  if(nh.getParam("robot/footprint", value))
  {
    if(value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for(int i = 0; i < value.size(); ++i)
      {
        if(value[i].getType() == XmlRpc::XmlRpcValue::TypeArray && value[i].size() == 2)
        {
          geometry_msgs::Point p;
          if(value[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            p.x = static_cast<double>(value[i][0]);
          else
            p.x = static_cast<int>(value[i][0]);
          if(value[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            p.y = static_cast<double>(value[i][1]);
          else
            p.y = static_cast<int>(value[i][1]);
          capabilities_.footprint.push_back(p);
        }
        else
          ROS_ERROR_STREAM("Expected an array of 2 values in footprint point number " << i);
      }
    }
    else
      ROS_ERROR_STREAM("Expected an array of points for the footprint");
    for(auto p: capabilities_.footprint)
      capabilities_.radius = std::max(capabilities_.radius, sqrt(p.x*p.x + p.y*p.y));
  }

  capabilities_.radius = readDoubleOrIntParameter(nh, "robot/radius", capabilities_.radius);

  context_->updateRobotCapabilities(capabilities_);
}

void Robot::sendControls(const geometry_msgs::TwistStamped& cmd_vel)
{
  cmd_vel_pub_.publish(cmd_vel);
}

void Robot::enableCallback(const std_msgs::BoolConstPtr& msg)
{
  enabled_ = msg->data;
  context_->updateOutputEnabled(enabled_);
}

void Robot::odometryUpdated()
{
  context_->updateOdometry(odom_);
}

void Robot::updateMarkers(visualization_msgs::MarkerArray& marker_array)
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
  for(auto p: capabilities_.footprint)
  {
    marker.points.push_back(p);
  }
  marker_array.markers.push_back(marker);

}

} // namespace project11_navigation

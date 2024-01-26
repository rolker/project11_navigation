#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/utilities.h>
#include <ros/ros.h>

namespace project11_navigation
{

RobotCapabilities::RobotCapabilities(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue value;
  if(nh.getParam("robot/turn_radius", value))
  {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
      for(int i = 0; i < value.size(); ++i)
        turn_radius_map[static_cast<double>(value[i]["velocity"])] = static_cast<double>(value[i]["radius"]);
    else
      nh.getParam("robot/turn_radius", turn_radius_map[0.0]);
  }

  readLinearAngularParameters(nh, "robot/max_velocity", max_velocity, max_velocity);
  readLinearAngularParameters(nh, "robot/min_velocity", min_velocity, min_velocity);
  readLinearAngularParameters(nh, "robot/default_velocity", default_velocity, default_velocity);

  readLinearAngularParameters(nh, "robot/max_acceleration", max_acceleration, max_acceleration);
  readLinearAngularParameters(nh, "robot/default_acceleration", default_acceleration, default_acceleration);

  readLinearAngularParameters(nh, "robot/max_deceleration", max_deceleration, max_deceleration);
  readLinearAngularParameters(nh, "robot/default_deceleration", default_deceleration, default_deceleration);

  if(nh.getParam("robot/footprint", value))
  {
    if(value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for(int i = 0; i < value.size(); ++i)
      {
        if(value[i].getType() == XmlRpc::XmlRpcValue::TypeArray && value[i].size() == 2)
        {
          geometry_msgs::Point32 p;
          if(value[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            p.x = static_cast<double>(value[i][0]);
          else
            p.x = static_cast<int>(value[i][0]);
          if(value[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            p.y = static_cast<double>(value[i][1]);
          else
            p.y = static_cast<int>(value[i][1]);
          footprint.points.push_back(p);
        }
        else
          ROS_ERROR_STREAM("Expected an array of 2 values in footprint point number " << i);
      }
    }
    else
      ROS_ERROR_STREAM("Expected an array of points for the footprint");
    for(auto p: footprint.points)
      radius = std::max(radius, sqrt(p.x*p.x + p.y*p.y));
  }

  radius = readDoubleOrIntParameter(nh, "robot/radius", radius);

}

double RobotCapabilities::getTurnRadiusAtSpeed(double speed) const
{
  if(!turn_radius_map.empty())
  {
    if(turn_radius_map.size() == 1)
      return turn_radius_map.begin()->second;
    auto low = turn_radius_map.begin();
    if(speed <= low->first)
      return low->second;
    auto high = low;
    ++high;
    while (high != turn_radius_map.end() && high->first < speed)
    {
      low = high;
      ++high;
    }
    if(high == turn_radius_map.end())
      return low->second;
    if(high->first <= speed)
      return high->second;
    return low->second + ((speed-low->first)/(high->first-low->first))*(high->second-low->second);
  }
  return 0.0;
}



}
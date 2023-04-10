#include <project11_navigation/utilities.h>

namespace project11_navigation
{

void adjustTrajectoryForSpeed(std::vector<geometry_msgs::PoseStamped>& trajectory, double speed)
{
  double total_distance = 0.0;
  auto last = trajectory.begin();
  auto next = last;
  if(next != trajectory.end())
    next++;
  while(next != trajectory.end())
  {
    double dx = next->pose.position.x - last->pose.position.x;
    double dy = next->pose.position.y - last->pose.position.y;
    double distance = sqrt(dx*dx+dy*dy);
    total_distance += distance;
    next->header.stamp = trajectory.front().header.stamp+ros::Duration(total_distance/speed);
    last = next;
    next++;
  }

}

geometry_msgs::Vector3 vectorBetween(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to)
{
  geometry_msgs::Vector3 ret;

  ret.x = to.position.x - from.position.x;
  ret.y = to.position.y - from.position.y;
  ret.z = to.position.z - from.position.z;

  return ret;
}

double length(const geometry_msgs::Vector3& vector)
{
  double sum = vector.x*vector.x + vector.y*vector.y + vector.z*vector.z;
  if(sum > 0.0)
    return sqrt(sum);
  return 0.0;
}

geometry_msgs::Vector3 normalize(const geometry_msgs::Vector3& vector)
{
  geometry_msgs::Vector3 ret;
  double l = length(vector);
  if(l>0.0)
  {
    ret.x = vector.x/l;
    ret.y = vector.y/l;
    ret.z = vector.z/l;
  }
  return ret;
}

double readDoubleOrIntParameter(ros::NodeHandle &nh, const std::string& parameter, double default_value)
{
  XmlRpc::XmlRpcValue value;
  if(nh.getParam(parameter, value))
  {
    if(value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      return static_cast<double>(value);
    if(value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      return static_cast<int>(value);
    ROS_FATAL_STREAM("Expected number for parameter " << parameter << " but got " << std::string(value));
    throw std::runtime_error("Could not read number from parameter");
  }
  return default_value;
}


} // namespace project11_navigation

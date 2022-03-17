#include <project11_navigation/platform.h>

namespace project11_navigation
{

Platform::Platform()
{
  odom_sub_ = nodeHandle_.subscribe("odom", 10, &Platform::odometryCallback, this);
}

void Platform::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_ = *msg;
  odometryUpdated();
}

std::string Platform::baseFrame() const
{
  return odom_.child_frame_id;
}

void Platform::odometryUpdated()
{
  
}

} // namespace project11_navigation

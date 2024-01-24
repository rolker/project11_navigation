#ifndef PROJECT11_NAVIGATION_PLATFORM_H
#define PROJECT11_NAVIGATION_PLATFORM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace project11_navigation
{

/// Represents a platfrom that can be tracked in ROS using Odometry messages.
class Platform
{
public:
  Platform();
  std::string baseFrame() const;
  const nav_msgs::Odometry &odometry() const;

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

protected:
  ros::NodeHandle nodeHandle_;
  nav_msgs::Odometry odom_;

private:
  ros::Subscriber odom_sub_;
};

} // namespace project11_navigation

#endif

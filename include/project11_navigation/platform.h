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

protected:
  ros::NodeHandle nodeHandle_;
  nav_msgs::Odometry odom_;
  virtual void odometryUpdated();

private:
  ros::Subscriber odom_sub_;
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

} // namespace project11_navigation

#endif

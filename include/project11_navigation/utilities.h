#ifndef PROJECT11_NAVIGATION_UTILITIES_H
#define PROJECT11_NAVIGATION_UTILITIES_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>

namespace project11_navigation
{

template<typename T>
void readLinearAngularParameters(ros::NodeHandle &nh, const std::string& parameter, T& value, const T& default_value)
{
  value.linear.x = nh.param(parameter+"/linear/x", default_value.linear.x);
  value.linear.y = nh.param(parameter+"/linear/y", default_value.linear.y);
  value.linear.z = nh.param(parameter+"/linear/z", default_value.linear.z);
  value.angular.x = nh.param(parameter+"/angular/x", default_value.angular.x);
  value.angular.y = nh.param(parameter+"/angular/y", default_value.angular.y);
  value.angular.z = nh.param(parameter+"/angular/z", default_value.angular.z);
}

void adjustTrajectoryForSpeed(std::vector<geometry_msgs::PoseStamped>& trajectory, double speed);

} // namespace project11_navigation

#endif

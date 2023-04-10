#ifndef PROJECT11_NAVIGATION_UTILITIES_H
#define PROJECT11_NAVIGATION_UTILITIES_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>

namespace project11_navigation
{

// Read a number from the parameter server as either a double or int.
// This avoids crashes when trying to read a paramter without a decimal point as a double.
double readDoubleOrIntParameter(ros::NodeHandle &nh, const std::string& parameter, double default_value);

// Read from the paramter server the linear and angular components of a type.
template<typename T>
void readLinearAngularParameters(ros::NodeHandle &nh, const std::string& parameter, T& value, const T& default_value)
{
  value.linear.x = readDoubleOrIntParameter(nh, parameter+"/linear/x", default_value.linear.x);
  value.linear.y = readDoubleOrIntParameter(nh, parameter+"/linear/y", default_value.linear.y);
  value.linear.z = readDoubleOrIntParameter(nh, parameter+"/linear/z", default_value.linear.z);
  value.angular.x = readDoubleOrIntParameter(nh, parameter+"/angular/x", default_value.angular.x);
  value.angular.y = readDoubleOrIntParameter(nh, parameter+"/angular/y", default_value.angular.y);
  value.angular.z = readDoubleOrIntParameter(nh, parameter+"/angular/z", default_value.angular.z);
}

// Adjust the timestamps along a trajectory for the given constant speed and starting
// at the first pose's timestamp.
void adjustTrajectoryForSpeed(std::vector<geometry_msgs::PoseStamped>& trajectory, double speed);

geometry_msgs::Vector3 vectorBetween(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to);
double length(const geometry_msgs::Vector3& vector);
geometry_msgs::Vector3 normalize(const geometry_msgs::Vector3& vector);

} // namespace project11_navigation

#endif

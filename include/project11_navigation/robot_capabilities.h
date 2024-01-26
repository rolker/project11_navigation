#ifndef PROJECT11_NAVIGATION_ROBOT_CAPABILITIES_H
#define PROJECT11_NAVIGATION_ROBOT_CAPABILITIES_H

#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>

namespace project11_navigation
{

/// Motion limits of a robot
struct RobotCapabilities
{
  RobotCapabilities(ros::NodeHandle& nh);

  geometry_msgs::Twist min_velocity;
  geometry_msgs::Twist max_velocity;

  /// default cruise speed
  geometry_msgs::Twist default_velocity;

  /// Max available acceleration
  geometry_msgs::Accel max_acceleration;

  /// Default acceleration that should be used
  /// for normal operations
  geometry_msgs::Accel default_acceleration;

  /// Powered deceleration
  geometry_msgs::Accel max_deceleration;

  /// Drifting deceleration
  geometry_msgs::Accel default_deceleration;

  /// Map speed to turn radius for a Dubin's robot.
  /// If empty, robot can turn in place so radius is 0.
  /// With only one entry, turn radius is constant in speed range.
  /// Speeds at which the robot can't run can be indicated with a NaN.
  std::map<double, double> turn_radius_map;
  double getTurnRadiusAtSpeed(double speed) const;

  /// Outline in XY plane of the robot
  //std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Polygon footprint;

  /// Radius used for collision checking
  float radius;
};

}

#endif

#ifndef PROJECT11_NAVIGATION_ROBOT_H
#define PROJECT11_NAVIGATION_ROBOT_H

#include <project11_navigation/platform.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Polygon.h>

namespace project11_navigation
{
class RobotCapabilities;

// Represents a platform that can be commanded using ROS Twist messages.
class Robot: public Platform
{
public:
  Robot();

  /// Sends command to robot if enabled.
  void sendControls(const geometry_msgs::TwistStamped& cmd_vel) const;

  /// Draws robot footprint using visualization markers
  ///  \todo move to a BT Action with access to footprint
  void updateMarkers(visualization_msgs::MarkerArray& marker_array, const geometry_msgs::Polygon& footprint) const;

  
  /// Returns true if robot can accept drive commands.
  bool enabled() const;

private:
  /// Velocity commands to move the robot.
  /// The output of the Navigator.
  ros::Publisher cmd_vel_pub_;

  /// Controls the output of drive commands.
  /// If true, we are in an autonomous mode where drive commands should be issued.
  /// If false, we are in a mode where drive commands should not be sent, but
  /// planning should occur and results made available for display in a UI to give 
  /// idea of what might happen when switching to an autonomous mode.
  bool enabled_ = false;
  ros::Subscriber enable_sub_;
  void enableCallback(const std_msgs::BoolConstPtr& msg);
};

} // namespace project11_navigation

#endif

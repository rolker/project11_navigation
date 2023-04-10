#ifndef PROJECT11_NAVIGATION_ROBOT_H
#define PROJECT11_NAVIGATION_ROBOT_H

#include <project11_navigation/platform.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/context.h>
#include <visualization_msgs/MarkerArray.h>

namespace project11_navigation
{

// Represents a platform that can be commanded using ROS Twist messages.
class Robot: public Platform
{
public:
  Robot(Context::Ptr context);

  void sendControls(const geometry_msgs::TwistStamped& cmd_vel);

  void updateMarkers(visualization_msgs::MarkerArray& marker_array);


protected:
  void odometryUpdated() override;

private:
  RobotCapabilities capabilities_;

  /// Velocity commands to move the robot.
  /// The output of the Navigator.
  ros::Publisher cmd_vel_pub_;

  /// Controls the output of drive commands.
  /// If true, we are in an autonomous mode where drive commands should be issued.
  /// If false, we are in a mode where drive commands should not be sent, but
  /// planning should occure and results made avaiable for display in a UI to give 
  /// idea of what might happen when switching to an autonomous mode.
  bool enabled_ = true;
  ros::Subscriber enable_sub_;
  void enableCallback(const std_msgs::BoolConstPtr& msg);

  Context::Ptr context_;
};

} // namespace project11_navigation

#endif

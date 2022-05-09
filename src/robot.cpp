#include <project11_navigation/robot.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

Robot::Robot(Context::Ptr context)
  :context_(context)
{
  ros::NodeHandle nh("~");
  cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

  enable_sub_ = nh.subscribe<std_msgs::Bool>("enable", 10, &Robot::enableCallback, this);

  if(nh.hasParam("turn_radius"))
    nh.getParam("turn_radius", capabilities_.turn_radius_map[0.0]);

  readLinearAngularParameters(nh, "robot/max_velocity", capabilities_.max_velocity, capabilities_.max_velocity);
  readLinearAngularParameters(nh, "robot/min_velocity", capabilities_.min_velocity, capabilities_.min_velocity);
  readLinearAngularParameters(nh, "robot/default_velocity", capabilities_.default_velocity, capabilities_.default_velocity);

  readLinearAngularParameters(nh, "robot/max_acceleration", capabilities_.max_acceleration, capabilities_.max_acceleration);
  readLinearAngularParameters(nh, "robot/max_deceleration", capabilities_.max_deceleration, capabilities_.max_deceleration);

  context_->updateRobotCapabilities(capabilities_);
}

void Robot::sendControls(const geometry_msgs::TwistStamped& cmd_vel)
{
  cmd_vel_pub_.publish(cmd_vel);
}

void Robot::enableCallback(const std_msgs::BoolConstPtr& msg)
{
  enabled_ = msg->data;
  context_->updateOutputEnabled(enabled_);
}

void Robot::odometryUpdated()
{
  context_->updateOdometry(odom_);
}

} // namespace project11_navigation

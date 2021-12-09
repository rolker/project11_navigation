#include <project11_navigation/context.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace project11_navigation
{

Context::Context():
  tf_listener_(tf_buffer_)
{
  
}

RobotCapabilities Context::getRobotCapabilities()
{
  std::lock_guard<std::mutex> lock(robot_capabilities_mutex_);
  return robot_capabilities_;
}

nav_msgs::Odometry Context::getOdometry()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return odom_;
}

bool Context::getOutputEnabled()
{
  std::lock_guard<std::mutex> lock(output_enabled_mutex_);
  return output_enabled_;
}

void Context::updateRobotCapabilities(const RobotCapabilities& robot_capabilities)
{
  std::lock_guard<std::mutex> lock(robot_capabilities_mutex_);
  robot_capabilities_ = robot_capabilities;
}

void Context::updateOdometry(const nav_msgs::Odometry& odom)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_ = odom;
}

void Context::updateOutputEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(output_enabled_mutex_);
  output_enabled_ = enabled;
}

tf2_ros::Buffer& Context::tfBuffer()
{
  return tf_buffer_;
}

geometry_msgs::PoseStamped Context::getPoseInFrame(std::string frame_id)
{
  auto odom = getOdometry();
  geometry_msgs::PoseStamped ret;
  ret.header = odom.header;
  ret.pose = odom.pose.pose;
  try
  {
    if(ret.header.frame_id != frame_id)
    {
      tf_buffer_.transform(ret, frame_id, ros::Duration(0.25));
      ret.header.frame_id = frame_id;
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Context::getPoseInFrame " << ex.what());
  }

  return ret;
}

} // namespace project11_navigation
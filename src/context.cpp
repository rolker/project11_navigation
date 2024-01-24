#include <project11_navigation/context.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace project11_navigation
{

Context::Context():
  tf_buffer_(new tf2_ros::Buffer), tf_listener_(*tf_buffer_)
{
 
}

const Environment& Context::environment() const
{
  return environment_;
}

const Robot& Context::robot() const
{
  return robot_;
}

Robot& Context::robot()
{
  return robot_;
}


std::shared_ptr<tf2_ros::Buffer> Context::tfBuffer() const
{
  return tf_buffer_;
}

geometry_msgs::PoseStamped Context::getPoseInFrame(std::string frame_id)
{
  auto odom = robot_.odometry();
  geometry_msgs::PoseStamped ret;
  ret.header = odom.header;
  ret.pose = odom.pose.pose;
  try
  {
    if(ret.header.frame_id != frame_id)
    {
      tf_buffer_->transform(ret, frame_id, ros::Duration(0.25));
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

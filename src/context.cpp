#include <project11_navigation/context.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <project11_navigation/plugins_loader.h>
#include <project11_navigation/task_plugins.h>

#include <project11_navigation/tasks/hover.h>
#include <project11_navigation/tasks/survey_area.h>
#include <project11_navigation/tasks/survey_line.h>
#include <project11_navigation/tasks/transit.h>

namespace project11_navigation
{

Context::Context(NavigatorSettings nav_settings):
  navigator_settings_(nav_settings), tf_listener_(tf_buffer_),
  plugins_loader_(new PluginsLoader()), task_plugins_(new TaskPlugins())
{
 
}

const NavigatorSettings& Context::navigatorSettings() const
{
  return navigator_settings_;
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

Environment& Context::environment()
{
  return environment_;
}

bool Context::getOutputEnabled()
{
  std::lock_guard<std::mutex> lock(output_enabled_mutex_);
  return output_enabled_;
}

std::shared_ptr<PluginsLoader> Context::pluginsLoader()
{
  return plugins_loader_;
}

std::shared_ptr<TaskPlugins> Context::taskPlugins()
{
  return task_plugins_;
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

void Context::setCurrentNavTaskID(std::string id)
{
  current_nav_task_id_ = id;
}

const std::string & Context::currentNavTaskID() const
{
  return current_nav_task_id_;
}

} // namespace project11_navigation

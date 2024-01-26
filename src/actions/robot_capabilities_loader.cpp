#include "project11_navigation/actions/robot_capabilities_loader.h"
#include <ros/ros.h>
#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

RobotCapabilitiesLoader::RobotCapabilitiesLoader(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList RobotCapabilitiesLoader::providedPorts()
{
  return {
    BT::OutputPort<double>("turn_radius"),
    BT::OutputPort<geometry_msgs::Twist>("maximum_velocity"),
    BT::OutputPort<geometry_msgs::Twist>("minimum_velocity"),
    BT::OutputPort<geometry_msgs::Twist>("default_velocity"),
    BT::OutputPort<geometry_msgs::Accel>("maximum_acceleration"),
    BT::OutputPort<geometry_msgs::Accel>("default_acceleration"),
    BT::OutputPort<geometry_msgs::Accel>("maximum_deceleration"),
    BT::OutputPort<geometry_msgs::Accel>("default_deceleration"),
    BT::OutputPort<geometry_msgs::Polygon>("footprint"),
    BT::OutputPort<double>("radius"),

    BT::OutputPort<double>("default_speed"),
    BT::OutputPort<double>("stopping_time"),
    BT::OutputPort<double>("stopping_distance"),
  };
}

BT::NodeStatus RobotCapabilitiesLoader::tick()
{
  ros::NodeHandle private_nh("~");
  RobotCapabilities rc(private_nh);
  setOutput("turn_radius", rc.getTurnRadiusAtSpeed(rc.default_velocity.linear.x));
  setOutput("maximum_velocity", rc.max_velocity);
  setOutput("minimum_velocity", rc.min_velocity);
  setOutput("default_velocity", rc.default_velocity);
  setOutput("maximum_acceleration", rc.max_acceleration);
  setOutput("default_acceleration", rc.default_acceleration);
  setOutput("maximum_deceleration", rc.max_deceleration);
  setOutput("default_deceleration", rc.default_deceleration);
  setOutput("footprint", rc.footprint);
  setOutput("radius", rc.radius);

  setOutput("default_speed", rc.default_velocity.linear.x);

  // how long to stop from max speed
  if(rc.default_deceleration.linear.x < 0.0)
  {
    auto stopping_time = -rc.max_velocity.linear.x/rc.default_deceleration.linear.x;
    auto stopping_distance = stopping_time*rc.max_velocity.linear.x/2.0;

    setOutput("stopping_time", stopping_time);
    setOutput("stopping_distance", stopping_distance);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

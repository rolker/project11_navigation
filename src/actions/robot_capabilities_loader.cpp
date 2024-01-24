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
    BT::OutputPort<std::shared_ptr<RobotCapabilities> >("robot_capabilities"),
    BT::OutputPort<double>("default_target_speed"),
  };
}

BT::NodeStatus RobotCapabilitiesLoader::tick()
{
  ros::NodeHandle private_nh("~");
  auto rc = std::make_shared<RobotCapabilities>(private_nh);
  setOutput("robot_capabilities", rc);
  setOutput("default_target_speed", rc->default_velocity.linear.x);
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

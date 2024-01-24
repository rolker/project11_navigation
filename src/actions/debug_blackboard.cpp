#include "project11_navigation/actions/debug_blackboard.h"
#include <ros/ros.h>

namespace project11_navigation
{

DebugBlackboard::DebugBlackboard(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList DebugBlackboard::providedPorts()
{
  return {BT::InputPort<std::string>("value")};
}

BT::NodeStatus DebugBlackboard::tick()
{

  BT::Expected<std::string> value = getInput<std::string>("value");
  if(!value)
  {
    ROS_WARN_STREAM("BT Node: " << name() <<  " missing input [value]");
  }
  else
    ROS_INFO_STREAM("BT Node: " << name() << " value: " << value.value());
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

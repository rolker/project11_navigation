#ifndef PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H
#define PROJECT11_NAVIGATION_ACTIONS_DEBUG_BLACKBOARD_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class DebugBlackboard: public BT::SyncActionNode
{
public:
  DebugBlackboard(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace project11_navigation

#endif

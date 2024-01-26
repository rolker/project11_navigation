#ifndef PROJECT11_NAVIGATION_ACTIONS_HOVER_H
#define PROJECT11_NAVIGATION_ACTIONS_HOVER_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class Hover: public BT::SyncActionNode
{
public:
  Hover(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


} // namespace project11_navigation

#endif

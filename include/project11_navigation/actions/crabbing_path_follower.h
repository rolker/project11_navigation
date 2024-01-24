#ifndef PROJECT11_NAVIGATION_ACTIONS_CRABBING_PATH_FOLLOWER_H
#define PROJECT11_NAVIGATION_ACTIONS_CRABBING_PATH_FOLLOWER_H

#include <behaviortree_cpp/bt_factory.h>

namespace project11_navigation
{

class CrabbingPathFollower: public BT::StatefulActionNode
{
public:
  CrabbingPathFollower(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

} // namespace path_follower

#endif

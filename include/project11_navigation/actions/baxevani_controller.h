#ifndef PROJECT11_NAVIGATION_BAXEVANI_CONTROLLER_H
#define PROJECT11_NAVIGATION_BAXEVANI_CONTROLLER_H

#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/Odometry.h>

namespace project11_navigation
{

class BaxevaniController: public BT::StatefulActionNode
{
public:
  BaxevaniController(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  nav_msgs::Odometry last_odom_;
};

} // namespace path_follower

#endif

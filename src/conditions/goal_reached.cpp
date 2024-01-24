#include <project11_navigation/conditions/goal_reached.h>
#include <nav_msgs/Odometry.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

GoalReached::GoalReached(const std::string& name, const BT::NodeConfig& config):
  BT::ConditionNode(name, config)
{

}

BT::PortsList GoalReached::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::Odometry>("odometry"),
    BT::InputPort<geometry_msgs::PoseStamped>("goal_pose"),
    BT::InputPort<double>("waypoint_reached_distance"),
    BT::InputPort<double>("heading_accuracy")
  };
}

BT::NodeStatus GoalReached::tick()
{
  auto odom = getInput<nav_msgs::Odometry>("odometry");
  auto goal = getInput<geometry_msgs::PoseStamped>("goal_pose");
  auto distance = getInput<double>("waypoint_reached_distance");
  if(odom && goal && distance)
  {
    if(length(vectorBetween(odom.value().pose.pose, goal.value().pose)) <= distance.value())
      return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace project11_navigation

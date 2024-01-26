#include <project11_navigation/actions/predict_stopping_pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

namespace project11_navigation
{

PredictStoppingPose::PredictStoppingPose(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList PredictStoppingPose::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::Odometry>("odometry"),
    BT::InputPort<geometry_msgs::Accel>("deceleration"),
    BT::OutputPort<geometry_msgs::PoseStamped>("pose")
  };
}

BT::NodeStatus PredictStoppingPose::tick()
{
  auto odom = getInput<nav_msgs::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError(name(), " missing required input [odometry]: ", odom.error() );
  }

  auto decel = getInput<geometry_msgs::Accel>("deceleration");
  if(!decel)
  {
    throw BT::RuntimeError(name(), " missing required input [deceleration]: ", decel.error() );
  }


  tf2::Vector3 motion_vector;
  tf2::fromMsg(odom.value().twist.twist.linear, motion_vector);
  tf2::Quaternion orientation;
  tf2::fromMsg(odom.value().pose.pose.orientation, orientation);
  motion_vector = tf2::quatRotate(orientation, motion_vector);

  double stop_time = -motion_vector.length()/decel.value().linear.x;
  motion_vector *= (stop_time/2.0);

  geometry_msgs::PoseStamped pose;
  pose.header = odom.value().header;
  pose.pose = odom.value().pose.pose;
  pose.pose.position.x += motion_vector.getX();
  pose.pose.position.y += motion_vector.getY();
  pose.pose.position.z += motion_vector.getZ();

  setOutput("pose", pose);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

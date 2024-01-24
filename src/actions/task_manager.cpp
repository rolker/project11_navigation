#include <project11_navigation/actions/task_manager.h>
#include <project11_navigation/robot.h>

namespace project11_navigation
{

TaskManagerAction::TaskManagerAction(const std::string& name, const BT::NodeConfig& config, boost::shared_ptr<TaskListToTwistWorkflow> task_manager, std::shared_ptr<Robot> robot):
  BT::SyncActionNode(name, config), task_manager_(task_manager), robot_(robot)
{

}

BT::PortsList TaskManagerAction::providedPorts()
{
  return {BT::InputPort<std::string>("base_frame")};
}

BT::NodeStatus TaskManagerAction::tick()
{
  if(!task_manager_->running())
    return BT::NodeStatus::FAILURE;
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = ros::Time::now();

  BT::Expected<std::string> frame_id = getInput<std::string>("base_frame");
  if(!frame_id)
  {
    ROS_WARN_STREAM_THROTTLE(10.0, "missing input [base_frame]");
  }
  else
  {
    cmd_vel.header.frame_id = frame_id.value();
    if(cmd_vel.header.frame_id.empty())
    {
      ROS_WARN_STREAM_THROTTLE(10.0, "Waiting for odom with non-empty child_frame_id");
    }
    else
    {
      task_manager_->getResult(cmd_vel);
      robot_->sendControls(cmd_vel);
    }
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

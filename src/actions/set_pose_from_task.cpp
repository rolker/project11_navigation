#include <project11_navigation/actions/set_pose_from_task.h>
#include <geometry_msgs/PoseStamped.h>
#include <project11_navigation/task.h>
#include <ros/ros.h>

namespace project11_navigation
{

SetPoseFromTask::SetPoseFromTask(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList SetPoseFromTask::providedPorts()
{
  return {
    BT::InputPort<TaskPtr>("task"),
    BT::InputPort<int>("pose_index"),
    BT::OutputPort<geometry_msgs::PoseStamped>("pose")
  };
}

BT::NodeStatus SetPoseFromTask::tick()
{
  auto task = getInput<TaskPtr>("task");
  if(!task)
  {
    throw BT::RuntimeError("missing required input [task]: ", task.error() );
  }

  auto pose_index = getInput<int>("pose_index");
  if(!pose_index)
  {
    throw BT::RuntimeError("missing required input [pose_index]: ", pose_index.error() );
  }

  int index = pose_index.value();
  if(index < 0) // python style count from the end
    index = task.value()->message().poses.size()+index;
  if(index < 0 || index >= task.value()->message().poses.size())
  {
    ROS_WARN_STREAM("SetPoseFromTask node named " << name() << " index " << pose_index.value() << " out of range for task with " << task.value()->message().poses.size() << " poses");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("pose", task.value()->message().poses[pose_index.value()]);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

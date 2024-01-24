#ifndef PROJECT11_NAVIGATION_ACTIONS_TASK_MANAGER_H
#define PROJECT11_NAVIGATION_ACTIONS_TASK_MANAGER_H

#include <behaviortree_cpp/bt_factory.h>
#include <project11_navigation/workflows/task_manager.h>

namespace project11_navigation
{

class TaskManagerAction: public BT::SyncActionNode
{
public:
  TaskManagerAction(const std::string& name, const BT::NodeConfig& config, boost::shared_ptr<TaskListToTwistWorkflow> task_manager, std::shared_ptr<Robot> robot);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  boost::shared_ptr<TaskListToTwistWorkflow> task_manager_;
  std::shared_ptr<Robot> robot_;
};

} // namespace project11_navigation

#endif

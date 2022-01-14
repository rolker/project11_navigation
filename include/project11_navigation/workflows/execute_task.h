#ifndef PROJECT11_NAVIGATION_WORKFLOWS_EXECUTE_TASK_H
#define PROJECT11_NAVIGATION_WORKFLOWS_EXECUTE_TASK_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <geometry_msgs/TwistStamped.h>
#include <project11_navigation/interfaces/task_to_twist_workflow.h>
#include <pluginlib/class_loader.h>

namespace project11_navigation
{

// Given a Task, forwards it to the apporiate workflow.
class ExecuteTask: public TaskToTwistWorkflow
{
public:
  ExecuteTask();
  ~ExecuteTask();

  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const std::shared_ptr<Task>& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;
private:
  bool updateCurrentHandler();

  Context::Ptr context_;
  std::map<std::string, boost::shared_ptr<TaskToTwistWorkflow> > task_handlers_;
  std::shared_ptr<Task> current_task_;
  std::shared_ptr<Task> current_nav_task_;
  boost::shared_ptr<TaskToTwistWorkflow> current_handler_;

  pluginlib::ClassLoader<TaskToTwistWorkflow> task_handler_loader_;

};

}  // namespace project11_navigation


#endif

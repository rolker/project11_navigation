#ifndef PROJECT11_NAVIGATION_WORKFLOWS_EXECUTE_TASK_H
#define PROJECT11_NAVIGATION_WORKFLOWS_EXECUTE_TASK_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <geometry_msgs/TwistStamped.h>
#include <project11_navigation/interfaces/task_to_twist_workflow.h>

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
  
  // Maps task types to plugins. If a task type is
  // not listed, an attempt to use a TaskToTwistWorkflow
  // plugin with the task type name will be made.
  // That means an entry is not needed here if the 
  // plugin to handle a task has the same name as
  // the task type.
  std::map<std::string, std::string> task_handlers_;
  
  std::shared_ptr<Task> current_task_;
  std::shared_ptr<Task> current_nav_task_;
  boost::shared_ptr<TaskToTwistWorkflow> current_handler_;
};

}  // namespace project11_navigation


#endif

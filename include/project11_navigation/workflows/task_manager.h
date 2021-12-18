#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASK_MANAGER_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASK_MANAGER_H

#include <project11_navigation/interfaces/tasklist_to_twist_workflow.h>
#include <project11_navigation/interfaces/tasklist_to_tasklist_workflow.h>
#include <project11_navigation/workflows/execute_task.h>
#include <project11_navigation/task.h>

namespace project11_navigation
{

class TaskManager: public TaskListToTwistWorkflow
{
public:
  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const std::shared_ptr<TaskList>& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;

private:
  void updateCurrentTask();

  Context::Ptr context_;

  std::shared_ptr<TaskListToTaskListWorkflow> task_connector_;

  std::shared_ptr<TaskToTwistWorkflow> executive_;

  std::shared_ptr<TaskList> task_list_;
  std::shared_ptr<Task> current_task_;

};

}  // namespace project11_navigation


#endif

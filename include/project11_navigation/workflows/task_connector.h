#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASK_CONNECTOR_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASK_CONNECTOR_H

#include <project11_navigation/interfaces/tasklist_to_tasklist_workflow.h>

namespace project11_navigation
{

class TaskConnector: public TaskListToTaskListWorkflow
{
public:
  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const std::shared_ptr<TaskList>& input) override;
  bool running() override;
  bool getResult(std::shared_ptr<TaskList>& output) override;

private:
  bool connectTasks();

  Context::Ptr context_;

  std::shared_ptr<TaskList> task_list_;
  bool connected_ = false;
};

}  // namespace project11_navigation


#endif

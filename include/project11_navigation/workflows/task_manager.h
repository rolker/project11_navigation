#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASK_MANAGER_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASK_MANAGER_H

#include <project11_navigation/interfaces/tasklist_to_twist_workflow.h>
#include <project11_navigation/interfaces/tasklist_to_tasklist_workflow.h>
#include <project11_navigation/interfaces/task_to_twist_workflow.h>
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

  boost::shared_ptr<TaskListToTaskListWorkflow> task_connector_;

  boost::shared_ptr<TaskToTwistWorkflow> executive_;

  std::shared_ptr<TaskList> task_list_;
  boost::shared_ptr<Task> current_task_;

  ros::Publisher display_pub_;
  ros::Duration display_interval_;
  ros::Time last_display_time_;

};

}  // namespace project11_navigation


#endif

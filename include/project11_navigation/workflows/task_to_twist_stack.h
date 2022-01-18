#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASK_TO_TWIST_STACK_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASK_TO_TWIST_STACK_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <geometry_msgs/TwistStamped.h>
#include <project11_navigation/interfaces/task_to_twist_workflow.h>
#include <project11_navigation/interfaces/task_to_task_workflow.h>

namespace project11_navigation
{

// Allow one or more plugins to handle a task and
// produce a twist output.
class TaskToTwistStack: public TaskToTwistWorkflow
{
public:

  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const std::shared_ptr<Task>& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;
private:
  Context::Ptr context_;
  std::shared_ptr<Task> current_task_;

  std::vector<boost::shared_ptr<TaskToTaskWorkflow> > steps_;
  boost::shared_ptr<TaskToTwistWorkflow> last_step_;
};

}  // namespace project11_navigation


#endif

#ifndef PROJECT11_NAVIGATION_WORKFLOWS_IDLE_H
#define PROJECT11_NAVIGATION_WORKFLOWS_IDLE_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <geometry_msgs/TwistStamped.h>
#include <project11_navigation/interfaces/task_to_twist_workflow.h>

namespace project11_navigation
{

// Given a Task, forwards it to the apporiate workflow.
class Idle: public TaskToTwistWorkflow
{
public:
  Idle();
  ~Idle();

  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const boost::shared_ptr<Task>& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;

};

}  // namespace project11_navigation


#endif

#ifndef PROJECT11_NAVIGATION_INTERFACES_TASK_WRAPPER_H
#define PROJECT11_NAVIGATION_INTERFACES_TASK_WRAPPER_H

#include <project11_navigation/task.h>

namespace project11_navigation
{
class Context;

class TaskWrapper
{
public:

  virtual bool needsTransit(const geometry_msgs::PoseStamped& from, geometry_msgs::PoseStamped& to)=0;
  virtual geometry_msgs::PoseStamped expectedEndPose(const geometry_msgs::PoseStamped& starting_pose)=0;
  virtual void updateTransit(const geometry_msgs::PoseStamped& starting_pose, geometry_msgs::PoseStamped& out_pose)=0;
  virtual std::shared_ptr<Task> getCurrentNavigationTask()=0;

  // Called once to initiate the wrapper. Name may be used for parameter server lookups.
  virtual void configure(std::string name, std::shared_ptr<Context> context) = 0;
protected:
  std::shared_ptr<Task> task_;
  Context* context_;
private:
  friend class Context;
};

} // namespace project11_navigation

#endif

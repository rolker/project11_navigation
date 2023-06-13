#ifndef PROJECT11_NAVIGATION_INTERFACES_TASK_WRAPPER_H
#define PROJECT11_NAVIGATION_INTERFACES_TASK_WRAPPER_H

#error

#include <project11_navigation/task.h>
#include <visualization_msgs/MarkerArray.h>

namespace project11_navigation
{
class Context;

class TaskWrapper
{
public:
  virtual void updateTransit(const geometry_msgs::PoseStamped& starting_pose, geometry_msgs::PoseStamped& out_pose)=0;
  virtual boost::shared_ptr<Task> getCurrentNavigationTask()=0;

  // Called once to initiate the wrapper. Name may be used for parameter server lookups.
  virtual void configure(std::string name, std::shared_ptr<Context> context) = 0;

  // Allows visualization markers to be added to preview task navigation.
  virtual void getPreviewDisplay(visualization_msgs::MarkerArray& marker_array) = 0;

protected:
  std::shared_ptr<Task> task_;
  Context* context_;
private:
  friend class Context;
};

} // namespace project11_navigation

#endif

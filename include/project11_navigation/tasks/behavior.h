#ifndef PROJECT11_NAVIGATION_TASKS_BEHAVIOR_H
#define PROJECT11_NAVIGATION_TASKS_BEHAVIOR_H

#include <project11_navigation/task.h>

namespace project11_navigation
{

class BehaviorTask: public Task
{
public:
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context) override;
  boost::shared_ptr<Task> getCurrentNavigationTask() override;
};

} // namespace project11_navigation

#endif

#ifndef PROJECT11_NAVIGATION_TASKS_HOVER_H
#define PROJECT11_NAVIGATION_TASKS_HOVER_H

#include <project11_navigation/task.h>

namespace project11_navigation
{

class HoverTask: public Task
{
public:
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context) override;
  boost::shared_ptr<Task> getCurrentNavigationTask() override;
  void getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const override;
};

} // namespace project11_navigation

#endif

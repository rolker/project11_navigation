#ifndef PROJECT11_NAVIGATION_TASKS_TRANSIT_H
#define PROJECT11_NAVIGATION_TASKS_TRANSIT_H

#include <project11_navigation/interfaces/task_wrapper.h>

namespace project11_navigation
{

class TransitTask: public TaskWrapper
{
public:
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose) override;
  std::shared_ptr<Task> getCurrentNavigationTask() override;
  void configure(std::string name, std::shared_ptr<Context> context) override;
  void getPreviewDisplay(visualization_msgs::MarkerArray& marker_array) override;
};

} // namespace project11_navigation

#endif

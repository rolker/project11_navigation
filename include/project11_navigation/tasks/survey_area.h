#ifndef PROJECT11_NAVIGATION_TASKS_SURVEY_AREA_H
#define PROJECT11_NAVIGATION_TASKS_SURVEY_AREA_H

#include <project11_navigation/interfaces/task_wrapper.h>

namespace project11_navigation
{

class SurveyAreaTask: public TaskWrapper
{
public:
  bool needsTransit(const geometry_msgs::PoseStamped& from, geometry_msgs::PoseStamped& to) override;
  geometry_msgs::PoseStamped expectedEndPose(const geometry_msgs::PoseStamped& starting_pose) override;
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose) override;
  std::shared_ptr<Task> getCurrentNavigationTask() override;
};

} // namespace project11_navigation

#endif

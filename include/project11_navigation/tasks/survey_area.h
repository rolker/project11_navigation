#ifndef PROJECT11_NAVIGATION_TASKS_SURVEY_AREA_H
#define PROJECT11_NAVIGATION_TASKS_SURVEY_AREA_H

#include <project11_navigation/interfaces/task_wrapper.h>

namespace project11_navigation
{

class SurveyAreaTask: public TaskWrapper
{
public:
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose) override;
  std::shared_ptr<Task> getCurrentNavigationTask() override;
  void configure(std::string name, std::shared_ptr<Context> context) override;
  void getPreviewDisplay(visualization_msgs::MarkerArray& marker_array) override;
};

} // namespace project11_navigation

#endif

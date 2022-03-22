#include <project11_navigation/tasks/survey_area.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::SurveyAreaTask, project11_navigation::TaskWrapper)

namespace project11_navigation
{

void SurveyAreaTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  geometry_msgs::PoseStamped pose = from_pose;
  bool has_survey_lines = false;
  for(auto t: task_->children().tasksByPriority())
    if(t->message().type == "survey_line")
    {
      auto slw = context_->getTaskWrapper(t);
      slw->updateTransit(pose, out_pose);
      pose = out_pose;
      has_survey_lines = true;
    }

  if(has_survey_lines)
    return;

  geometry_msgs::PoseStamped in_pose;
  if(task_->getFirstPose(in_pose))
  {
    if(length(vectorBetween(from_pose.pose, in_pose.pose))>10.0)
      task_->updateTransitTo(from_pose, in_pose);
  }
  else 
    in_pose = from_pose;

  if(!task_->getLastPose(out_pose))
    out_pose = in_pose;
}

std::shared_ptr<Task> SurveyAreaTask::getCurrentNavigationTask()
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
    if(t->message().type == "survey_line")
    {
      auto slw = context_->getTaskWrapper(t);
      return slw->getCurrentNavigationTask();
    }
    auto status = t->status();
    status["skipped"] = "Skipped by SurveyAreaTask";
    t->setStatus(status);
    t->setDone();
  }
  return task_;
}

void SurveyAreaTask::configure(std::string name, std::shared_ptr<Context> context)
{
  
}

void SurveyAreaTask::getPreviewDisplay(visualization_msgs::MarkerArray& marker_array)
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    auto tw = context_->getTaskWrapper(t);
    if(tw)
      tw->getPreviewDisplay(marker_array);
  }
}

} // namespace project11_navigation

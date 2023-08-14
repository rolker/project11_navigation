#include <project11_navigation/tasks/survey_area.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::SurveyAreaTask, project11_navigation::Task)

namespace project11_navigation
{

void SurveyAreaTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  geometry_msgs::PoseStamped pose = from_pose;
  bool has_survey_lines = false;
  for(auto t: children().tasksByPriority())
    if(t->message().type == "survey_line")
    {
      t->updateTransit(pose, out_pose, context);
      pose = out_pose;
      has_survey_lines = true;
    }

  if(has_survey_lines)
    return;

  geometry_msgs::PoseStamped in_pose;
  if(getFirstPose(in_pose))
  {
    if(length(vectorBetween(from_pose.pose, in_pose.pose)) > context->getRobotCapabilities().waypoint_reached_distance)
      updateTransitTo(from_pose, in_pose);
  }
  else 
    in_pose = from_pose;

  if(!getLastPose(out_pose))
    out_pose = in_pose;
}

boost::shared_ptr<Task> SurveyAreaTask::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
    if(t->message().type == "survey_line")
      return t->getCurrentNavigationTask();
    auto status = t->status();
    status["skipped"] = "Skipped by SurveyAreaTask";
    t->setStatus(status);
    t->setDone();
  }
  return self();
}


} // namespace project11_navigation

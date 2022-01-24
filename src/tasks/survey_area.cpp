#include <project11_navigation/tasks/survey_area.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>

namespace project11_navigation
{

bool SurveyAreaTask::needsTransit(const geometry_msgs::PoseStamped& from, geometry_msgs::PoseStamped& to)
{
  auto survey_line = task_->getFirstChildOfType("survey_line");
  if(survey_line)
  {
    auto slw = context_->getTaskWrapper(survey_line);
    if(slw)
      return slw->needsTransit(from, to);
  }


  if(!task_->message().poses.empty())
  {
    geometry_msgs::PoseStamped start_position = task_->message().poses[0];
    if(length(vectorBetween(from.pose, start_position.pose))>10.0)
    {
      to = start_position;
      return true;
    }
  }
  return false;
}

geometry_msgs::PoseStamped SurveyAreaTask::expectedEndPose(const geometry_msgs::PoseStamped& starting_pose)
{
  auto last_line = task_->getLastChildOfType("survey_line");
  if(last_line)
    if(!last_line->message().poses.empty())
      return last_line->message().poses.back();
  return starting_pose;
}

void SurveyAreaTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  auto survey_line = task_->getFirstChildOfType("survey_line");
  if(survey_line)
  {
    while(survey_line)
    {
      auto slw = context_->getTaskWrapper(survey_line);
      slw->updateTransit(from_pose, out_pose);
      survey_line = task_->getNextChildOfType(survey_line);
    }
    return;
  }

  geometry_msgs::PoseStamped in_pose;
  if(task_->getFirstPose(in_pose))
  {
    if(length(vectorBetween(from_pose.pose, in_pose.pose))>10.0)
      task_->updateTransitTo(in_pose);
  }
  else 
    in_pose = from_pose;

  if(!task_->getLastPose(out_pose))
    out_pose = in_pose;
}

std::shared_ptr<Task> SurveyAreaTask::getCurrentNavigationTask()
{
  auto transit_to = task_->getFirstChildOfTypeAndID("transit","transit_to");
  if(transit_to && !transit_to->done())
    return transit_to;

  auto next_task = task_->getFirstUndoneChildTask();
  while(next_task)
  {
    auto tw = context_->getTaskWrapper(next_task);
    if(tw)
    {
      auto nav_task = tw->getCurrentNavigationTask();
      if(nav_task)
        return nav_task;
    }
    next_task->setStatus("Skipped be SurveyAreaTask");
    next_task->setDone();
    next_task = task_->getFirstUndoneChildTask();
  }

  return task_;
}



} // namespace project11_navigation

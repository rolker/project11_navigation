#include <project11_navigation/tasks/survey_line.h>
#include <project11_navigation/utilities.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::SurveyLineTask, project11_navigation::TaskWrapper)

namespace project11_navigation
{

bool SurveyLineTask::needsTransit(const geometry_msgs::PoseStamped& from, geometry_msgs::PoseStamped& to)
{
  if(task_->message().poses.size() >= 2)
  {
    auto first_segment_direction_vector = normalize(vectorBetween(task_->message().poses[0].pose, task_->message().poses[1].pose));

    // TODO: get this from a parameter
    double lead_in_distance = 25.0;

    geometry_msgs::PoseStamped start_position = task_->message().poses[0];
    start_position.pose.position.x -= first_segment_direction_vector.x*lead_in_distance;
    start_position.pose.position.y -= first_segment_direction_vector.y*lead_in_distance;
    start_position.pose.position.z -= first_segment_direction_vector.z*lead_in_distance;

    if(length(vectorBetween(from.pose, start_position.pose))>10.0)
    {
      to = start_position;
      return true;
    }
  }
  return false;
}

geometry_msgs::PoseStamped SurveyLineTask::expectedEndPose(const geometry_msgs::PoseStamped& starting_pose)
{
  if(!task_->message().poses.empty())
    return task_->message().poses.back();
  return starting_pose;
}

void SurveyLineTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  if(task_->message().poses.size() >= 2)
  {
    auto first_segment_direction_vector = normalize(vectorBetween(task_->message().poses[0].pose, task_->message().poses[1].pose));

    // TODO: get this from a parameter
    double lead_in_distance = 25.0;

    geometry_msgs::PoseStamped in_pose = task_->message().poses[0];
    in_pose.pose.position.x -= first_segment_direction_vector.x*lead_in_distance;
    in_pose.pose.position.y -= first_segment_direction_vector.y*lead_in_distance;
    in_pose.pose.position.z -= first_segment_direction_vector.z*lead_in_distance;

    if(length(vectorBetween(from_pose.pose, in_pose.pose))>10.0)
      task_->updateTransitTo(in_pose);
  }
  else
  {
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
}

std::shared_ptr<Task> SurveyLineTask::getCurrentNavigationTask()
{
  auto transit_to = task_->getFirstChildOfTypeAndID("transit","transit_to");
  if(transit_to && !transit_to->done())
    return transit_to;
  return task_;
}

void SurveyLineTask::configure(std::string name, std::shared_ptr<Context> context)
{
  
}

} // namespace project11_navigation

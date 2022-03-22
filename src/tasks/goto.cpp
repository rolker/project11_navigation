#include <project11_navigation/tasks/goto.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::GotoTask, project11_navigation::TaskWrapper)

namespace project11_navigation
{

void GotoTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose)
{
  auto current_pose = from_pose;
  for(int i = 0; i < task_->message().poses.size(); i++)
  {
    auto next_pose = task_->message().poses[i];
    std::string task_id = "transit_to_wp"+std::to_string(i);
    std::shared_ptr<Task> transit;
    for(auto t: task_->children().tasks())
      if(t->message().type == "transit" && t->message().id == task_->getChildID(task_id))
      {
        transit = t;
        break;
      }
    if(!transit)
    {
      transit = task_->createChildTaskBefore(std::shared_ptr<Task>(), "transit");
      task_->setChildID(transit, task_id);
    }
    auto m = transit->message();
    m.poses.clear();
    m.poses.push_back(current_pose);
    m.poses.push_back(next_pose);
    transit->update(m);
    auto preview_planner = context_->pluginsLoader()->getPlugin<TaskToTaskWorkflow>("preview");
    if(preview_planner)
    {
      preview_planner->setGoal(transit);
      std::shared_ptr<Task> plan;
      preview_planner->getResult(plan);
    }
    current_pose = next_pose;
  }
  out_pose = current_pose;
}

std::shared_ptr<Task> GotoTask::getCurrentNavigationTask()
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return std::shared_ptr<Task>();
}

void GotoTask::configure(std::string name, std::shared_ptr<Context> context)
{
  
}

void GotoTask::getPreviewDisplay(visualization_msgs::MarkerArray& marker_array)
{
  for(auto t: task_->children().tasksByPriority(true))
  {
    auto tw = context_->getTaskWrapper(t);
    if(tw)
      tw->getPreviewDisplay(marker_array);
  }
}

} // namespace project11_navigation

#include "project11_navigation/task.h"
#include <project11_navigation/utilities.h>
#include <ros/ros.h>

#include <project11_navigation/context.h>
#include <project11_navigation/plugins_loader.h>
#include <project11_navigation/task_plugins.h>


namespace project11_navigation
{

std::shared_ptr<TaskPlugins> Task::creator_;

Task::Task():
  children_(this),
  last_update_time_(ros::Time::now())
{
}

void Task::setCreator(std::shared_ptr<TaskPlugins> creator)
{
  creator_ = creator;
}

boost::shared_ptr<Task> Task::create(const project11_nav_msgs::TaskInformation& task_msg, Task::Ptr parent_task)
{
  boost::shared_ptr<Task> new_task;

  if(creator_)
    new_task = (*creator_)(task_msg);

  if(!new_task)
    new_task = boost::make_shared<Task>();
  
  new_task->message_ = task_msg;
  new_task->self_ = new_task;
  new_task->parent_task_ = parent_task;
  return new_task;
}

boost::shared_ptr<Task> Task::self()
{
  return self_.lock();
}

bool Task::update(const project11_nav_msgs::TaskInformation& task_msg, bool check_id)
{
  if(!check_id || message_.id == task_msg.id)
  {
    if(message_ != task_msg)
      last_update_time_ = ros::Time::now();
    message_= task_msg;
    return true;
  }
  return false;
}

void Task::update(const std::vector<project11_nav_msgs::TaskInformation>& task_msgs)
{
  for(auto task_msg: task_msgs)
    update(task_msg);
  children_.update(task_msgs);
}

const project11_nav_msgs::TaskInformation & Task::message() const
{
  return message_;
}

void Task::setDone()
{
  message_.done = true;
  last_update_time_ = ros::Time::now();
}

YAML::Node Task::data() const
{
  return YAML::Load(message_.data);
}

void Task::setData(const YAML::Node& data)
{
  std::stringstream ss;
  ss << data;
  message_.data = ss.str();
  last_update_time_ = ros::Time::now();
}

YAML::Node Task::dataItem(std::string key, bool recurse_up) const
{
  auto d = data();
  if(d[key])
    return d[key];
  if(recurse_up && !parent_task_.empty())
    return parent_task_.lock()->dataItem(key, recurse_up);
  return {};
}

YAML::Node Task::status() const
{
  return YAML::Load(message_.status);
}

void Task::setStatus(const YAML::Node& status)
{
  std::stringstream ss;
  ss << status;
  message_.status = ss.str();
  last_update_time_ = ros::Time::now();
}

void Task::setID(std::string id)
{
  message_.id = id;
  last_update_time_ = ros::Time::now();
}

void Task::setChildID(boost::shared_ptr<Task> task, std::string id)
{
  task->setID(message_.id + "/" + id);
}

bool Task::done(bool recursive) const
{
  if(recursive)
    return message_.done && children_.allDone(true);
  return message_.done;
}

const TaskList& Task::children() const
{
  return children_;
}

ros::Time Task::lastUpdateTime() const
{
  return last_update_time_;  
}

bool Task::getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive) const
{
  if(!message_.poses.empty())
  {
    pose = message_.poses.front();
    return true;
  }
  if(recursive)
    return children_.getFirstPose(pose, recursive);
  return false;
}

bool Task::getLastPose(geometry_msgs::PoseStamped& pose, bool recursive) const
{
  if(!message_.poses.empty())
  {
    pose = message_.poses.back();
    return true;
  }
  if(recursive)
    return children_.getLastPose(pose, recursive);
  return false;
}

boost::shared_ptr<Task> Task::createChildTaskBefore(boost::shared_ptr<Task> task, std::string type)
{
  return children_.createTaskBefore(task, type);
}

std::string Task::getChildID(std::string id) const
{
  return message_.id+"/"+id;
}

void Task::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context)
{
  geometry_msgs::PoseStamped in_pose;
  if(getFirstPose(in_pose))
  {
    // \todo make waypoint distance a param
    if(length(vectorBetween(from_pose.pose, in_pose.pose))>10.0)
    {
      auto transit = updateTransitTo(from_pose, in_pose);
      auto preview_planner = context->pluginsLoader()->getPlugin<TaskToTaskWorkflow>("preview");
      if(preview_planner)
      {
        preview_planner->setGoal(transit);
        boost::shared_ptr<Task> plan;
        preview_planner->getResult(plan);
      }
    }
    else
      clearTransitTo();
  }
  else 
    in_pose = from_pose;

  if(!getLastPose(out_pose))
    out_pose = in_pose;
}


boost::shared_ptr<Task> Task::updateTransitTo(const geometry_msgs::PoseStamped& from_pose, const geometry_msgs::PoseStamped& in_pose)
{
  boost::shared_ptr<Task> transit;
  for(auto t: children_.tasks())
  {
    if (t->message().type == "transit" && t->message().id == getChildID("transit_to"))
    {
      transit = t;
      break;
    }
  }
  if(!transit)
  {
    boost::shared_ptr<Task> firstChild;
    if(!children_.tasks().empty())
      firstChild = children_.tasks().front();
    transit = createChildTaskBefore(firstChild, "transit");
    setChildID(transit, "transit_to");
  }
  auto m = transit->message();
  if(m.poses.size() == 2)
  {
    if(length(vectorBetween(from_pose.pose, m.poses[0].pose))<1.0 && length(vectorBetween(in_pose.pose, m.poses[1].pose))<1.0)
      return transit;
  }
  m.poses.clear();
  m.poses.push_back(from_pose);
  m.poses.push_back(in_pose);
  transit->update(m);
  return transit;
}

void Task::clearTransitTo()
{
  for(auto t: children_.tasks())
  {
    if (t->message().type == "transit" && t->message().id == getChildID("transit_to"))
    {
      auto m = t->message();
      m.poses.clear();
      t->update(m);
      break;
    }
  }
}

boost::shared_ptr<Task> Task::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return nullptr;
}

void Task::getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const
{
  for(auto m: marker_array_.markers)
    marker_array.markers.push_back(m);
  children().getDisplayMarkers(marker_array);
}

visualization_msgs::MarkerArray& Task::markerArray()
{
  return marker_array_;
}


}  // namespace project11_navigation

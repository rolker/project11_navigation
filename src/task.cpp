#include "project11_navigation/task.h"
#include <project11_navigation/utilities.h>
#include <ros/ros.h>

namespace project11_navigation
{

Task::Task(const project11_nav_msgs::Task& task_msg):
  message_(task_msg),
  children_(this),
  last_update_time_(ros::Time::now())
{

}

bool Task::update(const project11_nav_msgs::Task& task_msg)
{
  if(message_.id == task_msg.id)
  {
    if(message_ != task_msg)
      last_update_time_ = ros::Time::now();
    message_= task_msg;
    return true;
  }
  return false;
}

void Task::update(const std::vector<project11_nav_msgs::Task>& task_msgs)
{
  for(auto task_msg: task_msgs)
    update(task_msg);
  children_.update(task_msgs);
}

const project11_nav_msgs::Task & Task::message() const
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

void Task::setChildID(std::shared_ptr<Task> task, std::string id)
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

std::shared_ptr<Task> Task::createChildTaskBefore(std::shared_ptr<Task> task, std::string type)
{
  return children_.createTaskBefore(task, type);
}

std::string Task::getChildID(std::string id) const
{
  return message_.id+"/"+id;
}


std::shared_ptr<Task> Task::updateTransitTo(const geometry_msgs::PoseStamped& from_pose, const geometry_msgs::PoseStamped& in_pose)
{
  std::shared_ptr<Task> transit;
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
    std::shared_ptr<Task> firstChild;
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

}  // namespace project11_navigation

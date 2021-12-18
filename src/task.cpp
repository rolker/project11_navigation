#include "project11_navigation/task.h"
#include <ros/ros.h>

namespace project11_navigation
{

Task::Task(const project11_nav_msgs::Task& task_msg):
  message_(task_msg),
  children_(this)
{

}

bool Task::update(const project11_nav_msgs::Task& task_msg)
{
  if(message_.id == task_msg.id)
  {
    message_= task_msg;
    last_update_time_ = ros::Time::now();
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

void Task::setStatus(std::string status)
{
  message_.status = status;
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

ros::Time Task::lastUpdateTime() const
{
  return last_update_time_;  
}

std::vector<project11_nav_msgs::Task> Task::childrenTaskMessages() const
{
  return children_.taskMessages();
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

std::shared_ptr<Task> Task::getFirstChildTask() const
{
  return children_.getFirstTask();
}

std::shared_ptr<Task> Task::getFirstChildOfType(std::string type) const
{
  return children_.getFirstTaskOfType(type);
}

std::shared_ptr<Task> Task::getLastChildOfType(std::string type) const
{
  return children_.getLastTaskOfType(type);
}

std::shared_ptr<Task> Task::getFirstChildOfTypeAndID(std::string type, std::string id) const
{
  return children_.getFirstTaskOfTypeAndID(type, id);
}

std::shared_ptr<Task> Task::createChildTaskBefore(std::shared_ptr<Task> task)
{
  return children_.createTaskBefore(task);
}

void Task::updateTransitTo(const geometry_msgs::PoseStamped& in_pose)
{
  auto transit = getFirstChildOfTypeAndID("transit","tansit_to");
  if(!transit)
  {
    ROS_INFO_STREAM("creating a transit_to for " << message_.id);
    transit = createChildTaskBefore(getFirstChildTask());
    auto m = transit->message();
    m.type = "transit";
    m.poses.push_back(in_pose);
    transit->update(m);
    setChildID(transit, "transit_to");
  }
  else
    ROS_INFO_STREAM("transit_to exists for " << message_.id);
}

}  // namespace project11_navigation

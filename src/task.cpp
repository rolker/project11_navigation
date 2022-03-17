#include "project11_navigation/task.h"
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
  message_.status = status.as<std::string>();
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

bool Task::hasChildren() const
{
  return !children_.tasks().empty();
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

std::shared_ptr<Task> Task::getFirstUndoneChildTask() const
{
  return children_.getFirstUndoneTask();
}

std::shared_ptr<Task> Task::getFirstChildOfType(std::string type) const
{
  return children_.getFirstTaskOfType(type);
}

std::shared_ptr<Task> Task::getNextChildOfType(std::shared_ptr<Task> task) const
{
  return children_.getNextTaskOfType(task);
}


std::shared_ptr<Task> Task::getLastChildOfType(std::string type) const
{
  return children_.getLastTaskOfType(type);
}

std::shared_ptr<Task> Task::getFirstChildOfTypeAndID(std::string type, std::string id) const
{
  return children_.getFirstTaskOfTypeAndID(type, id);
}

std::shared_ptr<Task> Task::getFirstChildOfTypeAndIDOrCreate(std::string type, std::string id)
{
  auto ret = children_.getFirstTaskOfTypeAndID(type, id);
  if(!ret)
  {
    ret = createChildTaskBefore(getFirstChildTask(), type);
    setChildID(ret, id);
  }
  return ret;
}


std::shared_ptr<Task> Task::createChildTaskBefore(std::shared_ptr<Task> task, std::string type)
{
  return children_.createTaskBefore(task, type);
}

void Task::updateTransitTo(const geometry_msgs::PoseStamped& in_pose)
{
  auto transit = getFirstChildOfTypeAndIDOrCreate("transit","tansit_to");
  auto m = transit->message();
  m.poses.clear();
  m.poses.push_back(in_pose);
  transit->update(m);
}

}  // namespace project11_navigation

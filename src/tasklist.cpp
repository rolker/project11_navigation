#include "project11_navigation/tasklist.h"
#include "project11_navigation/task.h"
#include <ros/ros.h>

namespace project11_navigation
{

TaskList::TaskList(Task* parent):parent_task_(parent)
{

}

void TaskList::update(const std::vector<project11_nav_msgs::Task>& task_msgs)
{
  // Create a new vector adding tasks in order they appear.
  // Existing tasks may be copied from existing vector or new ones created if needed.
  std::vector<std::shared_ptr<Task> > new_task_list;
  // First, the direct children or top level tasks
  for(const auto& task_msg: task_msgs)
  {
    auto id_parts = splitChildID(task_msg.id);
    // Top level or direct child id? Yes if we have a first part but not a second part.
    if(id_parts.first.empty() && !id_parts.second.empty())
    {
      std::shared_ptr<Task> task;
      for(auto existing_task: tasks_)
      {
        if(existing_task->message().id == task_msg.id)
        {
          task = existing_task;
          break;
        }
      }
      if(!task)
        task = std::shared_ptr<Task>(new Task(task_msg));
      task->update(task_msgs);
      new_task_list.push_back(task);
    }
  }
  tasks_ = new_task_list;
}

std::pair<std::string, std::string> TaskList::splitChildID(const std::string& task_id) const
{
  std::pair<std::string, std::string> ret;
  std::string parent_path;
  if(parent_task_)
    parent_path = parent_task_->message().id+"/";
  // If we have a parent, does the id begin with the parent's id? 
  if(!parent_task_ || task_id.substr(0, parent_path.size()) == parent_path)
  {
    std::string child_part = task_id;
    // remove the common begining as well as the delimiter
    if(task_id.size() > parent_path.size())
      child_part = task_id.substr(parent_path.size());
    // find next delimiter, if any
    auto delimiter = child_part.find("/");
    if(delimiter == std::string::npos)
      ret.second = child_part;
    else
    {
      ret.first = child_part.substr(delimiter+1);
      ret.second = child_part.substr(0,delimiter);
    }
  }
  return ret;
}

const std::vector<std::shared_ptr<Task> >& TaskList::tasks() const
{
  return tasks_;
}

std::vector<project11_nav_msgs::Task> TaskList::taskMessages() const
{
  std::vector<project11_nav_msgs::Task> ret;
  std::vector<project11_nav_msgs::Task> children_messages;
  for(auto t: tasks_)
  {
    ret.push_back(t->message());
    auto this_children_messages = t->childrenTaskMessages();
    children_messages.insert(children_messages.end(), this_children_messages.begin(), this_children_messages.end());
  }
  ret.insert(ret.end(), children_messages.begin(), children_messages.end());
  return ret;
}

bool TaskList::getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive) const
{
  for(auto tp: tasks_)
    if(tp && tp->getFirstPose(pose, recursive))
      return true;
  return false;
}

bool TaskList::getLastPose(geometry_msgs::PoseStamped& pose, bool recursive) const
{
  for(auto t = tasks_.rbegin(); t != tasks_.rend(); t++)
    if(*t && (*t)->getLastPose(pose, recursive))
      return true;
  return false;
}

std::shared_ptr<Task> TaskList::getFirstTask() const
{
  if(!tasks_.empty())
    return tasks_.front();
  return std::shared_ptr<Task>();
}

std::shared_ptr<Task> TaskList::getFirstUndoneTask() const
{
  for(auto t: tasks_)
    if(!t->done())
      return t;
  return std::shared_ptr<Task>();
}


std::shared_ptr<Task> TaskList::getNextTask(std::shared_ptr<Task> task) const
{
  if(!task)
    return getFirstTask();
  auto task_iterator = tasks_.begin();
  while(task_iterator != tasks_.end())
  {
    auto task_to_check = *task_iterator;
    bool found = (task_to_check && task_to_check->message().id == task->message().id);
    task_iterator++;
    if(found && task_iterator != tasks_.end())
      return *task_iterator;
  }
  return std::shared_ptr<Task>();
}

std::shared_ptr<Task> TaskList::createTaskBefore(std::shared_ptr<Task> task, std::string type)
{
  auto task_iterator = tasks_.begin();
  while(task && task_iterator != tasks_.end() && *task_iterator != task)
    task_iterator++;
  
  std::shared_ptr<Task> ret;

  if(!task || task_iterator != tasks_.end())
  {
    // we found the target task
    project11_nav_msgs::Task tm;
    tm.type = type;
    if(task)
      tm.priority = task->message().priority;
    ret = std::make_shared<Task>(tm);
    tasks_.insert(task_iterator, ret);
  }
  return ret;
}

std::string TaskList::generateUniqueID(const std::string& prefix, std::shared_ptr<Task> skip) const
{
  std::string path;
  if(parent_task_)
    path = parent_task_->message().id+"/";
  for(int i = 0; i < 1000; i++)
  {
    std::string candidate = path+prefix;
    if(i < 10)
      candidate += "00";
    else if(i<100)
      candidate += "0";
    candidate += std::to_string(i);
    bool exists = false;
    for(auto t: tasks_)
      if(t && t->message().id == candidate)
      {
        if(!skip || t!=skip)
        {
          exists = true;
          break;
        }
      }
    if(!exists)
      return candidate;
  }

  return "";
}

bool TaskList::allDone(bool recursive) const
{
  for(auto t: tasks_)
    if(!t->done(recursive))
      return false;
  return true;
}

std::shared_ptr<Task> TaskList::getFirstTaskOfType(std::string type) const
{
  for(auto t: tasks_)
    if(t->message().type == type)
      return t;
  return std::shared_ptr<Task>();
}

std::shared_ptr<Task> TaskList::getNextTaskOfType(std::shared_ptr<Task> task) const
{
  if(task)
  {
    auto type = task->message().type;
    while(task)
    {
      task = getNextTask(task);
      if(task && task->message().type == type)
        return task;
    }
  }
  return std::shared_ptr<Task>();
}

std::shared_ptr<Task> TaskList::getFirstTaskOfTypeAndID(std::string type, std::string id) const
{
  for(auto t: tasks_)
    if(t->message().type == type)
    {
      auto id_parts = splitChildID(t->message().id);
      //ROS_INFO_STREAM(id_parts.first << ", " << id_parts.second << " ? " << id);
      if(id_parts.second == id)
        return t;
    }
  return std::shared_ptr<Task>();
}


std::shared_ptr<Task> TaskList::getLastTaskOfType(std::string type) const
{
  for(auto t = tasks_.rbegin(); t != tasks_.rend(); t++)
    if((*t)->message().type == type)
      return *t;
  return std::shared_ptr<Task>();
}

}  // namespace project11_navigation

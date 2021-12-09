#include "project11_navigation/task.h"

namespace project11_navigation
{

TaskList::TaskList(Task* parent):parent_task_(parent)
{

}

void TaskList::update(const std::vector<project11_nav_msgs::Task>& task_msgs)
{
  // Create a new vector adding tasks in order they appear.
  // Existing tasks may be copied from existing vector or new ones created is needed.
  std::vector<std::shared_ptr<Task> > new_task_list;
  // First, the direct children or top level tasks
  for(const auto& task_msg: task_msgs)
  {
    auto id_parts = splitChildID(task_msg.id);
    // Top level or direct child id? Yes if we have a first part but not a second part.
    if(!id_parts.first.empty() && id_parts.second.empty())
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
      ret.first = child_part;
    else
    {
      ret.first = child_part.substr(0,delimiter);
      ret.second = child_part.substr(delimiter+1);
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

Task::Task(const project11_nav_msgs::Task& task_msg):
  message_(task_msg),
  children_(this)
{

}

void Task::update(const std::vector<project11_nav_msgs::Task>& task_msgs)
{
  for(auto task_msg: task_msgs)
  {
    if(message_.id == task_msg.id)
    {
      message_= task_msg;
      last_update_time_ = ros::Time::now();
    }
  }
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


bool Task::done() const
{
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


}  // namespace project11_navigation

namespace std
{
  // compares pointers to Tasks by priority
  template<> class less<shared_ptr<project11_navigation::Task> >
  {
  public:
    bool operator()(const shared_ptr<project11_navigation::Task>& a, const shared_ptr<project11_navigation::Task>& b)
    {
      return a->message().priority > b->message().priority;
    }
  };
}



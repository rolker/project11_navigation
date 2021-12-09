#ifndef PROJECT11_NAVIGATION_TASK_H
#define PROJECT11_NAVIGATION_TASK_H

#include <project11_nav_msgs/Task.h>

namespace project11_navigation
{

class Task;

class TaskList
{
public:
  TaskList(Task* parent = nullptr);
  void update(const std::vector<project11_nav_msgs::Task>& task_msgs);

  const std::vector<std::shared_ptr<Task> >& tasks() const;
  std::vector<project11_nav_msgs::Task> taskMessages() const;
private:
  /// Split the id of a potential child task into the direct child part and remainder.
  /// Returns empty strings if the id is not for a child task.
  std::pair<std::string, std::string> splitChildID(const std::string& task_id) const;

  std::vector<std::shared_ptr<Task> > tasks_;
  Task* parent_task_ = nullptr;
};

// Represents the execution state of a task.
class Task
{
public:
  Task(const project11_nav_msgs::Task& task_msg);

  void update(const std::vector<project11_nav_msgs::Task>& task_msgs);

  const project11_nav_msgs::Task& message() const;
  bool done() const;

  void setDone();

  ros::Time lastUpdateTime() const;

  std::vector<project11_nav_msgs::Task> childrenTaskMessages() const;
private:
  project11_nav_msgs::Task message_;
  ros::Time last_update_time_;
  TaskList children_;
};

} // namespace project11_navigation

#endif

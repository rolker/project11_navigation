#ifndef PROJECT11_NAVIGATION_TASKLIST_H
#define PROJECT11_NAVIGATION_TASKLIST_H

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

  std::vector<std::shared_ptr<Task> > tasksByPriority(bool skip_done = false) const;

  bool getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive= false) const;
  bool getLastPose(geometry_msgs::PoseStamped& pose, bool recursive= false) const;

  std::shared_ptr<Task> createTaskBefore(std::shared_ptr<Task> task, std::string type = "");
  
  bool allDone(bool recursive=false) const;

  std::string generateUniqueID(const std::string &prefix, std::shared_ptr<Task> skip = std::shared_ptr<Task>()) const;

private:
  /// Split the id of a potential child task into the path and direct child part.
  /// Returns empty strings if the id is not for a child task.
  std::pair<std::string, std::string> splitChildID(const std::string& task_id) const;

  std::vector<std::shared_ptr<Task> > tasks_;
  Task* parent_task_ = nullptr;
};

} // namespace project11_navigation

#endif

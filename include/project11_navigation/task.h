#ifndef PROJECT11_NAVIGATION_TASK_H
#define PROJECT11_NAVIGATION_TASK_H

#include <project11_navigation/tasklist.h>

namespace project11_navigation
{
// Represents the execution state of a task.
class Task
{
public:
  Task(const project11_nav_msgs::Task& task_msg);

  bool update(const project11_nav_msgs::Task& task_msg);
  void update(const std::vector<project11_nav_msgs::Task>& task_msgs);

  const project11_nav_msgs::Task& message() const;
  bool done(bool recursive=false) const;

  void setDone();
  void setStatus(std::string status);
  void setChildID(std::shared_ptr<Task> task, std::string id);
  void setID(std::string id);

  ros::Time lastUpdateTime() const;

  std::vector<project11_nav_msgs::Task> childrenTaskMessages() const;

  bool getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;
  bool getLastPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;

  std::shared_ptr<Task> getFirstChildTask() const;
  std::shared_ptr<Task> getFirstChildOfType(std::string type) const;
  std::shared_ptr<Task> getLastChildOfType(std::string type) const;
  std::shared_ptr<Task> getFirstChildOfTypeAndID(std::string type, std::string id) const;

  std::shared_ptr<Task> createChildTaskBefore(std::shared_ptr<Task> task = std::shared_ptr<Task>());

  void updateTransitTo(const geometry_msgs::PoseStamped& from_pose);

private:
  project11_nav_msgs::Task message_;
  ros::Time last_update_time_;
  TaskList children_;
};

} // namespace project11_navigation

#endif

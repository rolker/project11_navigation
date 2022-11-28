#ifndef PROJECT11_NAVIGATION_TASK_H
#define PROJECT11_NAVIGATION_TASK_H

#include <project11_navigation/tasklist.h>
#include <yaml-cpp/yaml.h>

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

  const TaskList& children() const;

  void setDone();
  void setChildID(std::shared_ptr<Task> task, std::string id);
  void setID(std::string id);
  std::string getChildID(std::string id) const;


  YAML::Node data() const;
  void setData(const YAML::Node& data);

  YAML::Node status() const;
  void setStatus(const YAML::Node &status);

  ros::Time lastUpdateTime() const;

  bool getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;
  bool getLastPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;

  std::shared_ptr<Task> createChildTaskBefore(std::shared_ptr<Task> task = std::shared_ptr<Task>(), std::string type = "");

  std::shared_ptr<Task> updateTransitTo(const geometry_msgs::PoseStamped& from_pose, const geometry_msgs::PoseStamped& in_pose);

  void clearTransitTo();

private:
  project11_nav_msgs::Task message_;
  TaskList children_;
  ros::Time last_update_time_;
};

} // namespace project11_navigation

#endif

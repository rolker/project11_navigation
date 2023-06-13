#ifndef PROJECT11_NAVIGATION_TASKLIST_H
#define PROJECT11_NAVIGATION_TASKLIST_H

#include <project11_nav_msgs/TaskInformation.h>
#include <visualization_msgs/MarkerArray.h>

namespace project11_navigation
{

class Task;

class TaskList
{
public:
  TaskList(Task* parent = nullptr);
  void update(const std::vector<project11_nav_msgs::TaskInformation>& task_msgs);

  const std::vector<boost::shared_ptr<Task> >& tasks() const;
  std::vector<project11_nav_msgs::TaskInformation> taskMessages() const;

  std::vector<boost::shared_ptr<Task> > tasksByPriority(bool skip_done = false) const;

  bool getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive= false) const;
  bool getLastPose(geometry_msgs::PoseStamped& pose, bool recursive= false) const;

  boost::shared_ptr<Task> createTaskBefore(boost::shared_ptr<Task> task, std::string type = "");
  
  bool allDone(bool recursive=false) const;

  std::string generateUniqueID(const std::string &prefix, boost::shared_ptr<Task> skip = {}) const; //boost::shared_ptr<Task>()) const;

  void getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const;

private:
  /// Split the id of a potential child task into the path and direct child part.
  /// Returns empty strings if the id is not for a child task.
  std::pair<std::string, std::string> splitChildID(const std::string& task_id) const;

  std::vector<boost::shared_ptr<Task> > tasks_;
  Task* parent_task_ = nullptr;
};

} // namespace project11_navigation

#endif

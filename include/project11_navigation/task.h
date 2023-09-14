#ifndef PROJECT11_NAVIGATION_TASK_H
#define PROJECT11_NAVIGATION_TASK_H

#include <project11_navigation/tasklist.h>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/weak_ptr.hpp>

namespace project11_navigation
{

class Context;
class TaskPlugins;

// Represents the execution state of a task.
class Task
{
public:
  using Ptr = boost::shared_ptr<Task>;
  using ConstPtr = boost::shared_ptr<const Task>;


  // \todo make this private without breaking the create() method.
  Task();
  virtual ~Task() = default;

  // Creates a new Task or derived Task if a creator is set to do so.
  static boost::shared_ptr<Task> create(const project11_nav_msgs::TaskInformation& task_msg, Task::Ptr parent);

  // Sets a factory function that creates new Tasks or derived Tasks.
  static void setCreator(std::shared_ptr<TaskPlugins> creator);

  bool update(const project11_nav_msgs::TaskInformation& task_msg, bool check_id = true);
  void update(const std::vector<project11_nav_msgs::TaskInformation>& task_msgs);

  const project11_nav_msgs::TaskInformation& message() const;
  bool done(bool recursive=false) const;

  const TaskList& children() const;

  void setDone();
  void setChildID(boost::shared_ptr<Task> task, std::string id);
  void setID(std::string id);
  std::string getChildID(std::string id) const;


  YAML::Node data() const;
  void setData(const YAML::Node& data);

  YAML::Node dataItem(std::string key, bool recurse_up=true) const;

  YAML::Node status() const;
  void setStatus(const YAML::Node &status);

  ros::Time lastUpdateTime() const;

  bool getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;
  bool getLastPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;

  boost::shared_ptr<Task> createChildTaskBefore(boost::shared_ptr<Task> task = {}, std::string type = "");


  virtual void updateTransit(const geometry_msgs::PoseStamped& starting_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<Context> context);
  virtual boost::shared_ptr<Task> getCurrentNavigationTask();


  boost::shared_ptr<Task> updateTransitTo(const geometry_msgs::PoseStamped& from_pose, const geometry_msgs::PoseStamped& in_pose);

  void clearTransitTo();

  // Allows visualization markers to be added to preview task navigation.
  virtual void getDisplayMarkers(visualization_msgs::MarkerArray& marker_array) const;

  visualization_msgs::MarkerArray& markerArray();

  boost::shared_ptr<Task> self();

private:
  boost::weak_ptr<Task> self_;
  boost::weak_ptr<Task> parent_task_;

  project11_nav_msgs::TaskInformation message_;
  TaskList children_;
  ros::Time last_update_time_;

  static std::shared_ptr<TaskPlugins> creator_;

  visualization_msgs::MarkerArray marker_array_;
};

} // namespace project11_navigation

#endif

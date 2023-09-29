#ifndef PROJECT11_NAVIGATION_TASK_H
#define PROJECT11_NAVIGATION_TASK_H

#include <project11_navigation/task_list.h>
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

  /// Creates a new Task or derived Task if a creator is set to do so.
  static boost::shared_ptr<Task> create(const project11_nav_msgs::TaskInformation& task_msg, Task::Ptr parent);

  /// Sets a factory function that creates new Tasks or derived Tasks.
  static void setCreator(std::shared_ptr<TaskPlugins> creator);

  /// Update or replace this task's TaskInformation.
  /// If check_id is true (default), only replaces the message
  /// if the new message has the same id as the existing message.
  /// Returns true if the message got replaced.
  bool update(const project11_nav_msgs::TaskInformation& task_msg, bool check_id = true);

  /// Updates TaskInformation messages for this task
  /// and children tasks. Existing children tasks may be updated
  /// and new children tasks may be added. Existing children tasks
  /// that are not in task_msgs are discarded.
  void update(const std::vector<project11_nav_msgs::TaskInformation>& task_msgs);

  /// Returns TaskInformation message associated with this task. 
  const project11_nav_msgs::TaskInformation& message() const;

  /// Returns true if this task and optionally children tasks
  /// are all done. Children tasks are only checked if the
  /// recursive argument is true. Defaults to not checking
  /// children. 
  bool done(bool recursive=false) const;

  const TaskList& children() const;

  /// Sets the TaskInformation's done flag to true and updates
  /// last changed time.
  void setDone();

  /// Sets the id of the task argument as a child of this task.
  /// The id parameter gets added to this TaskInformation's id
  /// separated with a '/'.
  void setChildID(boost::shared_ptr<Task> task, std::string id);

  /// Sets the TaskInformation id and updates the modification time.
  void setID(std::string id);

  /// Generates a child id by adding '/' and id to this task's id.  
  std::string getChildID(std::string id) const;


  /// Returns YAML decoded data from the message.
  YAML::Node data() const;

  /// Sets the data of the TaskInformation message and
  /// updates the modification time.
  void setData(const YAML::Node& data);

  /// Returns a data item from the parsed data member of the message
  /// optionally recursing up the parents.
  YAML::Node dataItem(std::string key, bool recurse_up=true) const;

  /// Returns YAML decoded data from the message.
  YAML::Node status() const;

  /// Sets the status of the TaskInformation message and
  /// updates the modification time.
  void setStatus(const YAML::Node &status);

  /// Returns time TaskInformation was last updated
  ros::Time lastUpdateTime() const;

  /// Returns the first pose in this task's pose list. If
  /// this task's pose list is empty, optionally check
  /// the children recursively (depth first).
  bool getFirstPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;

  /// Returns the last pose in this task's pose list. If
  /// this task's pose list is empty, optionally check
  /// the children recursively (depth first).
  bool getLastPose(geometry_msgs::PoseStamped& pose, bool recursive = false) const;

  /// Creates a new task and inserts it as a child before task
  /// if not null. If task argument is null, the
  /// new task is appended to the children. No new task
  /// is created if task is not nullptr and is not found.
  boost::shared_ptr<Task> createChildTaskBefore(boost::shared_ptr<Task> task = {}, std::string type = "");


  /// \todo split the following to a TaskPluginBase or similar
  /// task and make the above a library that can be used by other 
  /// packages

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

#ifndef PROJECT11_NAVIGATION_NAVIGATOR_H
#define PROJECT11_NAVIGATION_NAVIGATOR_H

#include <ros/ros.h>
#include <project11_nav_msgs/TaskInformation.h>
#include <project11_navigation/context.h>
#include <project11_navigation/interfaces/tasklist_to_twist_workflow.h>

namespace project11_navigation
{

/// Executive that accepts tasks and executes them.
class Navigator
{
public:
  Navigator();
  ~Navigator();

protected:
  void updateTasks(const std::vector<project11_nav_msgs::TaskInformation>& tasks);
  virtual void done();
  virtual void iterate(const ros::TimerEvent& event);

  std::shared_ptr<TaskList> task_list_;
  std::shared_ptr<Context> context_;
private:
  ros::NodeHandle nodeHandle_;
  
  ros::Timer iterate_timer_;

  std::shared_ptr<Robot> robot_;

  double controller_frequency_ = 10.0;

  boost::shared_ptr<TaskListToTwistWorkflow> task_manager_;

  // Task list as provided by the user
  std::vector<project11_nav_msgs::TaskInformation> task_messages_;

  ros::Publisher display_pub_;
};

} // namespace project11_navigation

#endif

#ifndef PROJECT11_NAVIGATION_NAVIGATOR_H
#define PROJECT11_NAVIGATION_NAVIGATOR_H

#include <ros/ros.h>
#include <project11_nav_msgs/TaskInformation.h>
#include <actionlib/server/simple_action_server.h>
#include <project11_navigation/RunTasksAction.h>

#include <project11_navigation/context.h>
#include <project11_navigation/task_list.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_sqlite_logger.h>

namespace project11_navigation
{

/// Action server that accepts tasks and executes them.
class Navigator
{
public:
  Navigator(std::string name = "navigator");
  ~Navigator();

private:
  void goalCallback();
  void preemptCallback();

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /// Builds and returns the Behavior Tree
  BT::Tree buildBehaviorTree();

  ros::NodeHandle node_handle_;

  actionlib::SimpleActionServer<project11_navigation::RunTasksAction> action_server_;

  std::shared_ptr<Context> context_;

  ros::Subscriber odom_sub_;

  ros::Publisher display_pub_;

  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::shared_ptr<BT::Groot2Publisher> groot_;
  std::shared_ptr<BT::FileLogger2> logger_;
  std::shared_ptr<BT::SqliteLogger> sqlite_logger_;
  std::shared_ptr<BT::StdCoutLogger> console_logger_;
};

} // namespace project11_navigation

#endif

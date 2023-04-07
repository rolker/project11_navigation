#ifndef PROJECT11_NAVIGATION_CONTEXT_H
#define PROJECT11_NAVIGATION_CONTEXT_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/environment.h>
#include <mutex>

namespace project11_navigation
{

class PluginsLoader;
class Robot;
class Task;
class TaskWrapper;

// Assembles the relevant data for accomplishing navigation tasks.
class Context
{
public:
  typedef std::shared_ptr<Context> Ptr;
  typedef std::shared_ptr<const Context> ConstPtr;

  Context();

  RobotCapabilities getRobotCapabilities();
  nav_msgs::Odometry getOdometry();

  Environment& environment();

  bool getOutputEnabled();
  tf2_ros::Buffer& tfBuffer();
  geometry_msgs::PoseStamped getPoseInFrame(std::string frame_id);

  boost::shared_ptr<TaskWrapper> getTaskWrapper(std::shared_ptr<Task> task);

  std::shared_ptr<PluginsLoader> pluginsLoader();

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap();
  void setCostmap(std::shared_ptr<costmap_2d::Costmap2DROS> costmap);

private:
  void updateRobotCapabilities(const RobotCapabilities& robot_capabilities);
  void updateOdometry(const nav_msgs::Odometry& odom);
  void updateOutputEnabled(bool enabled);

  RobotCapabilities robot_capabilities_;
  std::mutex robot_capabilities_mutex_;

  Environment environment_;

  nav_msgs::Odometry odom_;
  std::mutex odom_mutex_;

  bool output_enabled_ = true;
  std::mutex output_enabled_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<PluginsLoader> plugins_loader_;

  std::string default_task_wrapper_ = "generic";

  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  std::mutex costmap_mutex_;

  friend class Robot;


};

} // namespace project11_navigation

#endif

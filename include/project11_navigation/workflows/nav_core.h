#ifndef PROJECT11_NAVIGATION_WORKFLOWS_NAV_CORE_H
#define PROJECT11_NAVIGATION_WORKFLOWS_NAV_CORE_H

#include <project11_navigation/interfaces/task_to_twist_workflow.h>

#include <pluginlib/class_loader.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <future>

namespace project11_navigation
{

class NavCore: public TaskToTwistWorkflow
{
public:
  NavCore();
  ~NavCore();

  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const std::shared_ptr<Task>& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;

private:
  bool planThread(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector< geometry_msgs::PoseStamped > &plan);
  void checkForNewPlan();
  void publishPlan();
  void updatePlan();
  
  Context::Ptr context_;

  boost::shared_ptr<nav_core::BaseLocalPlanner> controller_;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;

  std::shared_ptr<costmap_2d::Costmap2DROS> planner_costmap_;
  std::shared_ptr<costmap_2d::Costmap2DROS> controller_costmap_;

  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> planner_loader_;
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> controller_loader_;

  std::vector<geometry_msgs::PoseStamped> current_plan_;
  std::vector<geometry_msgs::PoseStamped> new_plan_;
  
  std::shared_ptr<Task> task_;
  ros::Time task_update_time_;
  std::future<bool> plan_ready_;

  ros::Publisher plan_publisher_;
};

}  // namespace project11_navigation

#endif

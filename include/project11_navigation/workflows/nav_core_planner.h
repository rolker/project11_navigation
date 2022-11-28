#ifndef PROJECT11_NAVIGATION_WORKFLOWS_NAV_CORE_PLANNER_H
#define PROJECT11_NAVIGATION_WORKFLOWS_NAV_CORE_PLANNER_H

#include <project11_navigation/interfaces/task_to_task_workflow.h>

#include <pluginlib/class_loader.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <future>

namespace project11_navigation
{

class NavCorePlanner: public TaskToTaskWorkflow
{
public:
  NavCorePlanner();
  ~NavCorePlanner();

  void configure(std::string name, Context::Ptr context) override;
  void setGoal(const std::shared_ptr<Task>& input) override;
  bool running() override;
  bool getResult(std::shared_ptr<project11_navigation::Task>& output) override;

private:
  bool planThread(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector< geometry_msgs::PoseStamped > &plan);
  void checkForNewPlan();
  void publishPlan();
  void updatePlan();
  
  Context::Ptr context_;

  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
  std::string global_planner_;
  std::string name_;

  std::shared_ptr<costmap_2d::Costmap2DROS> planner_costmap_;

  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> planner_loader_;

  std::vector<geometry_msgs::PoseStamped> current_plan_;
  std::vector<geometry_msgs::PoseStamped> new_plan_;
  
  std::shared_ptr<Task> task_ = nullptr;
  std::shared_ptr<Task> output_task_ = nullptr;

  ros::Time task_update_time_;
  std::future<bool> plan_ready_;

  ros::Publisher plan_publisher_;

  std::string output_task_type_ = "follow_trajectory";
  std::string output_task_name_ = "navigation_trajectory";
};

}  // namespace project11_navigation

#endif

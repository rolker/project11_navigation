#include "project11_navigation/workflows/nav_core.h"

#include <ros/ros.h>
#include <project11_navigation/utilities.h>
#include <nav_msgs/Path.h>

namespace project11_navigation
{

NavCore::NavCore():
  planner_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
  controller_loader_("nav_core", "nav_core::BaseLocalPlanner")
{

}

NavCore::~NavCore()
{
  planner_.reset();
  controller_.reset();

}

void NavCore::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh("~/" + name);
  
  std::string global_planner, local_planner;
  nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
  nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
  
  planner_costmap_ = std::make_shared<costmap_2d::Costmap2DROS>(name+"/global_costmap", context->tfBuffer());

  ROS_INFO_STREAM("Loading " << global_planner);

  try
  {
    planner_ = planner_loader_.createInstance(global_planner);
    planner_->initialize(name+"/"+planner_loader_.getName(global_planner), planner_costmap_.get());
  }
  catch(const std::exception& e)
  {
    ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), e.what());
    exit(1);
  }

  controller_costmap_ = std::make_shared<costmap_2d::Costmap2DROS>(name+"/local_costmap", context->tfBuffer());

  ROS_INFO_STREAM("Loading " << local_planner);

  try
  {
    controller_ = controller_loader_.createInstance(local_planner);
    controller_->initialize(name+"/"+controller_loader_.getName(local_planner), &context->tfBuffer(), controller_costmap_.get());
  }
  catch(const std::exception& e)
  {
    ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), e.what());
      exit(1);
  }

  planner_costmap_->start();
  controller_costmap_->start();

  plan_publisher_ = nh.advertise<nav_msgs::Path>("plan", 1);
  
}

void NavCore::setGoal(const std::shared_ptr<Task>& input)
{
  if(input)
    ROS_INFO_STREAM("task:\n" << input->message());
  else
    ROS_INFO_STREAM("empty task");
  
  if(task_ != input)
  {
    task_update_time_ = ros::Time();
    ROS_INFO_STREAM("it's a new task");
  }
  task_ = input;
  updatePlan();
}

void NavCore::updatePlan()
{
  // make sure we need an update
  if(task_ && task_->lastUpdateTime() == task_update_time_)
    return;

  ROS_INFO_STREAM("different task or same with different update time");

  current_plan_.clear();
  controller_->setPlan(current_plan_);
  publishPlan();
  if(task_)
    if(task_->message().poses.empty())
    {
      task_->setDone();
      task_update_time_ = task_->lastUpdateTime();
      ROS_INFO_STREAM("no goal pose");
    }
    else
    {
      if(!task_->done())
      {
        auto goal = task_->message().poses.front();
        geometry_msgs::PoseStamped start = context_->getPoseInFrame(goal.header.frame_id);
        ROS_INFO_STREAM("start:\n" << start);
        new_plan_.clear();
        plan_ready_ = std::async(&NavCore::planThread, this, start, goal, std::ref(new_plan_));
      }
      task_update_time_ = task_->lastUpdateTime();
    }
  else
    task_update_time_ = ros::Time();

}

bool NavCore::planThread(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector< geometry_msgs::PoseStamped > &plan)
{
  return planner_->makePlan(start, goal, plan);
}

bool NavCore::running()
{
  if(task_ && !task_->done())
  {
    if (controller_->isGoalReached())
    {
      ROS_INFO_STREAM("Task done: " << task_->message().id);
      task_->setDone();
      current_plan_.clear();
      controller_->setPlan(current_plan_);
      publishPlan();
    }
    return !task_->done();
  }
  return false;
}

bool NavCore::getResult(geometry_msgs::TwistStamped& output)
{
  updatePlan();
  checkForNewPlan();
  if(running())
      return controller_->computeVelocityCommands(output.twist);
  return false;
}

void NavCore::checkForNewPlan()
{
  if(task_ && !task_->message().poses.empty() && current_plan_.empty() && plan_ready_.valid())
  {
    auto status = plan_ready_.wait_for(std::chrono::milliseconds(10));
    if(status == std::future_status::ready)
    {
      bool success = plan_ready_.get();
      if(success)
      {
        current_plan_ = new_plan_;
        if(!current_plan_.empty())
         current_plan_.front().header.stamp = ros::Time::now();
        adjustTrajectoryForSpeed(current_plan_, context_->getRobotCapabilities().default_velocity.linear.x);
        controller_->setPlan(current_plan_);
        publishPlan();
      }
    }
  }
}

void NavCore::publishPlan()
{
  nav_msgs::Path path;
  if(!current_plan_.empty())
  {
    path.header = current_plan_.front().header;
    path.poses = current_plan_;
  }
  plan_publisher_.publish(path);
}

}  // namespace project11_navigation

#include "project11_navigation/workflows/nav_core_planner.h"

#include <ros/ros.h>
#include <project11_navigation/utilities.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::NavCorePlanner, project11_navigation::TaskToTaskWorkflow)

namespace project11_navigation
{

NavCorePlanner::NavCorePlanner():
  planner_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{

}

NavCorePlanner::~NavCorePlanner()
{
  planner_.reset();
}

void NavCorePlanner::configure(std::string name, Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh("~/" + name);
  name_ = name;

  nh.param("output_task_type", output_task_type_, output_task_type_);
  nh.param("output_task_name", output_task_name_, output_task_name_);
  nh.param("base_global_planner", global_planner_, std::string("navfn/NavfnROS"));

  plan_publisher_ = nh.advertise<nav_msgs::Path>("plan", 1);
}

void NavCorePlanner::setGoal(const boost::shared_ptr<Task>& input)
{
  if(input)
    ROS_INFO_STREAM("task:\n" << input->message());
  else
    ROS_INFO_STREAM("empty task");
  
  if(!input || task_ != input)
  {
    task_update_time_ = ros::Time();
    ROS_INFO_STREAM("it's a new or empty task");
    current_plan_.clear();
    plan_ready_ = std::future<bool>();
    output_task_.reset();
  }
  task_ = input;
  updatePlan();
}

void NavCorePlanner::updatePlan()
{
  publishPlan();
  if(task_)
    if(task_->message().poses.size() < 2)
    {
      task_->setDone();
      task_update_time_ = task_->lastUpdateTime();
      current_plan_.clear();
      plan_ready_ = std::future<bool>();
      output_task_.reset();
      ROS_INFO_STREAM("no start and goal poses");
    }
    else
    {
      if(!task_->done() && !plan_ready_.valid())
      {
        auto start = task_->message().poses[0]; 
        auto goal = task_->message().poses[1];
        std::string map_frame = goal.header.frame_id;
        if(context_->costmap())
          map_frame = context_->costmap()->getGlobalFrameID();
        if(start.header.frame_id != map_frame)
        {
          try
          {
           context_->tfBuffer().transform(start, start, map_frame);
            start.header.frame_id = map_frame;
          }
          catch(const std::exception& e)
          {
            ROS_WARN_STREAM(e.what());
            return;
          }
        }
        if(goal.header.frame_id != map_frame)
        {
          try
          {
           context_->tfBuffer().transform(goal, goal, map_frame);
            goal.header.frame_id = map_frame;
          }
          catch(const std::exception& e)
          {
            ROS_WARN_STREAM(e.what());
            return;
          }
        }
        ROS_DEBUG_STREAM("start:\n" << start);
        ROS_DEBUG_STREAM("goal:\n" << goal);
        new_plan_.clear();

        // wait until the last minute to init planner so that costmap exists
        if(!planner_)
        {
          ROS_INFO_STREAM("Loading " << global_planner_);
          planner_costmap_ = context_->costmap();
          costmap_2d::Costmap2DROS* costmap_ptr = nullptr;
          if (planner_costmap_)
            costmap_ptr = planner_costmap_.get();

          try
          {
            planner_ = planner_loader_.createInstance(global_planner_);
            planner_->initialize(name_+"/"+planner_loader_.getName(global_planner_), costmap_ptr);
          }
          catch(const std::exception& e)
          {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_.c_str(), e.what());
            exit(1);
          }
        }

        plan_ready_ = std::async(&NavCorePlanner::planThread, this, start, goal, std::ref(new_plan_));
      }
      task_update_time_ = task_->lastUpdateTime();
    }
  else
    task_update_time_ = ros::Time();

}

bool NavCorePlanner::planThread(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector< geometry_msgs::PoseStamped > &plan)
{
  return planner_->makePlan(start, goal, plan);
}

bool NavCorePlanner::running()
{
  return(task_ && !task_->message().poses.empty());// && current_plan_.empty() && !plan_ready_.valid());
}

bool NavCorePlanner::getResult(boost::shared_ptr<Task>& output)
{
  checkForNewPlan();
  if(output_task_)
  {
    output = output_task_;
    return true;
  }
  return false;
}

void NavCorePlanner::checkForNewPlan()
{
  if(task_ && plan_ready_.valid())
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
        for(auto t: task_->children().tasks())
          if(t->message().type == output_task_type_ && t->message().id == task_->getChildID(output_task_name_))
          {
            output_task_ = t;
            break;
          }
        if(!output_task_)
        {
          output_task_ = task_->createChildTaskBefore(project11_navigation::Task::Ptr(),output_task_type_);
          task_->setChildID(output_task_, output_task_name_);
        }
        auto out_msg = output_task_->message();
        out_msg.curved_trajectories.clear();
        out_msg.poses = current_plan_;
        output_task_->update(out_msg);

        publishPlan();
        updatePlan();
      }
    }
  }
}

void NavCorePlanner::publishPlan()
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

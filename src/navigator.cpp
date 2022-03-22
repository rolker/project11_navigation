#include <project11_navigation/navigator.h>
#include <ros/ros.h>
#include <project11_navigation/robot.h>
#include <project11_navigation/workflows/task_manager.h>
#include <project11_navigation/plugins_loader.h>

namespace project11_navigation
{

Navigator::Navigator()
{

  ros::param::param("~controller_frequency", controller_frequency_, controller_frequency_);

  double controller_period = 1.0/controller_frequency_;

  ROS_INFO_STREAM("Controller frequency: " << controller_frequency_ << " period: " << controller_period);

  context_ = Context::Ptr(new Context);
  context_->pluginsLoader()->configure(context_);

  robot_ = std::shared_ptr<Robot>(new Robot(context_));

  if(ros::param::has("~costmap"))
  {
    auto costmap = std::make_shared<costmap_2d::Costmap2DROS>("costmap", context_->tfBuffer());
    context_->setCostmap(costmap);
  }

  std::string task_manager_plugin = ros::param::param<std::string>("~task_manager_plugin", "task_manager");
  task_manager_ = context_->pluginsLoader()->getPlugin<TaskListToTwistWorkflow>(task_manager_plugin);

  if(!task_manager_)
    ROS_FATAL_STREAM("Unable to load toplevel task manager plugin: " << task_manager_plugin);

  iterate_timer_ = nodeHandle_.createTimer(ros::Duration(controller_period), &Navigator::iterate, this);
}

Navigator::~Navigator()
{

}

void Navigator::updateTasks(const std::vector<project11_nav_msgs::Task>& tasks)
{
  ROS_DEBUG_STREAM("update with " << tasks.size() << " tasks");
  task_messages_ = tasks;
  if(tasks.empty())
    task_list_.reset();
  else
  {
    if(!task_list_)
      task_list_ = std::make_shared<TaskList>();
    task_list_->update(tasks);
  }
  task_manager_->setGoal(task_list_);
}

void Navigator::iterate(const ros::TimerEvent& event)
{
  ROS_DEBUG_STREAM("last expected: " << event.last_expected << " last real: " << event.last_real << " current_expected: " << event.current_expected << " current real: " << event.current_real << " last duration: " << event.profile.last_duration);

  if(task_manager_->running())
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "Task manager is running");
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = robot_->baseFrame();
    if(cmd_vel.header.frame_id.empty())
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "Waiting for odom with non-empty child_frame_id");
      return;
    }
    task_manager_->getResult(cmd_vel);
    robot_->sendControls(cmd_vel);
  }
  else
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "Task manager is NOT running");
    done();
  }
}

void Navigator::done()
{

}

} // namespace project11_navigation

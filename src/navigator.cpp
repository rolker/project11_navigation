#include <project11_navigation/navigator.h>
#include <ros/ros.h>
#include <project11_navigation/robot.h>
#include <project11_navigation/workflows/task_manager.h>
#include <project11_navigation/plugins_loader.h>
#include <project11_navigation/task_plugins.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

Navigator::Navigator()
{

  ros::param::param("~controller_frequency", controller_frequency_, controller_frequency_);

  double controller_period = 1.0/controller_frequency_;

  ROS_INFO_STREAM("Controller frequency: " << controller_frequency_ << " period: " << controller_period);

  NavigatorSettings nav_settings;
  ros::NodeHandle private_nh("~");
  nav_settings.waypoint_reached_distance = readDoubleOrIntParameter(private_nh, "waypoint_reached_distance", nav_settings.waypoint_reached_distance);

  nav_settings.survey_lead_in_distance = readDoubleOrIntParameter(private_nh, "survey_lead_in_distance", nav_settings.survey_lead_in_distance);

  context_ = Context::Ptr(new Context(nav_settings));
  context_->pluginsLoader()->configure(context_);
  context_->taskPlugins()->configure(context_);
  Task::setCreator(context_->taskPlugins());

  robot_ = std::shared_ptr<Robot>(new Robot(context_));

  std::string task_manager_plugin = ros::param::param<std::string>("~task_manager_plugin", "task_manager");
  task_manager_ = context_->pluginsLoader()->getPlugin<TaskListToTwistWorkflow>(task_manager_plugin);

  if(!task_manager_)
    ROS_FATAL_STREAM("Unable to load toplevel task manager plugin: " << task_manager_plugin);

  display_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);


  iterate_timer_ = nodeHandle_.createTimer(ros::Duration(controller_period), &Navigator::iterate, this);
}

Navigator::~Navigator()
{

}

void Navigator::updateTasks(const std::vector<project11_nav_msgs::TaskInformation>& tasks)
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

  visualization_msgs::MarkerArray array;
  robot_->updateMarkers(array);
  if(!array.markers.empty())
    display_pub_.publish(array);

  if(task_manager_->running())
  {
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = robot_->baseFrame();
    if(cmd_vel.header.frame_id.empty())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Waiting for odom with non-empty child_frame_id");
      return;
    }
    task_manager_->getResult(cmd_vel);
    robot_->sendControls(cmd_vel);
  }
  else
  {
    done();
  }
}

void Navigator::done()
{

}

} // namespace project11_navigation

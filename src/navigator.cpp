#include <project11_navigation/navigator.h>
#include <ros/ros.h>
#include <project11_navigation/robot.h>
#include <project11_navigation/workflows/task_manager.h>

namespace project11_navigation
{

Navigator::Navigator()
{

  ros::param::param("~controller_frequency", controller_frequency_, controller_frequency_);

  double controller_period = 1.0/controller_frequency_;

  ROS_INFO_STREAM("Controller frequency: " << controller_frequency_ << " period: " << controller_period);

  ros::param::param("~base_frame", base_frame_, base_frame_);

  context_ = Context::Ptr(new Context);
  robot_ = std::shared_ptr<Robot>(new Robot(context_));

  task_manager_ = std::make_shared<TaskManager>();

  task_manager_->configure("task_manager", context_);

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
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    if(!base_frame_.empty())
      cmd_vel.header.frame_id = base_frame_;
    else
    {
      auto odom = context_->getOdometry();
      if(odom.child_frame_id.empty())
      {
        ROS_INFO_STREAM_THROTTLE(1.0, "Waiting for odom with non-empty child_frame_id");
        return;
      }
      cmd_vel.header.frame_id = odom.child_frame_id;
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

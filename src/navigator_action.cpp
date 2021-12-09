#include "project11_navigation/navigator_action.h"

namespace project11_navigation
{


NavigatorAction::NavigatorAction(std::string name):
  action_server_(ros::NodeHandle(), name+"/run_tasks", false)
{
  action_server_.registerGoalCallback(std::bind(&NavigatorAction::goalCallback, this));
  action_server_.registerPreemptCallback(std::bind(&NavigatorAction::preemptCallback, this));
  action_server_.start();
}

NavigatorAction::~NavigatorAction()
{

}

void NavigatorAction::goalCallback()
{
  auto goal = action_server_.acceptNewGoal();
  ROS_INFO_STREAM("got a goal");
  updateTasks(goal->tasks);
}

void NavigatorAction::preemptCallback()
{
  action_server_.setPreempted();
}

void NavigatorAction::iterate(const ros::TimerEvent& event)
{
  Navigator::iterate(event);
  if(action_server_.isActive() && event.current_real - last_feedback_send_time_ > ros::Duration(1.0))
  {
    project11_navigation::RunTasksFeedback feedback;
    if(task_list_)
      feedback.tasks = task_list_->taskMessages();
    action_server_.publishFeedback(feedback);
    last_feedback_send_time_ = event.current_real;
  }
}

void NavigatorAction::done()
{
  if(action_server_.isActive())
    action_server_.setSucceeded();
}

} // namespace project11_navigation

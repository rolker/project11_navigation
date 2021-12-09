#ifndef PROJECT11_NAVIGATION_NAVIGATOR_ACTION_H
#define PROJECT11_NAVIGATION_NAVIGATOR_ACTION_H

#include <project11_navigation/navigator.h>
#include <actionlib/server/simple_action_server.h>
#include <project11_navigation/RunTasksAction.h>

namespace project11_navigation
{

class NavigatorAction: public Navigator
{
public:
  NavigatorAction(std::string name);
  ~NavigatorAction();

protected:
  void done() override;
  void iterate(const ros::TimerEvent& event) override;

private:
  actionlib::SimpleActionServer<project11_navigation::RunTasksAction> action_server_;

  void goalCallback();
  void preemptCallback();

  ros::Time last_feedback_send_time_;
};

} // namespace project11_navigation

#endif

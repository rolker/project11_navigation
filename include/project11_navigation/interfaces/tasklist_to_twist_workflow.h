#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASKLIST_TO_TWIST_WORKFLOW_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASKLIST_TO_TWIST_WORKFLOW_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <geometry_msgs/TwistStamped.h>

namespace project11_navigation
{

class TaskListToTwistWorkflow: public Workflow<std::shared_ptr<TaskList>, geometry_msgs::TwistStamped>
{
};

}  // namespace project11_navigation

#endif

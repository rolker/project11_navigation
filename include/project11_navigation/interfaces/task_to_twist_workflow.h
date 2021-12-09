#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASK_TO_TWIST_WORKFLOW_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASK_TO_TWIST_WORKFLOW_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <geometry_msgs/TwistStamped.h>

namespace project11_navigation
{

class TaskToTwistWorkflow: public Workflow<std::shared_ptr<Task>, geometry_msgs::TwistStamped>
{
};

}  // namespace project11_navigation

#endif

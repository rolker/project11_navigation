#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASK_TO_CURVED_TRAJECTORY_WORKFLOW_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASK_TO_CURVED_TRAJECTORY_WORKFLOW_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>
#include <project11_nav_msgs/CurvedTrajectory.h>

namespace project11_navigation
{

class TaskToTwistWorkflow: public Workflow<std::shared_ptr<Task>, project11_nav_msgs::CurvedTrajectory>
{
};

}  // namespace project11_navigation

#endif

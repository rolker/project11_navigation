#ifndef PROJECT11_NAVIGATION_WORKFLOWS_CURVED_TRAJECTORY_TO_TWIST_WORKFLOW_H
#define PROJECT11_NAVIGATION_WORKFLOWS_CURVED_TRAJECTORY_TO_TWIST_WORKFLOW_H

#include <project11_navigation/workflow.h>
#include <project11_nav_msgs/CurvedTrajectory.h>
#include <geometry_msgs/TwistStamped.h>

namespace project11_navigation
{

class TaskToTwistWorkflow: public Workflow<project11_nav_msgs::CurvedTrajectory, geometry_msgs::TwistStamped>
{
};

}  // namespace project11_navigation

#endif

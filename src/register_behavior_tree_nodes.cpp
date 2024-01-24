#include <project11_navigation/actions/baxevani_controller.h>
#include <project11_navigation/actions/crabbing_path_follower.h>
#include <project11_navigation/actions/debug_blackboard.h>
#include <project11_navigation/actions/generate_plan.h>
#include <project11_navigation/actions/get_task_data_double.h>
#include <project11_navigation/actions/get_task_data_string.h>
#include <project11_navigation/actions/get_sub_tasks.h>
#include <project11_navigation/actions/navigator_settings_loader.h>
#include <project11_navigation/actions/robot_capabilities_loader.h>
#include <project11_navigation/actions/set_pose_from_task.h>
#include <project11_navigation/actions/set_task_done.h>
#include <project11_navigation/actions/set_trajectory_from_task.h>
#include <project11_navigation/actions/task_list_updater.h>
#include <project11_navigation/actions/update_current_segment.h>
#include <project11_navigation/actions/update_current_task.h>
#include <project11_navigation/actions/update_state.h>
#include <project11_navigation/actions/visualize_trajectory.h>

#include <project11_navigation/conditions/all_tasks_done.h>
#include <project11_navigation/conditions/goal_reached.h>
#include <project11_navigation/conditions/plan_needed.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<project11_navigation::BaxevaniController>("BaxevaniController");
  factory.registerNodeType<project11_navigation::CrabbingPathFollower>("CrabbingPathFollower");
  factory.registerNodeType<project11_navigation::DebugBlackboard>("DebugBlackboard");
  factory.registerNodeType<project11_navigation::GeneratePlan>("GeneratePlan");
  factory.registerNodeType<project11_navigation::GetTaskDataDouble>("GetTaskDataDouble");
  factory.registerNodeType<project11_navigation::GetTaskDataString>("GetTaskDataString");
  factory.registerNodeType<project11_navigation::GetSubTasks>("GetSubTasks");
  factory.registerNodeType<project11_navigation::NavigatorSettingsLoader>("NavigatorSettingsLoader");
  factory.registerNodeType<project11_navigation::RobotCapabilitiesLoader>("RobotCapabilitiesLoader");
  factory.registerNodeType<project11_navigation::SetPoseFromTask>("SetPoseFromTask");
  factory.registerNodeType<project11_navigation::SetTaskDone>("SetTaskDone");
  factory.registerNodeType<project11_navigation::SetTrajectoryFromTask>("SetTrajectoryFromTask");
  factory.registerNodeType<project11_navigation::TaskListUpdater>("TaskListUpdater");
  factory.registerNodeType<project11_navigation::UpdateCurrentSegment>("UpdateCurrentSegment");
  factory.registerNodeType<project11_navigation::UpdateCurrentTask>("UpdateCurrentTask");
  factory.registerNodeType<project11_navigation::UpdateState>("UpdateState");
  factory.registerNodeType<project11_navigation::VisualizeTrajectory>("VisualizeTrajectory");

  factory.registerNodeType<project11_navigation::AllTasksDone>("AllTasksDoneCondition");
  factory.registerNodeType<project11_navigation::GoalReached>("GoalReachedCondition");
  factory.registerNodeType<project11_navigation::PlanNeeded>("PlanNeededCondition");
}


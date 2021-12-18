#ifndef PROJECT11_NAVIGATION_WORKFLOWS_TASKLIST_TO_TASKLIST_WORKFLOW_H
#define PROJECT11_NAVIGATION_WORKFLOWS_TASKLIST_TO_TASKLIST_WORKFLOW_H

#include <project11_navigation/workflow.h>
#include <project11_navigation/task.h>

namespace project11_navigation
{

class TaskListToTaskListWorkflow: public Workflow<std::shared_ptr<TaskList>, std::shared_ptr<TaskList> >
{
};

}  // namespace project11_navigation

#endif

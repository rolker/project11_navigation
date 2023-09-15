#include <project11_navigation/workflows/idle.h>
#include <project11_navigation/workflows/nav_core.h>
#include <project11_navigation/workflows/task_to_twist_stack.h>
#include <project11_navigation/plugins_loader.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

PLUGINLIB_EXPORT_CLASS(project11_navigation::Idle, project11_navigation::TaskToTwistWorkflow)

namespace project11_navigation
{

Idle::Idle()
{
  
}

Idle::~Idle()
{

}

void Idle::configure(std::string name, Context::Ptr context)
{
 
}

void Idle::setGoal(const boost::shared_ptr<Task>& input)
{

}

bool Idle::running()
{
  return true;
}

bool Idle::getResult(geometry_msgs::TwistStamped& output)
{
    output.twist.linear.x = std::nan("");
    output.twist.linear.y = std::nan("");
    output.twist.linear.z = std::nan("");
    return true; 
}


}  // namespace project11_navigation

#include <ros/ros.h>
#include <project11_navigation/navigator_action.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;

  project11_navigation::NavigatorAction navigator(ros::this_node::getName());
  ros::spin();

  return 0;
}


#include <ros/ros.h>
#include <project11_navigation/navigator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");
  ros::NodeHandle nh;

  project11_navigation::Navigator navigator(ros::this_node::getName());
  ros::spin();

  return 0;
}

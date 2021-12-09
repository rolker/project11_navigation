#include <project11_navigation/utilities.h>

namespace project11_navigation
{

void adjustTrajectoryForSpeed(std::vector<geometry_msgs::PoseStamped>& trajectory, double speed)
{
  double total_distance = 0.0;
  auto last = trajectory.begin();
  auto next = last;
  if(next != trajectory.end())
    next++;
  while(next != trajectory.end())
  {
    double dx = next->pose.position.x - last->pose.position.x;
    double dy = next->pose.position.y - last->pose.position.y;
    double distance = sqrt(dx*dx+dy*dy);
    total_distance += distance;
    next->header.stamp = trajectory.front().header.stamp+ros::Duration(total_distance/speed);
    last = next;
    next++;
  }

}

} // namespace project11_navigation

#include <project11_navigation/robot_capabilities.h>

namespace project11_navigation
{

double RobotCapabilities::getTurnRadiusAtSpeed(double speed) const
{
  if(!turn_radius_map.empty())
    return turn_radius_map.begin()->second;
  //TODO implement checking the turn radius map
  return 0.0;
}

}
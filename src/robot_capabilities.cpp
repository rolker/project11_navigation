#include <project11_navigation/robot_capabilities.h>

namespace project11_navigation
{

double RobotCapabilities::getTurnRadiusAtSpeed(double speed) const
{
  if(!turn_radius_map.empty())
  {
    if(turn_radius_map.size() == 1)
      return turn_radius_map.begin()->second;
    auto low = turn_radius_map.begin();
    if(speed <= low->first)
      return low->second;
    auto high = low;
    ++high;
    while (high != turn_radius_map.end() && high->first < speed)
    {
      low = high;
      ++high;
    }
    if(high == turn_radius_map.end())
      return low->second;
    if(high->first <= speed)
      return high->second;
    return low->second + ((speed-low->first)/(high->first-low->first))*(high->second-low->second);
  }
  return 0.0;
}

}
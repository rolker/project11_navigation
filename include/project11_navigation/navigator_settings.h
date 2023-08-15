#ifndef PROJECT11_NAVIGATION_NAVIGATOR_SETTINGS_H
#define PROJECT11_NAVIGATION_NAVIGATOR_SETTINGS_H

namespace project11_navigation
{

/// Motion limits of a robot
struct NavigatorSettings
{
  /// Distance in meters within which a waypoint is deemed reached
  double waypoint_reached_distance = 10.0;

  /// Distance in meters to extend the start of a survey line to allow heading to settle
  double survey_lead_in_distance = 2.0;
};

}

#endif

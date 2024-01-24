#include "project11_navigation/actions/navigator_settings_loader.h"
#include <ros/ros.h>
#include <project11_navigation/navigator_settings.h>
#include <project11_navigation/utilities.h>

namespace project11_navigation
{

NavigatorSettingsLoader::NavigatorSettingsLoader(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList NavigatorSettingsLoader::providedPorts()
{
  return {
    BT::OutputPort<double>("waypoint_reached_distance"),
    BT::OutputPort<double>("survey_lead_in_distance"),
    BT::OutputPort<double>("maximum_cross_track_error"),
  };
}

BT::NodeStatus NavigatorSettingsLoader::tick()
{
  ros::NodeHandle private_nh("~");
  NavigatorSettings nav_settings;
  nav_settings.waypoint_reached_distance = readDoubleOrIntParameter(private_nh, "waypoint_reached_distance", nav_settings.waypoint_reached_distance);

  nav_settings.survey_lead_in_distance = readDoubleOrIntParameter(private_nh, "survey_lead_in_distance", nav_settings.survey_lead_in_distance);

  double max_cross_track = readDoubleOrIntParameter(private_nh, "maximum_cross_track_error", nav_settings.waypoint_reached_distance);

  setOutput("waypoint_reached_distance", nav_settings.waypoint_reached_distance);
  setOutput("survey_lead_in_distance", nav_settings.survey_lead_in_distance);
  setOutput("maximum_cross_track_error", max_cross_track);

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

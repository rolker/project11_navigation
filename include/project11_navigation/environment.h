#ifndef PROJECT11_NAVIGATION_ENVIRONMENT_H
#define PROJECT11_NAVIGATION_ENVIRONMENT_H

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace project11_navigation
{

// Represent what is known about the environment from static data
// such as charts and perception data from sensors
class Environment
{
public:
  using Ptr = std::shared_ptr<Environment>;

  Environment();

  // Contains copies of grid maps for use where the maps shouldn't change
  // such as planners that expect a static map
  struct Snapshot
  {
    std::map<std::string, grid_map::GridMap> static_grids;
    std::map<double, std::vector<std::string> > static_grids_by_resolution;

    std::map<std::string, grid_map::GridMap> dynamic_grids;
  };

  // Generates and returns a snapshot.
  Snapshot snapshot(bool dynamic_only=false);

  std::string mapFrame();
  
private:

  struct Grid
  {
    void gridCallback(const grid_map_msgs::GridMap::ConstPtr &data);
    void subscribe(std::string topic);

    grid_map::GridMap grid_map;
    std::mutex grid_map_mutex;

    ros::Subscriber subscriber;
  };

  std::map<std::string, Grid> static_grids_;
  std::map<std::string, Grid> dynamic_grids_;
};


} // namespace project11_navigation


#endif

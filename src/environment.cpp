#include <project11_navigation/environment.h>

namespace project11_navigation
{

Environment::Environment()
{
  XmlRpc::XmlRpcValue static_grid_params;
  if(ros::param::get("~/grids/static", static_grid_params))
  {
    for(auto sg: static_grid_params)
    {
      auto topic = static_cast<std::string>(sg.second);
      ROS_INFO_STREAM("static grid: " << sg.first << " topic: " << topic);
      static_grids_[sg.first].subscribe(topic);
    }
  }
}

std::map<std::string, grid_map::GridMap> Environment::getStaticGrids()
{
  std::map<std::string, grid_map::GridMap> ret;
  for(auto& g: static_grids_)
  {
    std::lock_guard<std::mutex> lock(g.second.grid_map_mutex);
    ret[g.first] = g.second.grid_map;
  }
  return ret;
}


void Environment::Grid::subscribe(std::string topic)
{
  subscriber = ros::NodeHandle().subscribe(topic, 1, &Grid::gridCallback, this);
}

void Environment::Grid::gridCallback(const grid_map_msgs::GridMap::ConstPtr &data)
{
  std::lock_guard<std::mutex> lock(grid_map_mutex);
  grid_map::GridMapRosConverter::fromMessage(*data, grid_map);
}

} // namespace project11_navigation

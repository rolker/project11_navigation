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

Environment::Snapshot Environment::snapshot(bool dynamic_only)
{
  Snapshot ret;
  if(!dynamic_only)
  {
    for(auto& g: static_grids_)
    {
      std::lock_guard<std::mutex> lock(g.second.grid_map_mutex);
      ret.static_grids[g.first] = g.second.grid_map;
    }
    for(auto& sg: ret.static_grids)
      ret.static_grids_by_resolution[sg.second.getResolution()].push_back(sg.first);
  }
  for(auto& g: dynamic_grids_)
  {
    std::lock_guard<std::mutex> lock(g.second.grid_map_mutex);
    ret.dynamic_grids[g.first] = g.second.grid_map;
  }
  return ret;
}

std::string Environment::mapFrame()
{
  if(!static_grids_.empty())
  {
    std::lock_guard<std::mutex> lock(static_grids_.begin()->second.grid_map_mutex);
    return static_grids_.begin()->second.grid_map.getFrameId();
  }
  if(!dynamic_grids_.empty())
  {
    std::lock_guard<std::mutex> lock(dynamic_grids_.begin()->second.grid_map_mutex);
    return dynamic_grids_.begin()->second.grid_map.getFrameId();
  }
  return "";
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

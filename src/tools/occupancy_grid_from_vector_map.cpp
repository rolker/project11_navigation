#include <ros/ros.h>

#include <project11_nav_msgs/OccupancyVectorMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>

ros::Publisher output_publisher;

double resolution = 1.0;

void callback(const project11_nav_msgs::OccupancyVectorMapConstPtr& message)
{
  auto width = message->bounds.max_pt.x - message->bounds.min_pt.x;
  auto height = message->bounds.max_pt.y - message->bounds.min_pt.y;

  grid_map::Position center(message->bounds.min_pt.x+width/2.0, message->bounds.min_pt.y+height/2.0);

  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(width, height), resolution);
  map.setPosition(center);
  map.setFrameId(message->header.frame_id);
  map.setTimestamp(message->header.stamp.toNSec());
  map.add("occupancy", message->default_occupancy_probability);


  for(const auto& polygon: message->polygons)
  {
    for(int row = 0; row < map.getSize()[1]; row++)
    {
      double wy;
      wy = map.getPosition().y() - map.getLength().y()/2.0;
      wy += map.getResolution()*row;

      std::set<double> nodes;

      const auto& points = polygon.polygon.points;

      for(int i = 0; i < points.size(); i++)
      {
        auto p0 = points[i];
        auto p1 = points[0];
        if(i+1 < points.size())
          p1 = points[i+1];

        if(p0.y < wy && p1.y >= wy || p1.y < wy && p0.y >= wy)
        {
          nodes.insert(p0.x+(wy-p0.y)/(p1.y-p0.y)*(p1.x-p0.x));
        }
      }

      auto node = nodes.begin();
      while(node != nodes.end())
      {
        auto next_node = node;
        next_node++;
        if(next_node == nodes.end())
          break;

        if(*node > map.getPosition().x() + map.getLength().x()/2.0)
          break;

        if(*next_node > map.getPosition().x() - map.getLength().x()/2.0)
        {
          for(auto x = *node; x <= *next_node; x += map.getResolution())
          {
            grid_map::Index i;
            if(map.getIndex(grid_map::Position(x, wy),i))
            {
              auto existing_cost = map.at("occupancy", i);
              if(isnan(existing_cost) || polygon.occupancy_probability > existing_cost)
                map.at("occupancy", i) = polygon.occupancy_probability;
            }
          }
        }
        node = next_node;
        node++;
      }
    }
  }

  nav_msgs::OccupancyGrid occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, "occupancy", 0.0, 100.0, occupancy_grid);
  output_publisher.publish(occupancy_grid);


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_from_vector_map");
  ros::NodeHandle nh("~");

  resolution = nh.param("resolution", resolution);

  output_publisher = nh.advertise<nav_msgs::OccupancyGrid>("output", 1, true);

  auto input_subscriber = nh.subscribe("input", 1, &callback);

  ros::spin();

  return 0;
}



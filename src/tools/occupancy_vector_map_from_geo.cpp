#include <ros/ros.h>

#include <project11_nav_msgs/GeoOccupancyVectorMap.h>
#include <project11_nav_msgs/OccupancyVectorMap.h>

#include <project11/tf2_utils.h>

ros::Publisher output_publisher;

std::shared_ptr<project11::Transformations> transformations;
std::string frame_id = "map";

void callback(const project11_nav_msgs::GeoOccupancyVectorMapConstPtr& message)
{
  if(transformations->canTransform(frame_id, message->header.stamp))
  {
    project11_nav_msgs::OccupancyVectorMap output;
    output.header.frame_id = frame_id;
    output.header.seq = message->header.seq;
    output.header.stamp = message->header.stamp;

    output.default_occupancy_probability = message->default_occupancy_probability;

    output.bounds.min_pt = transformations->wgs84_to_map(message->bounds.min_pt, frame_id, message->header.stamp);
    output.bounds.max_pt = transformations->wgs84_to_map(message->bounds.max_pt, frame_id, message->header.stamp);

    for(const auto& geo_polygon: message->polygons)
    {
      project11_nav_msgs::OccupancyPolygon polygon;
      polygon.occupancy_probability = geo_polygon.occupancy_probability;
      for(const auto& gp: geo_polygon.polygon.points)
      { 
        auto p = transformations->wgs84_to_map(gp, frame_id, message->header.stamp);
        geometry_msgs::Point32 p32;
        p32.x = p.x;
        p32.y = p.y;
        p32.z = p.z;
        polygon.polygon.points.push_back(p32);
      }
      output.polygons.push_back(polygon);
    }

    output_publisher.publish(output);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_vector_map_from_geo");
  ros::NodeHandle nh("~");

  frame_id = nh.param("frame_id", frame_id);
  
  transformations = std::make_shared<project11::Transformations>();

  output_publisher = nh.advertise<project11_nav_msgs::OccupancyVectorMap>("output", 1, true);

  auto input_subscriber = nh.subscribe("input", 1, &callback);

  ros::spin();

  return 0;
}

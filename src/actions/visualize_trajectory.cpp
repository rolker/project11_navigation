#include <project11_navigation/actions/visualize_trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

#include <project11_navigation/bt_types.h>

namespace project11_navigation
{

VisualizeTrajectory::VisualizeTrajectory(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList VisualizeTrajectory::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("trajectory"),
    BT::InputPort<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array"),
    BT::InputPort<std::string>("namespace"),
    BT::InputPort<geometry_msgs::PoseStamped>("current_pose"),
    BT::InputPort<double>("scale"),
    BT::InputPort<std_msgs::ColorRGBA>("past_color"),
    BT::InputPort<std_msgs::ColorRGBA>("current_color"),
    BT::InputPort<std_msgs::ColorRGBA>("future_color"),
    BT::InputPort<int>("current_segment"),
  };
}

BT::NodeStatus VisualizeTrajectory::tick()
{
  auto trajectory = getInput<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("trajectory");
  if(!trajectory)
  {
    throw BT::RuntimeError(name(), " missing required input [trajectory]: ", trajectory.error() );
  }
  if(!trajectory.value())
  {
    ROS_WARN_STREAM("VisualizeTrajectory node named " << name() << " [trajectory] is null");
    return BT::NodeStatus::FAILURE;
  }

  auto marker_array = getInput<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array");
  if(!marker_array)
  {
    ROS_WARN_STREAM("VisualizeTrajectory node named " << name() << " missing [marker_array]");
    return BT::NodeStatus::FAILURE;
  }
  if(!marker_array.value())
  {
    ROS_WARN_STREAM("VisualizeTrajectory node named " << name() << " [marker_array] is null");
    return BT::NodeStatus::FAILURE;
  }

  auto current_pose = getInput<geometry_msgs::PoseStamped>("current_pose");
  if(!current_pose)
  {
    ROS_WARN_STREAM("VisualizeTrajectory node named " << name() << " missing [current_pose]");
    return BT::NodeStatus::FAILURE;
  }

  auto current_segment = getInput<int>("current_segment");
  if(!current_segment)
  {
    throw BT::RuntimeError(name(), " missing required input [current_segment]: ", current_segment.error() );
  }

  auto ns = getInput<std::string>("namespace");

  auto scale = getInput<double>("scale");

  std::vector<std::string> color_labels = {
    "past_color", "current_color", "future_color",
  };

  std::vector<BT::Expected<std_msgs::ColorRGBA> > colors;
  for(auto label: color_labels)
    colors.push_back(getInput<std_msgs::ColorRGBA>(label));

  if(!trajectory.value()->empty())
  {
    const auto& poses = *trajectory.value();

    std::vector<visualization_msgs::Marker> markers(3);
    for(int i = 0; i < markers.size(); i++)
    {
      markers[i].header.frame_id = poses.front().header.frame_id;
      markers[i].header.stamp = current_pose.value().header.stamp;
      markers[i].id = i;
      markers[i].ns = ns.value();
      markers[i].action = visualization_msgs::Marker::ADD;
      markers[i].type = visualization_msgs::Marker::LINE_STRIP;
      markers[i].pose.orientation.w = 1.0;
      markers[i].color = colors[i].value();
      markers[i].scale.x = scale.value();
      markers[i].lifetime = ros::Duration(2.0);

    }

    int markers_index = 0; // start with past
    for(int i = 0; i < poses.size()-1; i++)
    {
      // still working on past markers?
      if(markers_index == 0)
      {
        // did we reach the current segment?
        if(i == current_segment.value())
        {
          // add the final point if necessary
          if(!markers[0].points.empty())
            markers[0].points.push_back(poses[i].pose.position);
          // add current segment
          markers[1].points.push_back(poses[i].pose.position);
          markers[1].points.push_back(poses[i+1].pose.position);
          // rest will be assigned to future
          markers_index = 2; 
          continue; // so we don't add to future on this iteration
        }
      }
      markers[markers_index].points.push_back(poses[i].pose.position);
    }
    // add last position to complete the segment, if necessary
    if(!markers[markers_index].points.empty())
      markers[markers_index].points.push_back(poses.back().pose.position);

    for(const auto& marker: markers)
      marker_array.value()->markers.push_back(marker);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace project11_navigation

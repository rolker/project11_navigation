#include <project11_navigation/actions/hover.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include "project11/utils.h"

namespace project11_navigation
{

Hover::Hover(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList Hover::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::Odometry>("odometry"),
    BT::OutputPort<geometry_msgs::TwistStamped>("command_velocity"),
    BT::InputPort<double>("minimum_distance"),
    BT::InputPort<double>("maximum_distance"),
    BT::InputPort<double>("maximum_speed"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::InputPort<geometry_msgs::PoseStamped>("goal_pose"),
    BT::InputPort<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array"),

  };
}

BT::NodeStatus Hover::tick()
{
  auto odom = getInput<nav_msgs::Odometry>("odometry");
  if(!odom)
  {
    throw BT::RuntimeError(name(), " missing required input [odometry]: ", odom.error() );
  }

  auto goal_pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    throw BT::RuntimeError(name(), " missing required input [goal_pose]: ", goal_pose.error() );
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer)
  {
    throw BT::RuntimeError(name(), " missing required input [tf_buffer]: ", tf_buffer.error() );
  }

  auto minimum_distance = getInput<double>("minimum_distance");
  if(!minimum_distance)
  {
    throw BT::RuntimeError(name(), " missing required input [minimum_distance]: ", minimum_distance.error() );
  }

  auto maximum_distance = getInput<double>("maximum_distance");
  if(!maximum_distance)
  {
    throw BT::RuntimeError(name(), " missing required input [maximum_distance]: ", maximum_distance.error() );
  }

  auto maximum_speed = getInput<double>("maximum_speed");
  if(!maximum_speed)
  {
    throw BT::RuntimeError(name(), " missing required input [maximum_speed]: ", maximum_speed.error() );
  }

  try
  {
    auto t =  tf_buffer.value()->lookupTransform(odom.value().child_frame_id, goal_pose.value().header.frame_id, ros::Time(0));
        
    geometry_msgs::Point target_local;
    tf2::doTransform(goal_pose.value().pose.position, target_local,t);

    auto current_range = sqrt(target_local.x*target_local.x+target_local.y*target_local.y);

    if(current_range > maximum_distance.value())
      return BT::NodeStatus::FAILURE;

    project11::AngleRadiansZeroCentered current_bearing = atan2(target_local.y, target_local.x);

    auto current_target_speed = 0.0;

    double turn_effort = current_range/maximum_distance.value();
    turn_effort = std::min(0.5, turn_effort);

    if (current_range >= maximum_distance.value())
      current_target_speed = maximum_speed.value();
    else if (current_range > minimum_distance.value())
    {
      float p = (current_range-minimum_distance.value())/(maximum_distance.value()-minimum_distance.value());
      current_target_speed = p*maximum_speed.value();
    }
    else
    {
      // in the zero speed zone so don't turn towards target if it's behind us
      if(abs(current_bearing.value()) > M_PI/2.0)
        current_bearing += M_PI;
      turn_effort*= 0.2;
    }
    

    ROS_DEBUG_STREAM("bearing: " << current_bearing.value() << " range: " << current_range << " target speed: " << current_target_speed << " max speed: " << maximum_speed.value());

    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = odom.value().header.stamp;
    cmd_vel.header.frame_id = odom.value().child_frame_id;

    cmd_vel.twist.angular.z = current_bearing.value()*turn_effort;
    cmd_vel.twist.linear.x = current_target_speed;
    setOutput("command_velocity", cmd_vel);

    auto marker_array = getInput<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array");
    if(marker_array && marker_array.value())
    {
      const auto& target = goal_pose.value();
      visualization_msgs::Marker marker;
      marker.header.frame_id = target.header.frame_id;
      marker.header.stamp = odom.value().header.stamp;
      marker.id = 0;
      marker.ns = "hover";
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position =  target.pose.position;
      marker.pose.orientation.w = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = .75;
      marker.scale.x =  2.0*maximum_distance.value();
      marker.scale.y = 2.0*maximum_distance.value();
      marker.scale.z = 0.01;
      marker.lifetime = ros::Duration(2.0);
      marker_array.value()->markers.push_back(marker);

      visualization_msgs::Marker inmarker;
      inmarker.header.frame_id = target.header.frame_id;
      inmarker.header.stamp = odom.value().header.stamp;
      inmarker.id = 1;
      inmarker.ns = "hover";
      inmarker.action = visualization_msgs::Marker::ADD;
      inmarker.type = visualization_msgs::Marker::SPHERE;
      inmarker.pose.position =  target.pose.position;
      inmarker.pose.orientation.w = 1.0;
      inmarker.color.r = 0.0;
      inmarker.color.g = 1.0;
      inmarker.color.b = 0.2;
      inmarker.color.a = .75;
      inmarker.scale.x =  2.0*minimum_distance.value();
      inmarker.scale.y = 2.0*minimum_distance.value();
      inmarker.scale.z = 0.01;
      inmarker.lifetime = ros::Duration(2.0);
      marker_array.value()->markers.push_back(inmarker);

    }


    return BT::NodeStatus::SUCCESS;
  }
  catch (tf2::TransformException &ex)
  {
    throw BT::RuntimeError(name(), " transform exception, odom child frame: ", odom.value().child_frame_id, " goal frame: ", goal_pose.value().header.frame_id, " ex: ", ex.what());
  }
  return BT::NodeStatus::FAILURE;
}


} // project11_navigation

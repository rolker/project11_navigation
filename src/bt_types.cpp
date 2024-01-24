#include <project11_navigation/bt_types.h>
#include <tf2/utils.h>
#include <cmath>

namespace project11_navigation
{

void registerJsonDefinitions()
{
  BT::RegisterJsonDefinition<std_msgs::ColorRGBA>(ColorRGBAToJson);
  BT::RegisterJsonDefinition<geometry_msgs::TwistStamped>(TwistStampedToJson);
  BT::RegisterJsonDefinition<geometry_msgs::PoseStamped>(PoseStampedToJson);
  BT::RegisterJsonDefinition<std::shared_ptr<Task> >(TaskPtrToJson);
}

void ColorRGBAToJson(nlohmann::json& dest, const std_msgs::ColorRGBA& color)
{
  dest["r"] = color.r;
  dest["g"] = color.g;
  dest["b"] = color.b;
  dest["a"] = color.a;
}


void TwistStampedToJson(nlohmann::json& dest, const geometry_msgs::TwistStamped& twist)
{
  dest["frame_id"] = twist.header.frame_id;
  dest["timestamp"] = fmod(twist.header.stamp.toSec(), 3600.0);
  dest["linear_x"] = twist.twist.linear.x;
  dest["linear_y"] = twist.twist.linear.y;
  dest["linear_z"] = twist.twist.linear.z;
  dest["angular_x"] = twist.twist.angular.x;
  dest["angular_y"] = twist.twist.angular.y;
  dest["angular_z"] = twist.twist.angular.z;
}

void PoseStampedToJson(nlohmann::json& dest, const geometry_msgs::PoseStamped& pose)
{
  dest["frame_id"] = pose.header.frame_id;
  dest["timestamp"] = fmod(pose.header.stamp.toSec(), 3600.0);
  dest["position_x"] = pose.pose.position.x;
  dest["position_y"] = pose.pose.position.y;
  dest["position_z"] = pose.pose.position.z;
  dest["yaw"] = tf2::getYaw(pose.pose.orientation);

}

void TaskPtrToJson(nlohmann::json& dest, const std::shared_ptr<Task>& task)
{
  if(!task)
    dest["task"] = "null";
  else
  {
    dest["id"] = task->message().id;
    dest["type"] = task->message().type;
    dest["done"] = task->done();
    dest["status"] = task->message().status;
  }
}

} // namespace project11_navigation

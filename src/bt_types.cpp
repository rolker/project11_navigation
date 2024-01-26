#include <project11_navigation/bt_types.h>
#include <tf2/utils.h>
#include <cmath>
#include <ctime>

namespace project11_navigation
{

void registerJsonDefinitions()
{
  BT::RegisterJsonDefinition<std_msgs::ColorRGBA>(ColorRGBAToJson);
  BT::RegisterJsonDefinition<std_msgs::Header>(HeaderToJson);
  BT::RegisterJsonDefinition<ros::Time>(TimeToJson);

  BT::RegisterJsonDefinition<geometry_msgs::Accel>(AccelToJson);
  BT::RegisterJsonDefinition<geometry_msgs::Point>(PointToJson);
  BT::RegisterJsonDefinition<geometry_msgs::Pose>(PoseToJson);
  BT::RegisterJsonDefinition<geometry_msgs::PoseStamped>(PoseStampedToJson);
  BT::RegisterJsonDefinition<geometry_msgs::Quaternion>(QuaternionToJson);
  BT::RegisterJsonDefinition<geometry_msgs::Twist>(TwistToJson);
  BT::RegisterJsonDefinition<geometry_msgs::TwistStamped>(TwistStampedToJson);
  BT::RegisterJsonDefinition<geometry_msgs::Vector3>(Vector3ToJson);

  BT::RegisterJsonDefinition<std::shared_ptr<Task> >(TaskPtrToJson);
}

// std_msgs

void ColorRGBAToJson(nlohmann::json& dest, const std_msgs::ColorRGBA& color)
{
  dest["r"] = color.r;
  dest["g"] = color.g;
  dest["b"] = color.b;
  dest["a"] = color.a;
}

void HeaderToJson(nlohmann::json& dest, const std_msgs::Header& header)
{
  dest["frame_id"] = header.frame_id;
  TimeToJson(dest["stamp"], header.stamp);
}

void TimeToJson(nlohmann::json& dest, const ros::Time& time)
{
  dest["seconds"] = time.toSec();
  std::time_t total_seconds = static_cast<time_t>(time.sec);
  auto tm = std::gmtime(&total_seconds);
  char timeString[std::size("yyyy-mm-ddThh:mm:ss")];
  std::strftime(std::data(timeString), std::size(timeString), "%FT%T", tm);
  std::stringstream datetime;
  datetime << timeString << "." << std::setw(9) << std::setfill('0') << time.nsec;
  dest["datetime"] = datetime.str();
}

// geometry_msgs

void AccelToJson(nlohmann::json& dest, const geometry_msgs::Accel& accel)
{
  Vector3ToJson(dest["linear"], accel.linear);
  Vector3ToJson(dest["angular"], accel.angular);
}

void PointToJson(nlohmann::json& dest, const geometry_msgs::Point& point)
{
  XYZToJson(dest, point);
}

void PoseToJson(nlohmann::json& dest, const geometry_msgs::Pose& pose)
{
  PointToJson(dest["position"], pose.position);
  QuaternionToJson(dest["orientation"], pose.orientation);
}

void PoseStampedToJson(nlohmann::json& dest, const geometry_msgs::PoseStamped& pose)
{
  StampedToJson(dest, pose);
  PoseToJson(dest, pose.pose);
}

void QuaternionToJson(nlohmann::json& dest, const geometry_msgs::Quaternion& quaternion)
{
  double yaw, pitch, roll;
  tf2::getEulerYPR(quaternion, yaw, pitch, roll);
  dest["yaw"] = yaw;
  dest["pitch"] = pitch;
  dest["roll"] = roll;
}

void TwistToJson(nlohmann::json& dest, const geometry_msgs::Twist& twist)
{
  Vector3ToJson(dest["linear"], twist.linear);
  Vector3ToJson(dest["angular"], twist.angular);
}

void TwistStampedToJson(nlohmann::json& dest, const geometry_msgs::TwistStamped& twist)
{
  StampedToJson(dest, twist);
  TwistToJson(dest, twist.twist);
}

void Vector3ToJson(nlohmann::json& dest, const geometry_msgs::Vector3& vector)
{
  XYZToJson(dest, vector);
}


// project11_navigation

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

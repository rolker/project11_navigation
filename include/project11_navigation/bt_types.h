#ifndef PROJECT11_NAVIGATION_BT_TYPES_H
#define PROJECT11_NAVIGATION_BT_TYPES_H

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/json_export.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <project11_navigation/task.h>

namespace BT
{
  template <> inline std_msgs::ColorRGBA convertFromString(StringView str)
  {
    auto parts = splitString(str, ',');
    if(parts.size() != 4)
    {
      throw RuntimeError("Invalid input, expecting 4 color values (r, g, b, a)");
    }
    std_msgs::ColorRGBA color;
    color.r = convertFromString<double>(parts[0]);
    color.g = convertFromString<double>(parts[1]);
    color.b = convertFromString<double>(parts[2]);
    color.a = convertFromString<double>(parts[3]);

    return color;
  }
} // end namespace BT


namespace project11_navigation
{

void registerJsonDefinitions();

void ColorRGBAToJson(nlohmann::json& dest, const std_msgs::ColorRGBA& color);

void TwistStampedToJson(nlohmann::json& dest, const geometry_msgs::TwistStamped& twist);
void PoseStampedToJson(nlohmann::json& dest, const geometry_msgs::PoseStamped& pose);

void TaskPtrToJson(nlohmann::json& dest, const std::shared_ptr<Task>& task);


} // namespace project11_navigation

#endif


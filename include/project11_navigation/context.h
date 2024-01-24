#ifndef PROJECT11_NAVIGATION_CONTEXT_H
#define PROJECT11_NAVIGATION_CONTEXT_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <project11_navigation/environment.h>
#include <project11_navigation/robot.h>
#include <mutex>

namespace project11_navigation
{

// Assembles the relevant data for accomplishing navigation tasks.
class Context
{
public:
  using Ptr = std::shared_ptr<Context>;
  using ConstPtr = std::shared_ptr<const Context>;

  Context();

  const Environment& environment() const;
  const Robot& robot() const;
  Robot& robot();

  std::shared_ptr<tf2_ros::Buffer> tfBuffer() const;
  geometry_msgs::PoseStamped getPoseInFrame(std::string frame_id);
private:
  Environment environment_;
  Robot robot_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace project11_navigation

#endif

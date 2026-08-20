#pragma once

// RESPLE.cpp includes this but never names nav_msgs::msg::Odometry in the
// RESPLE class. Present for include compatibility only.

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_msgs {
namespace msg {

struct Odometry {
  geometry_msgs::msg::Header header;
  geometry_msgs::msg::Pose pose;
};

}  // namespace msg
}  // namespace nav_msgs

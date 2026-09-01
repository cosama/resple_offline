#pragma once

// Satisfy the upstream include.

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_msgs {
namespace msg {

struct Odometry {
  geometry_msgs::msg::Header header;
  geometry_msgs::msg::Pose pose;
};

}  // namespace msg
}  // namespace nav_msgs

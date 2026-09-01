#pragma once

// Satisfy the upstream include.

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_msgs {
namespace msg {

struct Path {
  geometry_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

}  // namespace msg
}  // namespace nav_msgs

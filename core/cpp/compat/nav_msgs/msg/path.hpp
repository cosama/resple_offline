#pragma once

// RESPLE.cpp includes this but never names nav_msgs::msg::Path in the RESPLE
// class. Present for include compatibility only.

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

#pragma once

#include "rclcpp/time.hpp"

namespace geometry_msgs {
namespace msg {

struct Point {
  double x = 0, y = 0, z = 0;
};

struct Quaternion {
  double x = 0, y = 0, z = 0, w = 1;
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct Header {
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
};

struct PoseStamped {
  Header header;
  Pose pose;
};

}  // namespace msg
}  // namespace geometry_msgs

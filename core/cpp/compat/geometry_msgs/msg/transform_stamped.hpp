#pragma once

#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace geometry_msgs {
namespace msg {

struct Vector3 {
  double x = 0, y = 0, z = 0;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped {
  Header header;
  std::string child_frame_id;
  Transform transform;
};

}  // namespace msg
}  // namespace geometry_msgs

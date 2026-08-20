#pragma once

// RESPLE.cpp constructs a TransformBroadcaster but never calls sendTransform
// in the code path this bridge compiles (RESPLE class only, not Mapping.cpp).
// No-op: broadcasting a tf frame has no offline consumer.

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tf2_ros {

class TransformBroadcaster {
 public:
  explicit TransformBroadcaster(const rclcpp::Node::SharedPtr&) {}
  void sendTransform(const geometry_msgs::msg::TransformStamped&) {}
};

}  // namespace tf2_ros

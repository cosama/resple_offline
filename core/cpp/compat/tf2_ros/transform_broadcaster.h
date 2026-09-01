#pragma once

// Provide the unused broadcaster.

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

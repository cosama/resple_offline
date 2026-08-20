#pragma once

#include <string>

#include "rclcpp/time.hpp"

namespace sensor_msgs {
namespace msg {

struct Header {
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
};

}  // namespace msg
}  // namespace sensor_msgs

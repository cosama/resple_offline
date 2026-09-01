#pragma once

// Alias the Avia message.

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/time.hpp"

namespace livox_interfaces {
namespace msg {

struct CustomPoint {
  std::uint32_t offset_time = 0;
  float x = 0, y = 0, z = 0;
  std::uint8_t reflectivity = 0;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

struct Header {
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
};

struct CustomMsg {
  using SharedPtr = std::shared_ptr<CustomMsg>;
  Header header;
  std::uint32_t point_num = 0;
  std::vector<CustomPoint> points;
};

}  // namespace msg
}  // namespace livox_interfaces

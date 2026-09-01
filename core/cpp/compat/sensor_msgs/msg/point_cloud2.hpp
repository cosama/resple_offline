#pragma once

// Carry captured point clouds.

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/header.hpp"

namespace sensor_msgs {
namespace msg {

struct PointField {
  std::string name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
};

struct PointCloud2 {
  using SharedPtr = std::shared_ptr<PointCloud2>;
  Header header;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  std::vector<std::uint8_t> data;
  bool is_dense = true;
};

}  // namespace msg
}  // namespace sensor_msgs

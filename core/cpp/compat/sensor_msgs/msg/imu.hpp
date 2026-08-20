#pragma once

#include <memory>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/header.hpp"

namespace sensor_msgs {
namespace msg {

struct Vector3 {
  double x = 0, y = 0, z = 0;
};

struct Imu {
  using SharedPtr = std::shared_ptr<Imu>;
  Header header;
  Vector3 linear_acceleration;
  Vector3 angular_velocity;
};

}  // namespace msg
}  // namespace sensor_msgs

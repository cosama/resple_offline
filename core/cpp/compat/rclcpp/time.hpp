#pragma once

// Store nanosecond timestamps.

#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"

// Match the global clock type.
enum rcl_clock_type_t { RCL_ROS_TIME = 0, RCL_SYSTEM_TIME = 1, RCL_STEADY_TIME = 2 };

namespace rclcpp {

class Duration {
 public:
  explicit Duration(double seconds) : nanoseconds_(static_cast<std::int64_t>(seconds * 1e9)) {}
  double seconds() const { return static_cast<double>(nanoseconds_) * 1e-9; }

 private:
  std::int64_t nanoseconds_;
};

class Time {
 public:
  Time() = default;
  explicit Time(std::int64_t nanoseconds, rcl_clock_type_t = RCL_ROS_TIME)
      : nanoseconds_(nanoseconds) {}
  Time(std::int32_t sec, std::uint32_t nanosec, rcl_clock_type_t = RCL_ROS_TIME)
      : nanoseconds_(static_cast<std::int64_t>(sec) * 1000000000LL +
                     static_cast<std::int64_t>(nanosec)) {}
  explicit Time(const builtin_interfaces::msg::Time& stamp)
      : nanoseconds_(static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
                     static_cast<std::int64_t>(stamp.nanosec)) {}

  std::int64_t nanoseconds() const { return nanoseconds_; }
  double seconds() const { return static_cast<double>(nanoseconds_) * 1e-9; }

  operator builtin_interfaces::msg::Time() const {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(nanoseconds_ / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds_ % 1000000000LL);
    return stamp;
  }

  Duration operator-(const Time& other) const {
    return Duration(static_cast<double>(nanoseconds_ - other.nanoseconds_) * 1e-9);
  }

 private:
  std::int64_t nanoseconds_ = 0;
};

}  // namespace rclcpp

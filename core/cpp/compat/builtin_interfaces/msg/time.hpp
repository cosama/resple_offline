#pragma once

#include <cstdint>

namespace builtin_interfaces {
namespace msg {

struct Time {
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
};

}  // namespace msg
}  // namespace builtin_interfaces

#pragma once

#include <cstdint>
#include <vector>

#include "estimate_msgs/msg/knot.hpp"

namespace estimate_msgs {
namespace msg {

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};

struct Spline {
  std::int64_t dt = 0;
  int start_idx = 0;
  std::int64_t start_t = 0;
  Quat start_q;
  std::vector<Knot> knots;
  std::vector<Knot> idles;
};

}  // namespace msg
}  // namespace estimate_msgs

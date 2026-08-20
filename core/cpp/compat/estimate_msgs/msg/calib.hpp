#pragma once

// Included by RESPLE.cpp but never named in the RESPLE class (only Mapping.cpp
// consumes Calib). Present for include compatibility only.

#include "estimate_msgs/msg/knot.hpp"

namespace estimate_msgs {
namespace msg {

struct Calib {
  Vec3 gravity;
  Vec3 t_bl;
  struct { double x = 0, y = 0, z = 0, w = 1; } q_bl;
};

}  // namespace msg
}  // namespace estimate_msgs

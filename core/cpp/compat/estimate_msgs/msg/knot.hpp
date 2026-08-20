#pragma once

namespace estimate_msgs {
namespace msg {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

struct Knot {
  Vec3 position;
  Vec3 orientation_del;
};

}  // namespace msg
}  // namespace estimate_msgs

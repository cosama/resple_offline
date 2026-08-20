#pragma once

#include "estimate_msgs/msg/spline.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"

namespace estimate_msgs {
namespace msg {

struct Estimate {
  Spline spline;
  std_msgs::msg::Bool if_full_window;
  std_msgs::msg::Int64 runtime;
};

}  // namespace msg
}  // namespace estimate_msgs

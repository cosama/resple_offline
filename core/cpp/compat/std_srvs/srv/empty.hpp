#pragma once

// RESPLE.cpp includes <std_srvs/srv/empty.hpp> and <rclcpp/service.hpp> but
// registers no service in the offline-relevant (RESPLE class) code path.
// Present for include compatibility only.

namespace std_srvs {
namespace srv {

struct Empty {
  struct Request {};
  struct Response {};
};

}  // namespace srv
}  // namespace std_srvs

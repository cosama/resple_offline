#pragma once

// Minimal replacement for rclcpp/rclcpp.hpp.
//
// RESPLE.cpp (staged, patched only for member visibility -- see
// patches/integration/01-bridge-visibility.patch) compiles against this
// shim unmodified: rclcpp::Node's constructor/param/pub/sub surface stays
// structurally the same as real rclcpp, but:
//
//   - create_subscription() is a no-op placeholder. This bridge never drives
//     sensor data through RESPLE's own ROS callbacks -- it injects directly
//     into RESPLE's per-lidar buffers and IMU queue instead (see
//     cpp/bindings.cpp), which are exactly the buffers those callbacks would
//     have filled. That keeps RESPLE's own downsampling/PointData
//     construction/estimator loop (processData) genuinely unmodified.
//   - create_publisher() returns a Publisher<T> whose publish() forwards to
//     a bridge-registered capture callback (Publisher<T>::capture_slot()),
//     which is how this bridge observes committed poses/spline state and
//     scan/map updates -- the natural hook points ARCHITECTURE.md calls for,
//     realized entirely in this compat layer rather than by patching
//     RESPLE.cpp's publish call sites.
//   - parameters are a simple typed store the bridge pre-populates
//     (Node::set_parameter) before RESPLE's constructor runs, tracking which
//     keys were bridge-provided vs. fell back to a declared default, for
//     parameter-audit reporting.
//   - Rate::sleep() is a short fixed poll, not real hz-paced sleeping: this
//     bridge wants processData's wait loop to spin fast, not at wall-clock
//     20 Hz, so offline replay is not throttled to real time.

#include <any>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/time.hpp"

namespace rclcpp {

struct Logger {
  std::string name;
};

template <typename MsgT>
class Subscription {
 public:
  using SharedPtr = std::shared_ptr<Subscription<MsgT>>;
};

template <typename MsgT>
class Publisher {
 public:
  using SharedPtr = std::shared_ptr<Publisher<MsgT>>;

  void publish(const MsgT& message) {
    auto& slot = capture_slot();
    if (slot) slot(message);
  }

  // One slot per message type: RESPLE.cpp never publishes more than one
  // topic of a given message type, so this is unambiguous. Reset between
  // sessions by the bridge (see cpp/bindings.cpp).
  static std::function<void(const MsgT&)>& capture_slot() {
    static std::function<void(const MsgT&)> slot;
    return slot;
  }
};

class Node {
 public:
  using SharedPtr = std::shared_ptr<Node>;

  static SharedPtr make_shared(const std::string& name) {
    return std::make_shared<Node>(name);
  }

  explicit Node(std::string name) : name_(std::move(name)) {}

  // --- Bridge-facing: seed real values before RESPLE's constructor runs ---
  template <typename T>
  void set_parameter(const std::string& key, const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_[key] = value;
    provided_.insert(key);
  }

  bool was_provided(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return provided_.count(key) != 0;
  }
  bool was_read(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return read_.count(key) != 0;
  }
  std::set<std::string> fallback_keys() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fallback_;
  }

  // --- rclcpp::Node parameter surface RESPLE.cpp/CommonUtils::readParam use ---
  bool has_parameter(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return params_.count(key) != 0;
  }

  template <typename T>
  T declare_parameter(const std::string& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!params_.count(key)) params_[key] = T{};
    return *std::any_cast<T>(&params_[key]);
  }

  template <typename T>
  T declare_parameter(const std::string& key, const T& default_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!params_.count(key)) {
      params_[key] = default_value;
      fallback_.insert(key);
    }
    return *std::any_cast<T>(&params_[key]);
  }

  template <typename T>
  bool get_parameter(const std::string& key, T& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = params_.find(key);
    if (it == params_.end()) return false;
    read_.insert(key);
    if (const T* typed = std::any_cast<T>(&it->second)) {
      out = *typed;
      return true;
    }
    return false;
  }

  template <typename T>
  bool get_parameter(std::initializer_list<std::string> keys, T& out) const {
    return get_parameter(*keys.begin(), out);
  }

  template <typename T>
  void get_parameter_or(const std::string& key, T& out, const T& alternative) const {
    if (!get_parameter(key, out)) out = alternative;
  }

  template <typename MsgT, typename CallbackT>
  typename Subscription<MsgT>::SharedPtr create_subscription(const std::string&, int, CallbackT) {
    return std::make_shared<Subscription<MsgT>>();
  }

  template <typename MsgT>
  typename Publisher<MsgT>::SharedPtr create_publisher(const std::string&, int) {
    return std::make_shared<Publisher<MsgT>>();
  }

  Logger get_logger() const { return Logger{name_}; }

 private:
  std::string name_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::any> params_;
  std::set<std::string> provided_;
  std::set<std::string> fallback_;
  mutable std::set<std::string> read_;
};

class Rate {
 public:
  explicit Rate(double) {}
  void sleep() { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
};

inline std::atomic_bool& shutdown_requested() {
  static std::atomic_bool flag{false};
  return flag;
}
inline void init(int, char**) {}
inline bool ok() { return !shutdown_requested().load(); }
inline void shutdown() { shutdown_requested().store(true); }
inline void spin_some(const Node::SharedPtr&) {}

}  // namespace rclcpp

#define RCLCPP_INFO_STREAM(logger, args)                            \
  do {                                                              \
    std::ostringstream _resple_log_oss;                             \
    _resple_log_oss << args;                                        \
    std::fprintf(stderr, "[INFO] %s\n", _resple_log_oss.str().c_str()); \
  } while (0)

#define RCLCPP_WARN_STREAM(logger, args)                            \
  do {                                                              \
    std::ostringstream _resple_log_oss;                             \
    _resple_log_oss << args;                                        \
    std::fprintf(stderr, "[WARN] %s\n", _resple_log_oss.str().c_str()); \
  } while (0)

#define RCLCPP_FATAL_STREAM(logger, args)                           \
  do {                                                              \
    std::ostringstream _resple_log_oss;                             \
    _resple_log_oss << args;                                        \
    std::fprintf(stderr, "[FATAL] %s\n", _resple_log_oss.str().c_str()); \
  } while (0)

#pragma once

// Minimal rclcpp surface used by the offline bridge. Subscriptions are no-ops,
// publishers forward to capture callbacks, and Node stores typed parameters.
// Rate uses a short polling delay so replay is not throttled to wall-clock time.

#include <any>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/time.hpp"

namespace rclcpp {

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

  // RESPLE publishes at most one topic of each captured message type.
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

  explicit Node(std::string) {}

  // --- Bridge-facing: seed real values before RESPLE's constructor runs ---
  template <typename T>
  void set_parameter(const std::string& key, const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_[key] = value;
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
    if (!params_.count(key)) params_[key] = default_value;
    return *std::any_cast<T>(&params_[key]);
  }

  template <typename T>
  bool get_parameter(const std::string& key, T& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = params_.find(key);
    if (it == params_.end()) return false;
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

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::any> params_;
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

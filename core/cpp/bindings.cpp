// Offline host for the pinned upstream RESPLE estimator.
//
// Design: RESPLE.cpp (staged + patched, see patches/integration/) is
// #included directly below, giving this translation unit access to the
// RESPLE class with its buffers made public (01-bridge-visibility.patch).
// Sensor data is injected directly into the exact same per-lidar/IMU buffers
// RESPLE's own ROS callbacks would have filled -- ousterLidarCallback and
// friends are compiled (their addresses are taken in RESPLE's constructor)
// but never invoked, so RESPLE's own downsample/PointData construction and
// processData() estimator loop run completely unmodified. Publishers become
// hooks via the compat rclcpp::Publisher<T>::capture_slot() mechanism (see
// compat/rclcpp/rclcpp.hpp) rather than by patching publish call sites.
//
// processData() runs on its own worker thread, exactly as upstream's main()
// starts it. finish_requested() (resple_bridge/offline_bridge.hpp, wired via
// patches/integration/02-finish-condition.patch) is how the bridge stops
// that thread deterministically instead of relying on process termination.

#include <resple_bridge/offline_bridge.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// The staged, patched upstream translation unit. See CMakeLists.txt: only
// this file and ikd-Tree/ikd_Tree.cpp are compiled -- RESPLE.cpp and
// common_utils.h define process-global state (the ikd-tree, NUM_OF_THREAD,
// NUM_MATCH_POINTS) with no extern/inline, so it must live in exactly one
// translation unit.
#include "RESPLE.cpp"

namespace py = pybind11;

namespace resple_bridge {

namespace {
std::atomic<bool>& finish_flag() {
  static std::atomic<bool> flag{false};
  return flag;
}
}  // namespace

bool finish_requested() noexcept { return finish_flag().load(std::memory_order_relaxed); }

}  // namespace resple_bridge

namespace {

using Clock = std::chrono::steady_clock;
using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;
using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

struct PoseRecord {
  double timestamp = 0.0;
  double x = 0, y = 0, z = 0;
  double qx = 0, qy = 0, qz = 0, qw = 1;
};

struct PointRecord {
  float x = 0, y = 0, z = 0, intensity = 0;
};

struct Event {
  std::string kind;
  double timestamp = 0.0;
};

struct Recorder {
  std::mutex mutex;
  std::vector<PoseRecord> trajectory;
  std::deque<std::vector<PointRecord>> map_batches;
  std::vector<Event> events;
  std::int64_t imu_samples = 0;
  std::int64_t lidar_sweeps_pushed = 0;
  std::int64_t poses_committed = 0;
  std::int64_t map_batches_emitted = 0;

  void reset() {
    std::lock_guard<std::mutex> lock(mutex);
    trajectory.clear();
    map_batches.clear();
    events.clear();
    imu_samples = 0;
    lidar_sweeps_pushed = 0;
    poses_committed = 0;
    map_batches_emitted = 0;
  }
};

Recorder& recorder() {
  static Recorder value;
  return value;
}

double ns_to_seconds(std::int64_t ns) { return static_cast<double>(ns) * 1e-9; }
std::int64_t seconds_to_ns(double seconds) {
  return static_cast<std::int64_t>(std::llround(seconds * 1e9));
}

// ---------------------------------------------------------------------------
// Typed native parameter configuration.
//
// Every RESPLE parameter is set with its exact upstream C++ type
// (CommonUtils::readParam<T> is type-strict: std::any_cast<T> fails silently
// to "not present" for a mismatched type, e.g. double where float is read).
// A generic py::dict type-sniffing loop would risk exactly that mismatch, so
// this bridge takes explicit typed arguments instead (ARCHITECTURE.md #7:
// plumb every behavior-affecting option deliberately).
// ---------------------------------------------------------------------------
void configure_lidar(rclcpp::Node::SharedPtr& node, const std::string& name,
                     const std::string& lidar_type, int scan_line, float blind,
                     const std::vector<double>& q_lb, const std::vector<double>& t_lb,
                     double w_pt) {
  const std::string prefix = name + ".";
  node->set_parameter<std::string>(prefix + "topic_lidar", "/offline/" + name);
  node->set_parameter<std::string>(prefix + "lidar_type", lidar_type);
  node->set_parameter<int>(prefix + "scan_line", scan_line);
  node->set_parameter<float>(prefix + "blind", blind);
  node->set_parameter<std::vector<double>>(prefix + "q_lb", q_lb);
  node->set_parameter<std::vector<double>>(prefix + "t_lb", t_lb);
  node->set_parameter<double>(prefix + "w_pt", w_pt);
}

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

class RespleOdometry {
 public:
  RespleOdometry(const py::dict& params, const std::string& lidar_name,
                std::string lidar_type, int scan_line, float blind,
                const std::vector<double>& q_lb, const std::vector<double>& t_lb,
                double w_pt, int max_pending_sweeps, double min_imu_lead_seconds)
      : lidar_name_(lidar_name),
        lidar_type_(lidar_type),
        max_pending_sweeps_(max_pending_sweeps),
        min_imu_lead_seconds_(min_imu_lead_seconds) {
    if (max_pending_sweeps_ < 1)
      throw py::value_error("max_pending_sweeps must be at least 1");
    if (min_imu_lead_seconds_ < 0.0)
      throw py::value_error("min_imu_lead_seconds must be >= 0");

    // The ikd-tree (RESPLE.cpp file scope) and this bridge's own recorder
    // are process-global. A second session in the same process would
    // silently inherit the first session's map, so refuse outright rather
    // than document it: run one dataset (or segment) per subprocess.
    if (consumed().exchange(true))
      throw std::runtime_error(
          "a RESPLE session has already run in this process. Upstream estimator "
          "state (the ikd-tree map) is process-global and is never reset, so "
          "each session needs a fresh process; run one dataset (or segment) per "
          "subprocess.");

    recorder().reset();
    resple_bridge::finish_flag().store(false);
    reset_publisher_captures();

    node_ = rclcpp::Node::make_shared("RESPLE");
    apply_parameters(params);
    configure_lidar(node_, lidar_name_, lidar_type, scan_line, blind, q_lb, t_lb, w_pt);
    node_->set_parameter<std::vector<std::string>>("lidars", {lidar_name_});
    node_->set_parameter<bool>("if_lidar_only", false);
    if (!node_->was_provided("topic_imu"))
      node_->set_parameter<std::string>("topic_imu", "/offline/imu");

    resple_ = std::make_unique<RESPLE>(node_);
    install_captures();  // captures resple_.get(); must run after construction
    start_worker();
  }

  ~RespleOdometry() {
    try {
      shutdown(1.0);
    } catch (...) {
      // A destructor must not propagate; already surfaced via another call.
    }
  }

  std::int64_t push_imu(double timestamp, const std::vector<double>& acceleration,
                        const std::vector<double>& angular_velocity) {
    throw_if_failed();
    if (acceleration.size() != 3 || angular_velocity.size() != 3)
      throw py::value_error("acceleration and angular_velocity must have length 3");
    for (int axis = 0; axis < 3; ++axis)
      if (!std::isfinite(acceleration[axis]) || !std::isfinite(angular_velocity[axis]))
        throw py::value_error("IMU measurements must be finite");
    if (!std::isfinite(timestamp)) throw py::value_error("IMU timestamp must be finite");
    if (timestamp <= last_imu_stamp_)
      throw py::value_error("imu timestamps must be strictly increasing");
    last_imu_stamp_ = timestamp;

    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp = rclcpp::Time(seconds_to_ns(timestamp));
    message->linear_acceleration.x = acceleration[0];
    message->linear_acceleration.y = acceleration[1];
    message->linear_acceleration.z = acceleration[2];
    message->angular_velocity.x = angular_velocity[0];
    message->angular_velocity.y = angular_velocity[1];
    message->angular_velocity.z = angular_velocity[2];

    // Exactly what RESPLE::getImuCallback does with a subscribed message.
    {
      std::lock_guard<std::mutex> lock(resple_->m_buff);
      resple_->imu_int_buff.push_back(message);
    }

    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.imu_samples += 1;
    return state.imu_samples;
  }

  std::int64_t push_lidar(double timestamp, const FloatArray& points,
                          const DoubleArray& relative_times, double blind) {
    throw_if_failed();
    validate_sweep(timestamp, points, relative_times);
    apply_backpressure();
    throw_if_failed();

    if (last_imu_stamp_ < timestamp + min_imu_lead_seconds_)
      throw py::value_error(
          "insufficient IMU lead for sweep at t=" + std::to_string(timestamp) +
          ": have IMU through " + std::to_string(last_imu_stamp_) +
          ", need through at least " +
          std::to_string(timestamp + min_imu_lead_seconds_) +
          " (min_imu_lead_seconds=" + std::to_string(min_imu_lead_seconds_) +
          "). Push more IMU before this sweep.");

    auto p = points.unchecked<2>();
    auto t = relative_times.unchecked<1>();
    const py::ssize_t count = points.shape(0);
    const float blind2 = blind * blind;

    // Exactly what RESPLE's sensor-specific callbacks do: build a raw
    // pcl::PointXYZINormal cloud with intensity = relative time in ms
    // (matching e.g. the Ouster path's `pt.intensity = t_ns / 1e6`), apply
    // the same blind-range filter, and push (points, sweep-start-ns) onto
    // the lidar's own buffer under its own mutex.
    Eigen::aligned_vector<pcl::PointXYZINormal> cloud;
    cloud.reserve(static_cast<std::size_t>(count));
    for (py::ssize_t i = 0; i < count; ++i) {
      const float x = p(i, 0), y = p(i, 1), z = p(i, 2);
      if (x * x + y * y + z * z <= blind2) continue;
      pcl::PointXYZINormal pt;
      pt.x = x; pt.y = y; pt.z = z;
      pt.intensity = static_cast<float>(t(i) * 1000.0);  // seconds -> ms
      pt.curvature = points.shape(1) == 4 ? p(i, 3) : 0.0f;
      cloud.push_back(pt);
    }
    if (cloud.empty()) throw py::value_error("sweep has no points after blind-range filtering");

    const std::int64_t time_begin_ns = seconds_to_ns(timestamp);
    // lidars_data/lidars are keyed by lidar *type* upstream (see e.g.
    // RESPLE::ousterLidarCallback's `lidars.at("Ouster")`), not by the
    // "lidars" list's arbitrary config-prefix name.
    RESPLE::LidarData& buffers = resple_->lidars_data.at(lidar_type_);
    {
      std::lock_guard<std::mutex> lock(buffers.mtx_pc);
      buffers.pc_buff.push_back(std::move(cloud));
      buffers.t_buff.push_back(time_begin_ns);
    }

    last_lidar_stamp_ = timestamp;
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.lidar_sweeps_pushed += 1;
    return state.lidar_sweeps_pushed;
  }

  py::dict finish(double timeout_seconds) {
    {
      py::gil_scoped_release release;
      shutdown(timeout_seconds);
    }
    throw_if_failed();
    py::dict result;
    result["trajectory"] = trajectory_array();
    result["map_batches"] = drain_map_batches();
    result["metrics"] = metrics();
    result["events"] = events();
    return result;
  }

  py::array_t<double> trajectory() {
    throw_if_failed();
    return trajectory_array();
  }

  py::list drain_map_batches() {
    throw_if_failed();
    std::deque<std::vector<PointRecord>> batch;
    {
      auto& state = recorder();
      std::lock_guard<std::mutex> lock(state.mutex);
      batch.swap(state.map_batches);
    }
    py::list result;
    for (auto& points : batch) {
      py::array_t<float> array(py::array::ShapeContainer{
          static_cast<py::ssize_t>(points.size()), py::ssize_t(4)});
      if (!points.empty())
        std::memcpy(array.mutable_data(), points.data(), points.size() * sizeof(PointRecord));
      result.append(std::move(array));
    }
    return result;
  }

  py::dict metrics() {
    throw_if_failed();
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    py::dict result;
    result["imu_samples"] = state.imu_samples;
    result["lidar_sweeps_pushed"] = state.lidar_sweeps_pushed;
    result["poses_committed"] = state.poses_committed;
    result["map_batches_emitted"] = state.map_batches_emitted;
    result["estimator_threads"] = NUM_OF_THREAD;
    return result;
  }

  py::list events() {
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    py::list result;
    for (const auto& event : state.events) {
      py::dict item;
      item["kind"] = event.kind;
      item["timestamp"] = event.timestamp;
      result.append(std::move(item));
    }
    return result;
  }

  py::dict status() {
    throw_if_failed();
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    py::dict result;
    result["poses_committed"] = state.poses_committed;
    result["lidar_sweeps_pushed"] = state.lidar_sweeps_pushed;
    result["worker_finished"] = worker_finished_.load();
    result["last_imu_stamp"] = last_imu_stamp_;
    result["last_lidar_stamp"] = last_lidar_stamp_;
    return result;
  }

 private:
  static std::atomic<bool>& consumed() {
    static std::atomic<bool> value{false};
    return value;
  }

  void apply_parameters(const py::dict& params) {
    for (auto item : params) {
      const std::string key = py::cast<std::string>(item.first);
      py::handle value = item.second;
      if (key == "topic_imu" || key == "lidar_type") {
        node_->set_parameter<std::string>(key, py::cast<std::string>(value));
      } else if (key == "acc_ratio" || key == "if_lidar_only") {
        node_->set_parameter<bool>(key, py::cast<bool>(value));
      } else if (key == "knot_hz" || key == "point_filter_num" || key == "num_nn" ||
                key == "n_iter" || key == "num_points_upd") {
        node_->set_parameter<int>(key, py::cast<int>(value));
      } else if (key == "ds_lm_voxel" || key == "ds_scan_voxel") {
        node_->set_parameter<float>(key, py::cast<float>(value));
      } else if (key == "cov_acc" || key == "cov_gyro" || key == "cov_ba" || key == "cov_bg") {
        node_->set_parameter<std::vector<double>>(key, py::cast<std::vector<double>>(value));
      } else if (py::isinstance<py::float_>(value) || py::isinstance<py::int_>(value)) {
        // nn_thresh, coeff_cov, cube_len, cov_P0, cov_RCP_*, std_sys_*,
        // lidar_time_offset -- all plain doubles upstream.
        node_->set_parameter<double>(key, py::cast<double>(value));
      } else {
        throw py::type_error("unrecognized RESPLE native parameter: " + key);
      }
    }
  }

  void reset_publisher_captures() {
    rclcpp::Publisher<estimate_msgs::msg::Estimate>::capture_slot() = nullptr;
    rclcpp::Publisher<std_msgs::msg::Int64>::capture_slot() = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::capture_slot() = nullptr;
  }

  void install_captures() {
    rclcpp::Publisher<std_msgs::msg::Int64>::capture_slot() =
        [](const std_msgs::msg::Int64& msg) {
          auto& state = recorder();
          std::lock_guard<std::mutex> lock(state.mutex);
          state.events.push_back(Event{"init", ns_to_seconds(msg.data)});
        };

    // Fires whenever the spline gains a new knot (~every 1/knot_hz seconds
    // of estimated trajectory). Sample the pose one knot interval behind the
    // leading edge, where the spline is well-constrained rather than freshly
    // extrapolated -- the same interior-of-the-window assumption
    // RESPLE::getPositionLiDAR relies on elsewhere.
    RESPLE* resple_ptr = resple_.get();  // captured by pointer; alive for session lifetime
    rclcpp::Publisher<estimate_msgs::msg::Estimate>::capture_slot() =
        [resple_ptr](const estimate_msgs::msg::Estimate&) {
          if (!resple_ptr || !resple_ptr->spline) return;
          SplineState* spline = resple_ptr->spline;
          if (spline->numKnots() < 4) return;
          const std::int64_t t_ns = spline->maxTimeNs() - spline->getKnotTimeIntervalNs();
          if (t_ns < spline->minTimeNs()) return;
          Eigen::Vector3d pos = spline->itpPosition(t_ns);
          Eigen::Quaterniond q;
          spline->itpQuaternion(t_ns, &q);
          auto& state = recorder();
          std::lock_guard<std::mutex> lock(state.mutex);
          if (!state.trajectory.empty() && state.trajectory.back().timestamp >= ns_to_seconds(t_ns))
            return;
          state.trajectory.push_back(PoseRecord{ns_to_seconds(t_ns), pos.x(), pos.y(), pos.z(),
                                                q.x(), q.y(), q.z(), q.w()});
          state.poses_committed += 1;
        };

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::capture_slot() =
        [](const sensor_msgs::msg::PointCloud2& msg) {
          pcl::PointCloud<pcl::PointXYZI> cloud;
          pcl::fromROSMsg(msg, cloud);
          std::vector<PointRecord> points(cloud.size());
          for (std::size_t i = 0; i < cloud.size(); ++i)
            points[i] = PointRecord{cloud[i].x, cloud[i].y, cloud[i].z, cloud[i].intensity};
          auto& state = recorder();
          std::lock_guard<std::mutex> lock(state.mutex);
          state.map_batches.push_back(std::move(points));
          state.map_batches_emitted += 1;
        };
  }

  void start_worker() {
    worker_ = std::thread([this] {
      try {
        resple_->processData();
      } catch (...) {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (!worker_error_) worker_error_ = std::current_exception();
      }
      worker_finished_.store(true);
      worker_progress_.notify_all();
    });
  }

  void shutdown(double timeout_seconds) {
    if (!worker_.joinable()) return;
    const auto deadline = Clock::now() + std::chrono::duration<double>(
        timeout_seconds > 0 ? timeout_seconds : 0.0);

    // processData()'s outer loop only checks finish_requested() between full
    // drain passes. Setting the flag while pushed sweeps are still queued
    // would let the worker stop mid-drain and silently discard them
    // (ARCHITECTURE.md: "internal resets do not silently discard earlier
    // usable output"). Wait for the mutex-protected raw per-lidar queue to
    // empty out, then a fixed grace period for processData's own (internal,
    // single-threaded, unsynchronized) pt_buff/collectMeasurements draining
    // of what it already popped -- pt_buff has no mutex of its own to poll
    // safely from this thread. Not watertight under extreme load; see
    // UPSTREAM.md limitations.
    while (Clock::now() < deadline && !worker_finished_.load()) {
      RESPLE::LidarData& buffers = resple_->lidars_data.at(lidar_type_);
      bool drained;
      {
        std::lock_guard<std::mutex> lock(buffers.mtx_pc);
        drained = buffers.t_buff.empty();
      }
      if (drained) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    resple_bridge::finish_flag().store(true);
    while (Clock::now() < deadline && !worker_finished_.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!worker_finished_.load())
      throw std::runtime_error("timed out waiting for RESPLE's processData() thread to stop");
    worker_.join();
  }

  void throw_if_failed() {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_error_) std::rethrow_exception(worker_error_);
  }

  void apply_backpressure() {
    // 60s was too tight for real (not synthetic) datasets where processData()
    // is legitimately busy -- not stuck -- but slower than max_pending_sweeps_
    // worth of headroom, e.g. degenerate feature association / heavier
    // ikd-tree rebuild activity on noisy/sparse point returns. Confirmed on
    // miniprism_JHU_..._uasnoisy: CPU stayed pegged at ~230-300% (multiple
    // native threads busy) for the full run, yet the queue never drained
    // below max_pending_sweeps_ within 60s at some point mid-stream. Widened
    // to match the same order of magnitude as PROCESSING_TIMEOUT_SECONDS in
    // scripts/run_resple.py (the analogous tail-flush budget, also raised
    // for the same reason on nglamp_SRNL_..._deepforest).
    const auto deadline = Clock::now() + std::chrono::seconds(600);
    while (Clock::now() < deadline) {
      // lidars_data/lidars are keyed by lidar *type* upstream (see e.g.
    // RESPLE::ousterLidarCallback's `lidars.at("Ouster")`), not by the
    // "lidars" list's arbitrary config-prefix name.
    RESPLE::LidarData& buffers = resple_->lidars_data.at(lidar_type_);
      std::size_t pending;
      {
        std::lock_guard<std::mutex> lock(buffers.mtx_pc);
        pending = buffers.t_buff.size();
      }
      if (static_cast<int>(pending) < max_pending_sweeps_) return;
      if (worker_finished_.load()) return;
      throw_if_failed();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error(
        "timed out applying lidar backpressure: processData() is not draining its queue");
  }

  void validate_sweep(double timestamp, const FloatArray& points,
                      const DoubleArray& relative_times) const {
    if (points.ndim() != 2 || (points.shape(1) != 3 && points.shape(1) != 4))
      throw py::value_error("points must have shape (N, 3) or (N, 4)");
    if (points.shape(0) == 0) throw py::value_error("sweep must contain points");
    if (relative_times.ndim() != 1 || relative_times.shape(0) != points.shape(0))
      throw py::value_error("relative_times must have shape (N,)");
    if (!std::isfinite(timestamp)) throw py::value_error("sweep timestamp must be finite");
    if (timestamp <= last_lidar_stamp_)
      throw py::value_error("sweep timestamps must be strictly increasing");

    auto p = points.unchecked<2>();
    auto t = relative_times.unchecked<1>();
    double previous = -1.0;
    for (py::ssize_t i = 0; i < points.shape(0); ++i) {
      if (!std::isfinite(t(i)) || t(i) < 0 || t(i) < previous)
        throw py::value_error(
            "relative_times must be finite, non-negative, and non-decreasing");
      previous = t(i);
      for (py::ssize_t j = 0; j < points.shape(1); ++j)
        if (!std::isfinite(p(i, j))) throw py::value_error("points must be finite");
    }
  }

  py::array_t<double> trajectory_array() {
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    py::array_t<double> out({static_cast<py::ssize_t>(state.trajectory.size()), py::ssize_t(8)});
    auto dst = out.mutable_unchecked<2>();
    for (std::size_t i = 0; i < state.trajectory.size(); ++i) {
      const auto& pr = state.trajectory[i];
      dst(i, 0) = pr.timestamp;
      dst(i, 1) = pr.x; dst(i, 2) = pr.y; dst(i, 3) = pr.z;
      dst(i, 4) = pr.qx; dst(i, 5) = pr.qy; dst(i, 6) = pr.qz; dst(i, 7) = pr.qw;
    }
    return out;
  }

  std::string lidar_name_;
  std::string lidar_type_;
  int max_pending_sweeps_;
  double min_imu_lead_seconds_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RESPLE> resple_;
  std::thread worker_;
  std::atomic<bool> worker_finished_{false};
  std::mutex worker_mutex_;
  std::condition_variable worker_progress_;
  std::exception_ptr worker_error_;
  double last_imu_stamp_ = -std::numeric_limits<double>::infinity();
  double last_lidar_stamp_ = -std::numeric_limits<double>::infinity();
};

}  // namespace

PYBIND11_MODULE(_core, module) {
  module.doc() = "Offline host for pinned upstream RESPLE";

  py::class_<RespleOdometry>(module, "RespleOdometry")
      .def(py::init<const py::dict&, const std::string&, std::string, int, float,
                    const std::vector<double>&, const std::vector<double>&, double, int,
                    double>(),
           py::arg("parameters"), py::arg("lidar_name"), py::arg("lidar_type"),
           py::arg("scan_line"), py::arg("blind"), py::arg("q_lb"), py::arg("t_lb"),
           py::arg("w_pt"), py::arg("max_pending_sweeps") = 8,
           py::arg("min_imu_lead_seconds") = 0.0)
      .def("push_imu", &RespleOdometry::push_imu, py::arg("timestamp"),
           py::arg("acceleration"), py::arg("angular_velocity"))
      .def("push_lidar", &RespleOdometry::push_lidar, py::arg("timestamp"), py::arg("points"),
           py::arg("relative_times"), py::arg("blind"))
      .def("trajectory", &RespleOdometry::trajectory)
      .def("drain_map_batches", &RespleOdometry::drain_map_batches)
      .def("finish", &RespleOdometry::finish, py::arg("timeout_seconds") = 60.0)
      .def("metrics", &RespleOdometry::metrics)
      .def("events", &RespleOdometry::events)
      .def("status", &RespleOdometry::status);
}

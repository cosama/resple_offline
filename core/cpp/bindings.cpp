// Offline host for the pinned upstream RESPLE estimator.
//
// Design: RESPLE.cpp (staged + patched, see patches/integration/) is
// #included directly below, giving this translation unit access to the
// RESPLE class with its buffers made public (01-bridge-visibility.patch).
// Canonical/preprocessed sensor data is injected at RESPLE's internal
// per-lidar/IMU cloud buffers. Sensor callbacks are compiled (their addresses
// are taken in RESPLE's constructor) but are not invoked; callback behavior
// representable by the prepared format is applied at this boundary. RESPLE's
// downstream PointData construction and estimator loop remain upstream code.
// Publishers become hooks via the compat rclcpp::Publisher<T>::capture_slot()
// mechanism (see compat/rclcpp/rclcpp.hpp) rather than by patching publish
// call sites.
//
// processData() runs on its own worker thread, exactly as upstream's main()
// starts it. Its completion condition is state-based rather than timed: the
// producer closes input, and the worker stops after the first pass that both
// *began* with input closed and moved nothing (see offline_bridge.hpp).

#include <resple_bridge/offline_bridge.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <limits>
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
std::atomic<bool>& input_closed_flag() {
  static std::atomic<bool> flag{false};
  return flag;
}
// Monotonic counter bumped by every note_progress(). The backpressure wait
// compares against a snapshot rather than a bare flag, so a notification that
// lands between "read the snapshot" and "start waiting" cannot be missed.
std::atomic<std::uint64_t>& progress_epoch() {
  static std::atomic<std::uint64_t> value{0};
  return value;
}
std::mutex& progress_mutex() {
  static std::mutex value;
  return value;
}
std::condition_variable& progress_condition() {
  static std::condition_variable value;
  return value;
}
}  // namespace

// --- offline_bridge.hpp surface ---------------------------------------------

void reset_session_state() noexcept {
  input_closed_flag().store(false, std::memory_order_release);
  progress_epoch().store(0, std::memory_order_release);
}

bool input_closed() noexcept {
  return input_closed_flag().load(std::memory_order_acquire);
}

void note_progress() noexcept {
  progress_epoch().fetch_add(1, std::memory_order_release);
  progress_condition().notify_all();
}

void close_input() noexcept {
  input_closed_flag().store(true, std::memory_order_release);
  note_progress();  // release a producer parked in await_progress()
}

// --- Producer-side wait (bindings.cpp only, never seen by upstream) ---------

std::uint64_t current_progress_epoch() noexcept {
  return progress_epoch().load(std::memory_order_acquire);
}

// Blocks until the epoch moves past `epoch`, i.e. until the worker has
// actually dequeued something (or exited, or input was closed). Untimed on
// purpose: the only state the caller waits on is a full sweep queue, and the
// only thing that drains it is the worker loop, which notifies on every pop
// and once more when it stops.
std::uint64_t await_progress(std::uint64_t epoch) noexcept {
  std::unique_lock<std::mutex> lock(progress_mutex());
  progress_condition().wait(lock, [epoch] { return current_progress_epoch() != epoch; });
  return current_progress_epoch();
}

}  // namespace resple_bridge

namespace {

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
  std::int64_t map_batches_emitted = 0;
  // Data still buffered when the worker stopped. Non-zero is not necessarily a
  // bug (upstream never processes a trailing partial measurement batch either)
  // but it is the signal that would expose a silent tail drop, so it is
  // reported rather than discarded. Filled once, after the worker is joined.
  std::int64_t residual_sweeps = 0;
  std::int64_t residual_points = 0;

  void reset() {
    std::lock_guard<std::mutex> lock(mutex);
    trajectory.clear();
    map_batches.clear();
    events.clear();
    imu_samples = 0;
    lidar_sweeps_pushed = 0;
    map_batches_emitted = 0;
    residual_sweeps = 0;
    residual_points = 0;
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

// RESPLE's initialization() derives the gravity direction (and hence the
// initial orientation, and hence the whole trajectory) from the mean of
// `std::min(15, imu_buff.size())` samples. Below 15 that count depends on how
// far the producer thread happened to run before the worker reached init, so
// the result is scheduler-dependent; at 15 or more the mean is over a fixed
// prefix of a FIFO and is deterministic. Keep in sync with upstream
// RESPLE.cpp's `int n_imu = std::min(15, buff_size);`.
constexpr std::int64_t kInitGravitySamples = 15;

// The window of a spline over which a sampled pose is meaningful.
//
// Both ends of [minTimeNs(), maxTimeNs()] are excluded:
//   - minTimeNs() sits one knot interval *before* the first real knot while
//     the spline still carries its leading idle knots, so poses there are
//     backward extrapolation into the pre-initialization region.
//   - the final interval is forward propagation only: collectMeasurements()
//     will not form a batch until data exists beyond maxTimeNs() + dt_ns, so
//     at end of input the last interval never receives a measurement update.
//     This is also why upstream's own live publication trails the leading
//     edge by one knot.
// A cubic B-spline needs 4 control points before any of it is meaningful.
bool spline_pose_window(const SplineState* spline, std::int64_t& first_ns,
                        std::int64_t& last_ns) {
  if (!spline || spline->numKnots() < 4) return false;
  first_ns = std::max(spline->minTimeNs(), spline->getKnotTimeNs(0));
  last_ns = spline->maxTimeNs() - spline->getKnotTimeIntervalNs();
  return last_ns >= first_ns;
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
    resple_bridge::reset_session_state();
    reset_publisher_captures();

    node_ = rclcpp::Node::make_shared("RESPLE");
    apply_parameters(params);
    configure_lidar(node_, lidar_name_, lidar_type, scan_line, blind, q_lb, t_lb, w_pt);
    node_->set_parameter<std::vector<std::string>>("lidars", {lidar_name_});
    node_->set_parameter<bool>("if_lidar_only", false);
    if (!node_->was_provided("topic_imu"))
      node_->set_parameter<std::string>("topic_imu", "/offline/imu");

    resple_ = std::make_unique<RESPLE>(node_);
    if (resple_->point_filter_num < 1)
      throw py::value_error("point_filter_num must be at least 1");
    install_captures();  // captures resple_.get(); must run after construction
    start_worker();
  }

  ~RespleOdometry() {
    try {
      shutdown();
    } catch (...) {
      // A destructor must not propagate. Any worker failure has already been
      // surfaced by finish()/push_*; all this can lose is a join() error.
    }
  }

  std::int64_t push_imu(double timestamp, const std::vector<double>& acceleration,
                        const std::vector<double>& angular_velocity) {
    throw_if_failed();
    throw_if_input_closed();
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

    imu_pushed_ += 1;
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.imu_samples = imu_pushed_;
    return state.imu_samples;
  }

  std::int64_t push_lidar(double timestamp, const FloatArray& points,
                          const DoubleArray& relative_times) {
    throw_if_failed();
    throw_if_input_closed();
    validate_sweep(timestamp, points, relative_times);

    const std::int64_t time_begin_ns =
        seconds_to_ns(timestamp) - (lidar_type_ == "Ouster" ? resple_->time_offset : 0);
    // Both IMU-coverage checks run before the (blocking) backpressure wait:
    // neither depends on worker progress, so failing them early keeps a
    // rejected sweep from parking the producer first.
    require_imu_coverage(timestamp, time_begin_ns);
    apply_backpressure();
    throw_if_failed();

    auto p = points.unchecked<2>();
    auto t = relative_times.unchecked<1>();
    const py::ssize_t count = points.shape(0);
    const float blind = resple_->lidars.at(lidar_type_).blind;
    const float blind2 = blind * blind;
    // Upstream's per-frame `static int64_t last_t_ns = time_begin;` -- on the
    // first frame the cross-frame filter below compares against that frame's
    // own start, so a point at exactly offset 0 is dropped there too.
    if (last_absolute_point_stamp_ns_ == std::numeric_limits<std::int64_t>::min())
      last_absolute_point_stamp_ns_ = time_begin_ns;

    // The prepared dataset has already canonicalized sensor packets. At this
    // boundary preserve XYZI, encode relative time in the internal intensity
    // field, and apply the per-point filters ousterLidarCallback applies, in
    // the same order and against the same reference values: raw-index
    // thinning, blind range, and the strictly-increasing absolute-stamp test
    // against the previous frame's last kept point.
    Eigen::aligned_vector<pcl::PointXYZINormal> cloud;
    cloud.reserve(static_cast<std::size_t>(count));
    std::int64_t max_kept_offset_ns = 0;
    for (py::ssize_t i = 0; i < count; ++i) {
      if (i % resple_->point_filter_num != 0) continue;
      const float x = p(i, 0), y = p(i, 1), z = p(i, 2);
      if (x * x + y * y + z * z <= blind2) continue;
      const std::int64_t offset_ns = seconds_to_ns(t(i));
      if (time_begin_ns + offset_ns <= last_absolute_point_stamp_ns_) continue;
      pcl::PointXYZINormal pt;
      pt.x = x; pt.y = y; pt.z = z;
      pt.intensity = static_cast<float>(t(i) * 1000.0);  // seconds -> ms
      pt.curvature = points.shape(1) == 4 ? p(i, 3) : 0.0f;
      cloud.push_back(pt);
      if (offset_ns > max_kept_offset_ns) max_kept_offset_ns = offset_ns;
    }
    if (cloud.empty())
      throw py::value_error(
          "sweep at t=" + std::to_string(timestamp) +
          " has no points left after point_filter_num/blind-range/timestamp filtering");

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
    last_absolute_point_stamp_ns_ = time_begin_ns + max_kept_offset_ns;
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.lidar_sweeps_pushed += 1;
    return state.lidar_sweeps_pushed;
  }

  py::dict finish() {
    // No timeout parameter on purpose: completion is a state condition, and a
    // wall-clock deadline could only truncate a still-progressing replay,
    // which is exactly the nondeterminism this bridge exists to avoid.
    {
      py::gil_scoped_release release;
      shutdown();
    }
    throw_if_failed();
    sample_finalized_trajectory();
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
    // Rows in the trajectory buffer: live knot samples before finish(),
    // finalized-spline samples after it. status() reports the live count under
    // its own name so the two are never confused.
    result["poses_committed"] = static_cast<std::int64_t>(state.trajectory.size());
    result["map_batches_emitted"] = state.map_batches_emitted;
    result["residual_sweeps"] = state.residual_sweeps;
    result["residual_points"] = state.residual_points;
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
    result["live_poses_sampled"] = static_cast<std::int64_t>(state.trajectory.size());
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

    // Live samples are useful before finish(). finish() replaces this vector
    // by sampling the fully drained, finalized spline.
    RESPLE* resple_ptr = resple_.get();
    rclcpp::Publisher<estimate_msgs::msg::Estimate>::capture_slot() =
        [resple_ptr](const estimate_msgs::msg::Estimate&) {
          if (!resple_ptr) return;
          SplineState* spline = resple_ptr->spline;
          std::int64_t first_ns = 0, t_ns = 0;
          if (!spline_pose_window(spline, first_ns, t_ns)) return;
          Eigen::Vector3d pos = spline->itpPosition(t_ns);
          Eigen::Quaterniond q;
          spline->itpQuaternion(t_ns, &q);
          auto& state = recorder();
          std::lock_guard<std::mutex> lock(state.mutex);
          if (!state.trajectory.empty() && state.trajectory.back().timestamp >= ns_to_seconds(t_ns))
            return;
          state.trajectory.push_back(PoseRecord{ns_to_seconds(t_ns), pos.x(), pos.y(), pos.z(),
                                                q.x(), q.y(), q.z(), q.w()});
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
      worker_finished_.store(true, std::memory_order_release);
      // Unblocks a producer parked in apply_backpressure() on a worker that
      // has stopped; it rechecks worker_finished_ and raises.
      resple_bridge::note_progress();
    });
  }

  void shutdown() {
    if (!worker_.joinable()) return;
    resple_bridge::close_input();
    // join() is the state wait: processData returns only after a pass that
    // began with input closed and made no progress. Keep ownership until
    // then; this worker captures `this` and must never be detached.
    worker_.join();
    record_residual_buffers();
  }

  // Only safe once the worker is joined: pt_buff is worker-owned and has no
  // mutex of its own.
  void record_residual_buffers() {
    std::int64_t sweeps = 0, points = 0;
    for (const auto& [name, buffers] : resple_->lidars_data) {
      (void)name;
      sweeps += static_cast<std::int64_t>(buffers.t_buff.size());
      points += static_cast<std::int64_t>(buffers.pt_buff.size());
    }
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.residual_sweeps = sweeps;
    state.residual_points = points;
  }

  void throw_if_failed() {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_error_) std::rethrow_exception(worker_error_);
  }

  void throw_if_input_closed() const {
    if (resple_bridge::input_closed())
      throw std::runtime_error("RESPLE input is closed; no more sensor data may be pushed");
  }

  // Bounds the raw sweep queue. Note this bounds pc_buff only: processData
  // drains all of pc_buff into pt_buff on every pass, so pt_buff -- the far
  // heavier buffer -- is what actually grows when the producer outruns the
  // estimator. metrics()["residual_points"] reports its depth at exit.
  void apply_backpressure() {
    RESPLE::LidarData& buffers = resple_->lidars_data.at(lidar_type_);
    // Snapshot the epoch *before* reading the queue depth: a dequeue landing
    // between the two only makes the wait return immediately.
    std::uint64_t epoch = resple_bridge::current_progress_epoch();
    while (true) {
      std::size_t pending;
      {
        std::lock_guard<std::mutex> lock(buffers.mtx_pc);
        pending = buffers.t_buff.size();
      }
      if (static_cast<int>(pending) < max_pending_sweeps_) return;
      throw_if_failed();
      if (worker_finished_.load(std::memory_order_acquire))
        throw std::runtime_error("RESPLE worker stopped while applying lidar backpressure");
      epoch = resple_bridge::await_progress(epoch);
    }
  }

  // The two IMU-coverage preconditions, both producer-side and both checked
  // before any blocking wait.
  void require_imu_coverage(double timestamp, std::int64_t time_begin_ns) const {
    // "insufficient IMU lead" is load-bearing: scripts/run_resple.py matches on
    // it to tell a leading-edge IMU gap (skip this sweep) from a real error.
    if (last_lidar_stamp_ == -std::numeric_limits<double>::infinity() &&
        imu_pushed_ < kInitGravitySamples)
      throw py::value_error(
          "insufficient IMU lead for the first sweep at t=" + std::to_string(timestamp) +
          ": RESPLE averages the first " + std::to_string(kInitGravitySamples) +
          " IMU samples to derive gravity, so at least that many must precede the "
          "first accepted sweep for the result to be reproducible (have " +
          std::to_string(imu_pushed_) + ").");
    const double adjusted_timestamp = ns_to_seconds(time_begin_ns);
    if (last_imu_stamp_ < adjusted_timestamp + min_imu_lead_seconds_)
      throw py::value_error(
          "insufficient IMU lead for sweep at t=" + std::to_string(timestamp) +
          ": have IMU through " + std::to_string(last_imu_stamp_) +
          ", need through at least " +
          std::to_string(adjusted_timestamp + min_imu_lead_seconds_) +
          " (min_imu_lead_seconds=" + std::to_string(min_imu_lead_seconds_) +
          "). Push more IMU before this sweep.");
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

  // Replaces the live samples with a uniform resampling of the finalized
  // spline, once the worker has drained and joined. Publication timing is not
  // authoritative for a spline estimator -- knots keep being refined after the
  // control point that produced them was published.
  void sample_finalized_trajectory() {
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.trajectory.clear();
    SplineState* spline = resple_->spline;
    std::int64_t first_ns = 0, last_ns = 0;
    if (!spline_pose_window(spline, first_ns, last_ns)) return;
    const std::int64_t dt_ns = spline->getKnotTimeIntervalNs();
    for (std::int64_t t_ns = first_ns; t_ns <= last_ns; t_ns += dt_ns) {
      Eigen::Vector3d pos = spline->itpPosition(t_ns);
      Eigen::Quaterniond q;
      spline->itpQuaternion(t_ns, &q);
      state.trajectory.push_back(PoseRecord{ns_to_seconds(t_ns), pos.x(), pos.y(), pos.z(),
                                            q.x(), q.y(), q.z(), q.w()});
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
  std::exception_ptr worker_error_;
  std::int64_t imu_pushed_ = 0;
  double last_imu_stamp_ = -std::numeric_limits<double>::infinity();
  double last_lidar_stamp_ = -std::numeric_limits<double>::infinity();
  std::int64_t last_absolute_point_stamp_ns_ = std::numeric_limits<std::int64_t>::min();
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
           py::arg("relative_times"))
      .def("trajectory", &RespleOdometry::trajectory)
      .def("drain_map_batches", &RespleOdometry::drain_map_batches)
      .def("finish", &RespleOdometry::finish)
      .def("metrics", &RespleOdometry::metrics)
      .def("events", &RespleOdometry::events)
      .def("status", &RespleOdometry::status);
}

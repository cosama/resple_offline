// Host the upstream estimator.

#include <resple_bridge/offline_bridge.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "RESPLE.cpp"
#pragma GCC diagnostic pop

namespace py = pybind11;

namespace resple_bridge {

namespace {
struct TicketState {
  bool committed = false;
  bool raw_ingested = false;
  std::size_t pending_points = 0;
};

struct SynchronizationState {
  std::mutex mutex;
  std::condition_variable condition;
  std::uint64_t submission_epoch = 0;
  std::uint64_t stable_blocked_epoch = 0;
  std::uint64_t next_ticket = 1;
  std::uint64_t latest_committed_ticket = 0;
  std::uint64_t completed_prefix = 0;
  bool worker_finished = false;
  std::uint64_t request_generation = 0;
  std::uint64_t completed_request_generation = 0;
  std::uint64_t requested_epoch = 0;
  std::map<std::uint64_t, TicketState> tickets;
};

SynchronizationState& synchronization_state() {
  static SynchronizationState value;
  return value;
}

void advance_completed_prefix(SynchronizationState& state) {
  while (true) {
    const auto found = state.tickets.find(state.completed_prefix + 1);
    if (found == state.tickets.end() || !found->second.committed ||
        !found->second.raw_ingested || found->second.pending_points != 0)
      return;
    ++state.completed_prefix;
    // Discard completed ticket state.
    state.tickets.erase(found);
  }
}

std::atomic<bool>& input_closed_flag() {
  static std::atomic<bool> flag{false};
  return flag;
}
// Guard progress notifications.
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

void check_invariant(bool condition, const char* message) {
  if (!condition)
    throw std::logic_error(std::string("RESPLE bridge invariant violated: ") + message);
}

void reset_session_state() noexcept {
  input_closed_flag().store(false, std::memory_order_release);
  progress_epoch().store(0, std::memory_order_release);
  auto& state = synchronization_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.submission_epoch = 0;
  state.stable_blocked_epoch = 0;
  state.next_ticket = 1;
  state.latest_committed_ticket = 0;
  state.completed_prefix = 0;
  state.worker_finished = false;
  state.request_generation = 0;
  state.completed_request_generation = 0;
  state.requested_epoch = 0;
  state.tickets.clear();
}

bool input_closed() noexcept {
  return input_closed_flag().load(std::memory_order_acquire);
}

void note_progress() noexcept {
  {
    std::lock_guard<std::mutex> lock(progress_mutex());
    progress_epoch().fetch_add(1, std::memory_order_release);
  }
  progress_condition().notify_all();
}

std::uint64_t current_submission_epoch() noexcept {
  auto& state = synchronization_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.submission_epoch;
}

void submission_snapshot(std::uint64_t& epoch, std::uint64_t& latest_ticket) noexcept {
  auto& state = synchronization_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  epoch = state.submission_epoch;
  latest_ticket = state.latest_committed_ticket;
}

void note_imu_submission() noexcept {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.submission_epoch;
  }
  state.condition.notify_all();
}

std::uint64_t reserve_lidar_ticket() noexcept {
  auto& state = synchronization_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  const std::uint64_t ticket = state.next_ticket++;
  state.tickets.emplace(ticket, TicketState{});
  return ticket;
}

void commit_lidar_ticket(std::uint64_t ticket) {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    auto found = state.tickets.find(ticket);
    check_invariant(found != state.tickets.end() && !found->second.committed,
                    "lidar ticket committed twice or never reserved");
    found->second.committed = true;
    state.latest_committed_ticket = ticket;
    ++state.submission_epoch;
    advance_completed_prefix(state);
  }
  state.condition.notify_all();
}

void release_lidar_ticket(std::uint64_t ticket) noexcept {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    const auto found = state.tickets.find(ticket);
    if (found == state.tickets.end() || found->second.committed) return;
    // Retire unused ticket numbers.
    found->second.committed = true;
    found->second.raw_ingested = true;
    found->second.pending_points = 0;
    advance_completed_prefix(state);
  }
  state.condition.notify_all();
}

void note_raw_ingested(std::uint64_t ticket, std::size_t point_count) {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    auto found = state.tickets.find(ticket);
    check_invariant(found != state.tickets.end() && !found->second.raw_ingested,
                    "sweep ticket ingested twice or never reserved");
    found->second.raw_ingested = true;
    found->second.pending_points = point_count;
    advance_completed_prefix(state);
  }
  state.condition.notify_all();
}

void note_point_processed(std::uint64_t ticket) {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    auto found = state.tickets.find(ticket);
    check_invariant(found != state.tickets.end() && found->second.raw_ingested &&
                        found->second.pending_points > 0,
                    "more points acknowledged than the sweep ticket ingested");
    --found->second.pending_points;
    advance_completed_prefix(state);
  }
  state.condition.notify_all();
}

void note_stable_blocked(std::uint64_t submission_epoch) noexcept {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.stable_blocked_epoch = std::max(state.stable_blocked_epoch, submission_epoch);
  }
  state.condition.notify_all();
}

void wait_for_synchronization_release(std::uint64_t submission_epoch) noexcept {
  auto& state = synchronization_state();
  std::unique_lock<std::mutex> lock(state.mutex);
  if (state.request_generation == state.completed_request_generation ||
      submission_epoch < state.requested_epoch)
    return;
  // Preserve this handoff generation.
  const std::uint64_t handed_off_generation = state.request_generation;
  state.condition.wait(lock, [&] {
    return state.completed_request_generation >= handed_off_generation ||
           state.worker_finished;
  });
}

void note_worker_finished() noexcept {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.worker_finished = true;
  }
  state.condition.notify_all();
}

bool wait_for_stable_blocked(std::uint64_t submission_epoch) noexcept {
  auto& state = synchronization_state();
  std::unique_lock<std::mutex> lock(state.mutex);
  state.condition.wait(lock, [&] {
    return state.stable_blocked_epoch >= submission_epoch || state.worker_finished;
  });
  return state.stable_blocked_epoch >= submission_epoch;
}

std::uint64_t request_synchronization(std::uint64_t submission_epoch) {
  auto& state = synchronization_state();
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    check_invariant(state.request_generation == state.completed_request_generation,
                    "synchronization requested while another request is outstanding");
    generation = ++state.request_generation;
    state.requested_epoch = submission_epoch;
  }
  state.condition.notify_all();
  return generation;
}

void complete_synchronization(std::uint64_t request_generation) {
  auto& state = synchronization_state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    check_invariant(request_generation == state.request_generation,
                    "synchronization completed out of order");
    state.completed_request_generation = request_generation;
  }
  state.condition.notify_all();
}

std::uint64_t completed_ticket_prefix() noexcept {
  auto& state = synchronization_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.completed_prefix;
}

void ticket_snapshot(std::uint64_t& latest, std::uint64_t& completed,
                     std::size_t& incomplete) noexcept {
  auto& state = synchronization_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  latest = state.latest_committed_ticket;
  completed = state.completed_prefix;
  // Exclude released trailing reservations.
  incomplete = latest > completed ? static_cast<std::size_t>(latest - completed) : 0;
}

void close_input() noexcept {
  input_closed_flag().store(true, std::memory_order_release);
  note_progress();  // Release waiting producers.
}

std::uint64_t current_progress_epoch() noexcept {
  return progress_epoch().load(std::memory_order_acquire);
}

std::uint64_t await_progress(std::uint64_t epoch) noexcept {
  std::unique_lock<std::mutex> lock(progress_mutex());
  progress_condition().wait(lock, [epoch] { return current_progress_epoch() != epoch; });
  return current_progress_epoch();
}

}  // namespace resple_bridge

namespace {

using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;
using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IntArray = py::array_t<std::int32_t, py::array::c_style | py::array::forcecast>;

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
  std::int64_t lidar_points_after_preprocessing = 0;
  std::int64_t map_batches_emitted = 0;
  // Record unprocessed trailing input.
  std::int64_t residual_sweeps = 0;
  std::int64_t residual_points = 0;

};

Recorder& recorder() {
  static Recorder value;
  return value;
}

double ns_to_seconds(std::int64_t ns) { return static_cast<double>(ns) * 1e-9; }
std::int64_t seconds_to_ns(double seconds) {
  return static_cast<std::int64_t>(std::llround(seconds * 1e9));
}

// Exclude unrefined spline intervals.
bool spline_pose_window(const SplineState* spline, std::int64_t& first_ns,
                        std::int64_t& last_ns) {
  if (!spline || spline->numKnots() < 4) return false;
  first_ns = std::max(spline->minTimeNs(), spline->getKnotTimeNs(0));
  last_ns = spline->maxTimeNs() - spline->getKnotTimeIntervalNs();
  return last_ns >= first_ns;
}

// Preserve upstream parameter types.
struct LidarSpec {
  std::string name;
  std::string topic_lidar;
  std::string lidar_type;
  int scan_line = 1;
  float blind = 0.0f;
  std::vector<double> q_lb;
  std::vector<double> t_lb;
  double w_pt = 0.01;
};

LidarSpec parse_lidar_spec(const py::dict& entry) {
  auto field = [&entry](const char* key) -> py::handle {
    if (!entry.contains(key))
      throw py::value_error(std::string("lidar entry is missing '") + key + "'");
    return entry[key];
  };
  LidarSpec spec;
  spec.name = py::cast<std::string>(field("name"));
  spec.lidar_type = py::cast<std::string>(field("lidar_type"));
  spec.topic_lidar = py::cast<std::string>(field("topic_lidar"));
  spec.scan_line = py::cast<int>(field("scan_line"));
  spec.blind = py::cast<float>(field("blind"));
  spec.q_lb = py::cast<std::vector<double>>(field("q_lb"));
  spec.t_lb = py::cast<std::vector<double>>(field("t_lb"));
  spec.w_pt = py::cast<double>(field("w_pt"));
  if (spec.name.empty()) throw py::value_error("lidar name must not be empty");
  if (spec.q_lb.size() != 4)
    throw py::value_error("lidar '" + spec.name + "': q_lb must have length 4 (w, x, y, z)");
  if (spec.t_lb.size() != 3)
    throw py::value_error("lidar '" + spec.name + "': t_lb must have length 3");
  return spec;
}

void configure_lidar(rclcpp::Node::SharedPtr& node, const LidarSpec& spec) {
  const std::string prefix = spec.name + ".";
  node->set_parameter<std::string>(
      prefix + "topic_lidar",
      spec.topic_lidar.empty() ? "/offline/" + spec.name : spec.topic_lidar);
  node->set_parameter<std::string>(prefix + "lidar_type", spec.lidar_type);
  node->set_parameter<int>(prefix + "scan_line", spec.scan_line);
  node->set_parameter<float>(prefix + "blind", spec.blind);
  node->set_parameter<std::vector<double>>(prefix + "q_lb", spec.q_lb);
  node->set_parameter<std::vector<double>>(prefix + "t_lb", spec.t_lb);
  node->set_parameter<double>(prefix + "w_pt", spec.w_pt);
}

// Mirror per-type upstream state.
struct LidarStream {
  std::string type;
  int scan_line = 1;
  double last_stamp = -std::numeric_limits<double>::infinity();
  std::int64_t last_absolute_point_stamp_ns = std::numeric_limits<std::int64_t>::min();
  std::int64_t sweeps_pushed = 0;
  std::int64_t points_after_preprocessing = 0;
};

class RespleOdometry {
 public:
  RespleOdometry(const py::dict& params, const std::vector<py::dict>& lidars,
                int max_pending_sweeps)
      : max_pending_sweeps_(max_pending_sweeps) {
    if (lidars.empty()) throw py::value_error("at least one lidar is required");
    if (max_pending_sweeps_ < 1)
      throw py::value_error("max_pending_sweeps must be at least 1");

    if (consumed().exchange(true))
      throw std::runtime_error(
          "a RESPLE session has already run in this process. Upstream estimator "
          "state (the ikd-tree map) is process-global and is never reset, so "
          "each session needs a fresh process; run one dataset (or segment) per "
          "subprocess.");

    resple_bridge::reset_session_state();

    node_ = rclcpp::Node::make_shared("RESPLE");
    apply_parameters(params);
    std::vector<std::string> lidar_names;
    for (const py::dict& entry : lidars) {
      const LidarSpec spec = parse_lidar_spec(entry);
      for (const auto& [existing_name, existing] : streams_)
        if (existing.type == spec.lidar_type)
          throw py::value_error(
              "lidars '" + existing_name + "' and '" + spec.name + "' both declare "
              "lidar_type '" + spec.lidar_type + "': upstream RESPLE keys its "
              "per-lidar configuration and buffers by type (lidars.emplace(lidar.type, "
              "lidar)), so the second would be dropped and its sweeps fused into the "
              "first one's stream.");
      if (!streams_.emplace(spec.name, LidarStream{spec.lidar_type, spec.scan_line}).second)
        throw py::value_error("duplicate lidar name '" + spec.name + "'");
      configure_lidar(node_, spec);
      lidar_names.push_back(spec.name);
    }
    node_->set_parameter<std::vector<std::string>>("lidars", lidar_names);
    if (!node_->has_parameter("if_lidar_only"))
      node_->set_parameter<bool>("if_lidar_only", false);
    if (!node_->has_parameter("topic_imu"))
      node_->set_parameter<std::string>("topic_imu", "/offline/imu");

    resple_ = std::make_unique<RESPLE>(node_);
    if (resple_->point_filter_num < 1)
      throw py::value_error("point_filter_num must be at least 1");
    install_captures();  // Requires constructed estimator.
    start_worker();
  }

  ~RespleOdometry() {
    try {
      shutdown();
    } catch (...) {
      // Destructors cannot report failures.
    }
  }

  std::int64_t push_imu(double timestamp, const std::vector<double>& acceleration,
                        const std::vector<double>& angular_velocity) {
    throw_if_failed();
    // Prevent unused IMU buffering.
    if (resple_->if_lidar_only)
      throw py::value_error(
          "push_imu is not valid in LiDAR-only mode (if_lidar_only=true): "
          "upstream never consumes the IMU buffer in this mode, so these "
          "samples would accumulate unboundedly and never reach the estimator.");
    if (acceleration.size() != 3 || angular_velocity.size() != 3)
      throw py::value_error("acceleration and angular_velocity must have length 3");
    for (int axis = 0; axis < 3; ++axis)
      if (!std::isfinite(acceleration[axis]) || !std::isfinite(angular_velocity[axis]))
        throw py::value_error("IMU measurements must be finite");
    if (!std::isfinite(timestamp)) throw py::value_error("IMU timestamp must be finite");
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp = rclcpp::Time(seconds_to_ns(timestamp));
    message->linear_acceleration.x = acceleration[0];
    message->linear_acceleration.y = acceleration[1];
    message->linear_acceleration.z = acceleration[2];
    message->angular_velocity.x = angular_velocity[0];
    message->angular_velocity.y = angular_velocity[1];
    message->angular_velocity.z = angular_velocity[2];

    // Commit samples and epochs atomically.
    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      throw_if_input_closed();
      if (timestamp <= last_imu_stamp_)
        throw py::value_error("imu timestamps must be strictly increasing");
      std::lock_guard<std::mutex> lock(resple_->m_buff);
      resple_->imu_int_buff.push_back(message);
      last_imu_stamp_ = timestamp;
      ++imu_pushed_;
      resple_bridge::note_imu_submission();
    }

    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.imu_samples = imu_pushed_;
    return state.imu_samples;
  }

  std::int64_t push_imu_batch(
      const std::vector<double>& timestamps,
      const std::vector<std::vector<double>>& accelerations,
      const std::vector<std::vector<double>>& angular_velocities) {
    throw_if_failed();
    if (resple_->if_lidar_only)
      throw py::value_error("push_imu_batch is not valid in LiDAR-only mode");
    if (timestamps.empty()) throw py::value_error("IMU batch must not be empty");
    if (accelerations.size() != timestamps.size() ||
        angular_velocities.size() != timestamps.size())
      throw py::value_error("IMU batch fields must have equal lengths");

    Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> messages;
    messages.reserve(timestamps.size());
    double previous = last_imu_stamp_;
    for (std::size_t i = 0; i < timestamps.size(); ++i) {
      if (!std::isfinite(timestamps[i]) || timestamps[i] <= previous)
        throw py::value_error("imu timestamps must be finite and strictly increasing");
      if (accelerations[i].size() != 3 || angular_velocities[i].size() != 3)
        throw py::value_error("IMU measurements must have length 3");
      auto message = std::make_shared<sensor_msgs::msg::Imu>();
      message->header.stamp = rclcpp::Time(seconds_to_ns(timestamps[i]));
      for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(accelerations[i][axis]) ||
            !std::isfinite(angular_velocities[i][axis]))
          throw py::value_error("IMU measurements must be finite");
      }
      message->linear_acceleration.x = accelerations[i][0];
      message->linear_acceleration.y = accelerations[i][1];
      message->linear_acceleration.z = accelerations[i][2];
      message->angular_velocity.x = angular_velocities[i][0];
      message->angular_velocity.y = angular_velocities[i][1];
      message->angular_velocity.z = angular_velocities[i][2];
      messages.push_back(std::move(message));
      previous = timestamps[i];
    }

    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      throw_if_input_closed();
      // Commit complete IMU intervals.
      std::lock_guard<std::mutex> lock(resple_->m_buff);
      resple_->imu_int_buff.insert(
          resple_->imu_int_buff.end(), messages.begin(), messages.end());
      last_imu_stamp_ = timestamps.back();
      imu_pushed_ += static_cast<std::int64_t>(timestamps.size());
      resple_bridge::note_imu_submission();
    }

    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.imu_samples = imu_pushed_;
    return state.imu_samples;
  }

  std::int64_t push_lidar(double timestamp, const FloatArray& points,
                          const DoubleArray& relative_times,
                          const py::object& lines_object = py::none(),
                          const py::object& tags_object = py::none(),
                          const std::string& lidar = std::string()) {
    throw_if_failed();
    throw_if_input_closed();
    LidarStream& stream = select_stream(lidar);
    validate_sweep(stream, timestamp, points, relative_times, lines_object, tags_object);

    const std::int64_t time_begin_ns =
        seconds_to_ns(timestamp) - (stream.type == "Ouster" ? resple_->time_offset : 0);
    {
      py::gil_scoped_release release;
      apply_backpressure(stream);
    }
    throw_if_failed();

    auto p = points.unchecked<2>();
    auto t = relative_times.unchecked<1>();
    IntArray lines;
    IntArray tags;
    const bool has_lines = !lines_object.is_none();
    const bool has_tags = !tags_object.is_none();
    if (has_lines) lines = py::cast<IntArray>(lines_object);
    if (has_tags) tags = py::cast<IntArray>(tags_object);
    const py::ssize_t count = points.shape(0);
    const float blind = resple_->lidars.at(stream.type).blind;
    const float blind2 = blind * blind;
    // Initialize cross-frame filtering state.
    if (stream.last_absolute_point_stamp_ns == std::numeric_limits<std::int64_t>::min())
      stream.last_absolute_point_stamp_ns = time_begin_ns;

    // Match upstream callback filtering.
    Eigen::aligned_vector<pcl::PointXYZINormal> cloud;
    cloud.reserve(static_cast<std::size_t>(count));
    std::int64_t max_kept_offset_ns = 0;
    const bool livox_custom = stream.type == "Mid70Avia" || stream.type == "HAP360" ||
                              stream.type == "AviaResple";
    const bool avia_resple = stream.type == "AviaResple";
    int valid_point_num = 0;
    float previous_x = count ? p(0, 0) : 0.0f;
    float previous_y = count ? p(0, 1) : 0.0f;
    float previous_z = count ? p(0, 2) : 0.0f;
    for (py::ssize_t i = livox_custom ? 1 : 0; i < count; ++i) {
      const std::int64_t raw_offset_ns = seconds_to_ns(t(i));
      const float offset_ms = static_cast<float>(t(i) * 1000.0);
      const std::int64_t float_ms_offset_ns =
          static_cast<std::int64_t>(offset_ms * static_cast<float>(1e6));

      if (livox_custom) {
        const int line = has_lines ? lines.data()[i] : 0;
        const int tag = has_tags ? tags.data()[i] : 0;
        const bool driver_valid = line < stream.scan_line &&
            (((tag & 0x30) == 0x10) || ((tag & 0x30) == 0x00));
        if (!driver_valid) continue;
        // Preserve profile-specific predicate order.
        if (avia_resple && time_begin_ns + raw_offset_ns <= stream.last_absolute_point_stamp_ns)
          continue;
        ++valid_point_num;
        if (valid_point_num % resple_->point_filter_num != 0) continue;
      } else if (i % resple_->point_filter_num != 0) {
        continue;
      }

      const float x = p(i, 0), y = p(i, 1), z = p(i, 2);
      const bool duplicate = livox_custom && std::abs(x - previous_x) <= 1e-7f &&
                             std::abs(y - previous_y) <= 1e-7f &&
                             std::abs(z - previous_z) <= 1e-7f;
      const std::int64_t callback_offset_ns =
          (stream.type == "Hesai" || stream.type == "Mid360Boxi")
              ? float_ms_offset_ns : raw_offset_ns;
      const bool time_valid = avia_resple ||
          time_begin_ns + callback_offset_ns > stream.last_absolute_point_stamp_ns;
      const bool keep = offset_ms >= 0.0f && !duplicate &&
                        x * x + y * y + z * z > blind2 && time_valid;
      if (livox_custom) {
        // Preserve candidate duplicate state.
        previous_x = x;
        previous_y = y;
        previous_z = z;
      }
      if (!keep) continue;
      pcl::PointXYZINormal pt;
      pt.x = x; pt.y = y; pt.z = z;
      pt.intensity = offset_ms;
      pt.curvature = points.shape(1) == 4 ? p(i, 3) : 0.0f;
      cloud.push_back(pt);
      if (callback_offset_ns > max_kept_offset_ns) max_kept_offset_ns = callback_offset_ns;
    }

    const std::int64_t kept_points = static_cast<std::int64_t>(cloud.size());
    RESPLE::LidarData& buffers = resple_->lidars_data.at(stream.type);
    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      throw_if_input_closed();
      // Recheck after unlocked waiting.
      if (timestamp <= stream.last_stamp)
        throw py::value_error("sweep timestamps must be strictly increasing per lidar");
      const std::uint64_t ticket = resple_bridge::reserve_lidar_ticket();
      // Retire failed ticket reservations.
      struct Reservation {
        std::uint64_t ticket;
        bool committed = false;
        ~Reservation() {
          if (!committed) resple_bridge::release_lidar_ticket(ticket);
        }
      } reservation{ticket};
      std::lock_guard<std::mutex> lock(buffers.mtx_pc);
      buffers.pc_buff.push_back(std::move(cloud));
      buffers.t_buff.push_back(time_begin_ns);
      buffers.raw_ticket_buff.push_back(ticket);
      resple_bridge::check_invariant(buffers.pc_buff.size() == buffers.t_buff.size(),
                                     "raw sweep queue desynced from its timestamp sidecar");
      resple_bridge::check_invariant(buffers.pc_buff.size() == buffers.raw_ticket_buff.size(),
                                     "raw sweep queue desynced from its ticket sidecar");

      stream.last_stamp = timestamp;
      stream.last_absolute_point_stamp_ns = time_begin_ns + max_kept_offset_ns;
      stream.sweeps_pushed += 1;
      stream.points_after_preprocessing += kept_points;
      last_lidar_stamp_ = std::max(last_lidar_stamp_, timestamp);
      resple_bridge::commit_lidar_ticket(ticket);
      reservation.committed = true;

      auto& state = recorder();
      std::lock_guard<std::mutex> state_lock(state.mutex);
      state.lidar_sweeps_pushed += 1;
      state.lidar_points_after_preprocessing += kept_points;
      return static_cast<std::int64_t>(ticket);
    }
  }

  bool synchronize(const py::object& ticket_object = py::none()) {
    throw_if_failed();
    std::uint64_t target_epoch = 0, latest_ticket = 0;
    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      resple_bridge::submission_snapshot(target_epoch, latest_ticket);
    }
    const bool has_ticket = !ticket_object.is_none();
    const std::uint64_t target_ticket =
        has_ticket ? py::cast<std::uint64_t>(ticket_object) : latest_ticket;
    if (has_ticket && (target_ticket == 0 || target_ticket > latest_ticket))
      throw py::value_error("ticket is not a committed LiDAR submission");

    bool reached_stable = false;
    std::uint64_t completed_prefix = 0;
    {
      py::gil_scoped_release release;
      // Serialize explicit boundaries.
      std::unique_lock<std::mutex> synchronize_lock(synchronize_mutex_);
      const std::uint64_t request_generation =
          resple_bridge::request_synchronization(target_epoch);
      reached_stable = resple_bridge::wait_for_stable_blocked(target_epoch);
      // Hold estimator work quiescent.
      std::lock_guard<std::mutex> processing_lock(resple_->bridge_processing_mutex);
      ikdtree.wait_for_pending_rebuild();
      // Sample the requested boundary.
      completed_prefix = resple_bridge::completed_ticket_prefix();
      resple_bridge::complete_synchronization(request_generation);
    }
    throw_if_failed();
    if (!reached_stable) return false;
    // Accept absorbed IMU-only snapshots.
    return target_ticket == 0 || completed_prefix >= target_ticket;
  }

  py::dict finish() {
    // Wait for state completion.
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

  py::object latest_pose() {
    throw_if_failed();
    auto& state = recorder();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.trajectory.empty()) return py::none();
    const PoseRecord& pose = state.trajectory.back();
    py::array_t<double> out(8);
    auto row = out.mutable_unchecked<1>();
    row(0) = pose.timestamp;
    row(1) = pose.x; row(2) = pose.y; row(3) = pose.z;
    row(4) = pose.qx; row(5) = pose.qy; row(6) = pose.qz; row(7) = pose.qw;
    return out;
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
    result["lidar_points_after_preprocessing"] = state.lidar_points_after_preprocessing;
    result["poses_committed"] = static_cast<std::int64_t>(state.trajectory.size());
    result["map_batches_emitted"] = state.map_batches_emitted;
    result["residual_sweeps"] = state.residual_sweeps;
    result["residual_points"] = state.residual_points;
    add_ticket_counters(result);
    result["estimator_threads"] = NUM_OF_THREAD;
    result["per_lidar"] = per_lidar();
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
    result["lidar_points_after_preprocessing"] = state.lidar_points_after_preprocessing;
    result["worker_finished"] = worker_finished_.load();
    result["last_imu_stamp"] = last_imu_stamp_;
    result["last_lidar_stamp"] = last_lidar_stamp_;
    add_ticket_counters(result);
    result["per_lidar"] = per_lidar();
    return result;
  }

 private:
  // Snapshot ticket counters together.
  static void add_ticket_counters(py::dict& result) {
    std::uint64_t latest = 0, completed = 0;
    std::size_t incomplete = 0;
    resple_bridge::ticket_snapshot(latest, completed, incomplete);
    result["latest_ticket"] = latest;
    result["completed_ticket"] = completed;
    result["incomplete_tickets"] = static_cast<std::int64_t>(incomplete);
  }

  // Expose stalled LiDAR streams.
  py::dict per_lidar() const {
    py::dict result;
    for (const auto& [name, stream] : streams_) {
      py::dict entry;
      entry["lidar_type"] = stream.type;
      entry["sweeps_pushed"] = stream.sweeps_pushed;
      entry["points_after_preprocessing"] = stream.points_after_preprocessing;
      entry["last_stamp"] = stream.last_stamp;
      result[py::str(name)] = std::move(entry);
    }
    return result;
  }

  // Resolve explicit LiDAR streams.
  LidarStream& select_stream(const std::string& name) {
    if (name.empty()) {
      if (streams_.size() != 1)
        throw py::value_error(
            "this session has " + std::to_string(streams_.size()) +
            " lidars (" + lidar_names() + "); push_lidar needs an explicit lidar= name");
      return streams_.begin()->second;
    }
    auto found = streams_.find(name);
    if (found == streams_.end())
      throw py::value_error("unknown lidar '" + name + "'; configured: " + lidar_names());
    return found->second;
  }

  std::string lidar_names() const {
    std::string result;
    for (const auto& [name, stream] : streams_) {
      (void)stream;
      if (!result.empty()) result += ", ";
      result += "'" + name + "'";
    }
    return result;
  }

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
        // Preserve upstream double parameters.
        node_->set_parameter<double>(key, py::cast<double>(value));
      } else {
        throw py::type_error("unrecognized RESPLE native parameter: " + key);
      }
    }
  }

  void install_captures() {
    rclcpp::Publisher<std_msgs::msg::Int64>::capture_slot() =
        [](const std_msgs::msg::Int64& msg) {
          auto& state = recorder();
          std::lock_guard<std::mutex> lock(state.mutex);
          state.events.push_back(Event{"init", ns_to_seconds(msg.data)});
        };

    // Capture provisional live poses.
    RESPLE* resple_ptr = resple_.get();
    rclcpp::Publisher<estimate_msgs::msg::Estimate>::capture_slot() =
        [resple_ptr](const estimate_msgs::msg::Estimate&) {
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
      resple_bridge::note_worker_finished();
      // Release blocked producers.
      resple_bridge::note_progress();
    });
  }

  void shutdown() {
    if (!worker_.joinable()) return;
    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      resple_bridge::close_input();
    }
    // Retain worker ownership.
    worker_.join();
    // Quiesce final tree rebuilds.
    ikdtree.wait_for_pending_rebuild();
    record_residual_buffers();
  }

  // Requires a joined worker.
  void record_residual_buffers() {
    std::int64_t sweeps = 0, points = 0;
    for (const auto& entry : resple_->lidars_data) {
      const auto& buffers = entry.second;
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

  void apply_backpressure(const LidarStream& stream) {
    RESPLE::LidarData& buffers = resple_->lidars_data.at(stream.type);
    // Snapshot progress before depth.
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

  void validate_sweep(const LidarStream& stream, double timestamp, const FloatArray& points,
                      const DoubleArray& relative_times,
                      const py::object& lines, const py::object& tags) const {
    if (points.ndim() != 2 || (points.shape(1) != 3 && points.shape(1) != 4))
      throw py::value_error("points must have shape (N, 3) or (N, 4)");
    if (points.shape(0) == 0) throw py::value_error("sweep must contain points");
    if (relative_times.ndim() != 1 || relative_times.shape(0) != points.shape(0))
      throw py::value_error("relative_times must have shape (N,)");
    if (!std::isfinite(timestamp)) throw py::value_error("sweep timestamp must be finite");
    // Validate timestamps per stream.
    if (timestamp <= stream.last_stamp)
      throw py::value_error("sweep timestamps must be strictly increasing per lidar");

    auto p = points.unchecked<2>();
    auto t = relative_times.unchecked<1>();
    for (py::ssize_t i = 0; i < points.shape(0); ++i) {
      if (!std::isfinite(t(i)))
        throw py::value_error("relative_times must be finite");
      for (py::ssize_t j = 0; j < points.shape(1); ++j)
        if (!std::isfinite(p(i, j))) throw py::value_error("points must be finite");
    }
    for (const auto& field : {std::make_pair("lines", &lines), std::make_pair("tags", &tags)}) {
      if (field.second->is_none()) continue;
      IntArray values = py::cast<IntArray>(*field.second);
      if (values.ndim() != 1 || values.shape(0) != points.shape(0))
        throw py::value_error(std::string(field.first) + " must have shape (N,)");
    }
  }

  // Sample the finalized spline.
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

  // Key streams by profile.
  std::map<std::string, LidarStream> streams_;
  int max_pending_sweeps_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RESPLE> resple_;
  std::thread worker_;
  std::atomic<bool> worker_finished_{false};
  std::mutex worker_mutex_;
  std::mutex input_mutex_;
  std::mutex synchronize_mutex_;
  std::exception_ptr worker_error_;
  std::int64_t imu_pushed_ = 0;
  double last_imu_stamp_ = -std::numeric_limits<double>::infinity();
  // Track the latest sweep.
  double last_lidar_stamp_ = -std::numeric_limits<double>::infinity();
};

}  // namespace

PYBIND11_MODULE(_core, module) {
  module.doc() = "Offline host for pinned upstream RESPLE";

  py::class_<RespleOdometry>(module, "RespleOdometry")
      .def(py::init<const py::dict&, const std::vector<py::dict>&, int>(),
           py::arg("parameters"), py::arg("lidars"),
           py::arg("max_pending_sweeps") = 8)
      .def("push_imu", &RespleOdometry::push_imu, py::arg("timestamp"),
           py::arg("acceleration"), py::arg("angular_velocity"))
      .def("push_imu_batch", &RespleOdometry::push_imu_batch, py::arg("timestamps"),
           py::arg("accelerations"), py::arg("angular_velocities"))
      .def("push_lidar", &RespleOdometry::push_lidar, py::arg("timestamp"), py::arg("points"),
           py::arg("relative_times"), py::arg("lines") = py::none(),
           py::arg("tags") = py::none(), py::arg("lidar") = std::string())
      .def("synchronize", &RespleOdometry::synchronize, py::arg("ticket") = py::none())
      .def("trajectory", &RespleOdometry::trajectory)
      .def("latest_pose", &RespleOdometry::latest_pose)
      .def("drain_map_batches", &RespleOdometry::drain_map_batches)
      .def("finish", &RespleOdometry::finish)
      .def("metrics", &RespleOdometry::metrics)
      .def("events", &RespleOdometry::events)
      .def("status", &RespleOdometry::status);
}

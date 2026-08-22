#pragma once

#include <cstddef>
#include <cstdint>

// Hook ABI between the pinned upstream RESPLE source and the offline bridge.
//
// Included by patched upstream/RESPLE.cpp (see patches/integration/). Must
// stay free of pybind11, Python, and any host type: upstream sees POD/free
// functions only. Definitions live in bindings.cpp.

namespace resple_bridge {

// --- Producer side (bindings.cpp) ------------------------------------------
// Clear all bridge-owned session state. Called once before the worker starts.
void reset_session_state() noexcept;
// Called once, after the last push_imu/push_lidar, to declare input finished.
void close_input() noexcept;

// Every committed producer insertion advances this epoch. A worker pass
// snapshots it before scanning the input queues; publishing that epoch as
// stable-blocked proves that all input committed through the snapshot has
// been observed and that no further estimator batch can be formed from it.
std::uint64_t current_submission_epoch() noexcept;
void submission_snapshot(std::uint64_t& epoch, std::uint64_t& latest_ticket) noexcept;
void note_imu_submission() noexcept;

// LiDAR tickets are process-global across all configured streams. Both calls
// happen under the producer's own push serialization, so tickets are reserved
// and committed in submission order; commit happens only after the raw cloud,
// timestamp, and ticket have been enqueued together, under the buffer lock the
// worker must take to see any of them. A ticket reserved but not committed
// (only reachable if the enqueue itself throws) holds back the completed
// prefix rather than letting it run ahead of the data.
std::uint64_t reserve_lidar_ticket() noexcept;
void commit_lidar_ticket(std::uint64_t ticket);
// Drops a reservation whose enqueue threw before it could be committed. Without
// it that ticket would hold the completed prefix one below itself for the rest
// of the session -- a silent wedge, where the failure it came from was loud.
void release_lidar_ticket(std::uint64_t ticket) noexcept;

// --- Worker side (patched processData) -------------------------------------
// Sampled *once at the top of each outer pass*, never at the bottom: a pass
// that starts with input already closed has necessarily seen everything the
// producer will ever push (pushes happen-before the close, and push_* throws
// afterwards), so "closed at pass start and this pass moved nothing" proves
// the buffers are drained. Testing closure at the end of a pass instead would
// race with a push that landed after the pass scanned the buffers.
bool input_closed() noexcept;

// Called by the worker whenever it actually moved data out of a producer-fed
// buffer. This is the only thing that releases push_lidar's backpressure
// wait, so it must not be called on no-progress passes.
void note_progress() noexcept;

// Ticket lifecycle hooks. `point_count` is the post-voxel-filter count. Each
// point is acknowledged exactly once: after its estimator batch, or when
// upstream window logic definitively discards it. A sweep left with no points
// by preprocessing therefore completes at raw ingestion.
//
// Not noexcept: these validate their ticket, and on the worker thread
// bindings.cpp turns the resulting exception into a session failure.
void note_raw_ingested(std::uint64_t ticket, std::size_t point_count);
void note_point_processed(std::uint64_t ticket);

// Always-on invariant check for the ticket sidecars; throws std::logic_error.
// Deliberately not assert(): the only build configuration that ships is
// Release (pyproject sets it), so NDEBUG would compile these out exactly where
// a desynced sidecar stops being a wrong number and becomes undefined
// behavior -- front() on an emptied deque, or an unsigned point counter
// wrapping past zero.
void check_invariant(bool condition, const char* message);

// Published only after a pass has drained raw inputs and cannot form another
// measurement batch. synchronize() waits on this epoch rather than polling.
void note_stable_blocked(std::uint64_t submission_epoch) noexcept;
void wait_for_synchronization_release(std::uint64_t submission_epoch) noexcept;
void note_worker_finished() noexcept;

// Producer-side synchronization helpers. Definitions remain bridge-owned;
// patched upstream code only publishes lifecycle events through the hooks
// above.
bool wait_for_stable_blocked(std::uint64_t submission_epoch) noexcept;
// Not noexcept: both check the request-generation handshake (see
// check_invariant), and the caller is a Python thread inside synchronize().
std::uint64_t request_synchronization(std::uint64_t submission_epoch);
void complete_synchronization(std::uint64_t request_generation);
std::uint64_t completed_ticket_prefix() noexcept;
// All three counters under one lock: read separately they can disagree
// (`incomplete != latest - completed`) whenever the worker advances between
// two of the reads, which is exactly what a live status call would do.
void ticket_snapshot(std::uint64_t& latest, std::uint64_t& completed,
                     std::size_t& incomplete) noexcept;

}  // namespace resple_bridge

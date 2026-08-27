#pragma once

#include <cstddef>
#include <cstdint>

// Dependency-free hook ABI used by the patched upstream translation unit.
// Definitions live in bindings.cpp.

namespace resple_bridge {

void reset_session_state() noexcept;
void close_input() noexcept;

// Every committed insertion advances this epoch. A stable-blocked publication
// proves the worker has observed everything through its snapshot.
std::uint64_t current_submission_epoch() noexcept;
void submission_snapshot(std::uint64_t& epoch, std::uint64_t& latest_ticket) noexcept;
void note_imu_submission() noexcept;

// Tickets are global across streams and committed in submission order.
std::uint64_t reserve_lidar_ticket() noexcept;
void commit_lidar_ticket(std::uint64_t ticket);
void release_lidar_ticket(std::uint64_t ticket) noexcept;

// Sampled at the start of each worker pass. Closed-at-start plus no progress
// proves the producer buffers are drained without racing a final push.
bool input_closed() noexcept;

// Releases producer backpressure; call only after moving buffered input.
void note_progress() noexcept;

// point_count is post-voxel-filter. Each point is acknowledged after estimation
// or definitive discard; these functions throw on a broken ticket lifecycle.
void note_raw_ingested(std::uint64_t ticket, std::size_t point_count);
void note_point_processed(std::uint64_t ticket);

// Always on in Release builds, unlike assert().
void check_invariant(bool condition, const char* message);

// Published only after a pass has drained raw inputs and cannot form another
// measurement batch. synchronize() waits on this epoch rather than polling.
void note_stable_blocked(std::uint64_t submission_epoch) noexcept;
void wait_for_synchronization_release(std::uint64_t submission_epoch) noexcept;
void note_worker_finished() noexcept;

// Deterministic replay makes measurement-batch composition a function of
// sensor timestamps rather than of how far the producer has run when the
// estimator worker examines its buffers. Set once before the worker starts;
// read only by that worker thereafter.
bool deterministic_replay() noexcept;
void set_deterministic_replay(bool enabled) noexcept;

bool wait_for_stable_blocked(std::uint64_t submission_epoch) noexcept;
std::uint64_t request_synchronization(std::uint64_t submission_epoch);
void complete_synchronization(std::uint64_t request_generation);
std::uint64_t completed_ticket_prefix() noexcept;
// Takes one consistent snapshot under the synchronization-state lock.
void ticket_snapshot(std::uint64_t& latest, std::uint64_t& completed,
                     std::size_t& incomplete) noexcept;

}  // namespace resple_bridge

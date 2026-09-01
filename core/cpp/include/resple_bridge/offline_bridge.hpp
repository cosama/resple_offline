#pragma once

#include <cstddef>
#include <cstdint>

// Patched upstream hook ABI.

namespace resple_bridge {

void reset_session_state() noexcept;
void close_input() noexcept;

// Track committed input epochs.
std::uint64_t current_submission_epoch() noexcept;
void submission_snapshot(std::uint64_t& epoch, std::uint64_t& latest_ticket) noexcept;
void note_imu_submission() noexcept;

// Track globally ordered tickets.
std::uint64_t reserve_lidar_ticket() noexcept;
void commit_lidar_ticket(std::uint64_t ticket);
void release_lidar_ticket(std::uint64_t ticket) noexcept;

// Detect closed input state.
bool input_closed() noexcept;

// Release waiting producers.
void note_progress() noexcept;

// Track processed ticket points.
void note_raw_ingested(std::uint64_t ticket, std::size_t point_count);
void note_point_processed(std::uint64_t ticket);

// Enforce release-build invariants.
void check_invariant(bool condition, const char* message);

// Publish stable worker boundaries.
void note_stable_blocked(std::uint64_t submission_epoch) noexcept;
void wait_for_synchronization_release(std::uint64_t submission_epoch) noexcept;
void note_worker_finished() noexcept;

bool wait_for_stable_blocked(std::uint64_t submission_epoch) noexcept;
std::uint64_t request_synchronization(std::uint64_t submission_epoch);
void complete_synchronization(std::uint64_t request_generation);
std::uint64_t completed_ticket_prefix() noexcept;
// Read consistent ticket counters.
void ticket_snapshot(std::uint64_t& latest, std::uint64_t& completed,
                     std::size_t& incomplete) noexcept;

}  // namespace resple_bridge

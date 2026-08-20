#pragma once

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

}  // namespace resple_bridge

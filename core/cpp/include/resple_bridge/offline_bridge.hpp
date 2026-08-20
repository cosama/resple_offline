#pragma once

// Hook ABI between the pinned upstream RESPLE source and the offline bridge.
//
// Included by patched upstream/RESPLE.cpp (see patches/integration/). Must
// stay free of pybind11, Python, and any host type: upstream sees POD/free
// functions only. Definitions live in bindings.cpp.

namespace resple_bridge {

// processData()'s main loop has no exit condition upstream (real ROS nodes
// are expected to be killed, not joined). This is what lets the bridge stop
// it deterministically once all pushed sensor data has drained -- see
// patches/integration/02-finish-condition.patch.
bool finish_requested() noexcept;

}  // namespace resple_bridge

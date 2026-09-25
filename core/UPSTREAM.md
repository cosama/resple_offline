# Upstream integration

## Pin

- Source: `upstream/RESPLE`
- Revision: `2f8b09e62597c3cb7492c37bb2d1c1736a501d85`
- Package: `resple/`

The submodule stays unmodified. CMake copies `resple/include` and `resple/src`
into `build/upstream_staged`, then applies `patches/integration` with zero fuzz.
Patch failures indicate upstream drift.

## Bridge boundary

The bridge compiles the staged `RESPLE.cpp` against a small ROS compatibility
layer. `RESPLE.cpp` is included in `bindings.cpp` so upstream globals remain in
one translation unit.

The shim provides only required surfaces:

- typed ROS parameters;
- prepared IMU and LiDAR input;
- publisher capture callbacks;
- worker lifecycle synchronization.

Sensor input enters upstream queues after matching each configured callback's
filtering. Estimation, association, spline updates, and map maintenance remain
upstream code.

Ticket sidecars track accepted sweeps without changing estimator data types.
A ticket completes after all surviving points finish estimation, map insertion,
or terminal upstream discard.

## Configuration

`RespleConfig` contains upstream ROS parameters only. Offline queue policy stays
on `RespleOdometry` and in the run manifest.

The resolver accepts flat fallback mappings and ROS2 parameter envelopes.
Wildcard `/**` parameters take precedence over unrelated node envelopes.
Named profiles not selected by `lidars` are ignored, matching upstream.

Dataset `resple.yaml` files are authoritative and remain unchanged. When none
exists, the runner resolves `configs/resple/default.yaml` in memory and adds the
single dataset LiDAR profile from metadata and URDF data. The effective upstream
mapping is written only to the run output directory.

Multi-LiDAR sessions require one distinct upstream `lidar_type` per profile.
Every configured stream must be supplied because upstream forms a batch only
when all LiDAR buffers contain points. The prepared-dataset runner rejects
multi-LiDAR configs because its schema contains one point stream.

## Determinism

Determinism is structural, not configurable. The host submits complete,
timestamp-ordered intervals and establishes explicit `synchronize()` boundaries.

Each boundary waits until:

- submitted queues reach a stable state;
- the worker releases its processing pass;
- pending ikd-tree rebuilds finish.

Initialization requires the fixed 15-sample gravity prefix and a complete
100 ms map window. Filter propagation occurs once after both requirements pass.
OpenMP is required to match upstream's five-thread execution path.

Free-running clients retain upstream rebuild scheduling between explicit
boundaries. `finish()` closes input, joins the worker, and quiesces the final
tree rebuild without a wall-clock cutoff.

## Patch series

| Patch | Purpose |
|---|---|
| `01-resple-node.patch` | Adds offline lifecycle, queue locking, tickets, deterministic initialization, safe shutdown, and required bounds checks. |
| `02-ikd-tree.patch` | Exposes rebuild quiescence at explicit host boundaries. |
| `03-relocate-new-knot-noise.patch` | Fixes upstream's new-control-point process-noise block being written over the IMU-bias states instead of spline state indices 18–23. |

Estimator-math patches require an isolated upstream defect, a focused source
contract, and real-data validation. Measurement-readiness changes require
separate scheduling and input-conservation validation.

## Validation limits

The native suite covers callback filtering, lifecycle, ticket completion,
multi-LiDAR input, sparse spline extension, final draining, and repeated-run
trajectory equality. A real ROS2 parity run remains required before treating
new upstream revisions as benchmark-grade.

## Upgrade

1. Update the recorded revision.
2. Reconfigure the native build.
3. Rebase rejected patch hunks.
4. Run native and runner tests.
5. Compare repeated real-data outputs.
6. Confirm the submodule is clean.

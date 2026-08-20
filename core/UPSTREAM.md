# Upstream provenance and patch policy

## Pin

| | |
|---|---|
| Source | `upstream/RESPLE` (submodule, `git@github.com:ASIG-X/RESPLE.git`) |
| Revision | `2f8b09e62597c3cb7492c37bb2d1c1736a501d85` (`main`) |
| Package used | `resple/` (`RESPLE.cpp` + its headers + `include/ikd-Tree/`). `Mapping.cpp` and the per-sensor message packages (`AviaResple_msgs`, `HAP360_msgs`, `Mid70Avia_msgs`, `estimate_msgs`) are not built -- see "Design" below. |

The submodule is **never modified**. `git -C upstream/RESPLE status --short`
must stay empty. At configure time CMake copies `resple/{include,src}` into
`build/upstream_staged` and applies the patch series there, so the local
delta is exactly the contents of `patches/integration/`.

## Design

RESPLE.cpp is one ROS2 `rclcpp::Node` that mixes real algorithm orchestration
(`processData`: deskew, `Estimator`/`Association` calls, ikd-tree incremental
map insert, FOV box segmentation -- same FAST-LIO lineage as DALI-SLAM's
`laserMapping.cpp`) with ROS I/O (subscriber callbacks, parameter loading,
publishers). Unlike DALI's upstream, this isn't already split into a
separate `io.cpp`.

Rather than hand-splitting that file, this bridge compiles it **unmodified**
against a compat `rclcpp` shim (`cpp/compat/`) that structurally replaces the
ROS graph:

- **Parameters**: `rclcpp::Node`'s param store is bridge-populated before
  RESPLE's constructor runs (`Node::set_parameter<T>`), typed exactly as
  `CommonUtils::readParam<T>` expects -- `std::any_cast<T>` fails silently on
  a type mismatch (e.g. `double` where upstream reads `float`), so
  `cpp/bindings.cpp`'s `apply_parameters`/`configure_lidar` set every key
  with its exact upstream C++ type rather than sniffing types from a generic
  dict.
- **Sensor input**: `create_subscription<T>()` is a no-op -- the bridge never
  drives data through RESPLE's own per-sensor callbacks (they still compile,
  since their addresses are taken in the constructor, but are never invoked).
  Instead `RespleOdometry::push_imu`/`push_lidar` inject directly into the
  *same buffers* those callbacks would have filled (`imu_int_buff`,
  `lidars_data[type].{pc_buff,t_buff}`), replicating each callback's own
  blind-range filter and ms-offset intensity encoding at the injection site.
  Everything downstream -- `processData`'s deskew/PointData construction,
  the IEKF update, ikd-tree maintenance -- runs as upstream wrote it.
- **Results**: `create_publisher<T>()` returns a `Publisher<T>` whose
  `publish()` forwards to a bridge-registered capture callback
  (`Publisher<T>::capture_slot()`). This is how the bridge observes init
  (`start_time`), spline growth (`est_window`, used to sample
  `spline->itpPosition/itpQuaternion` one knot interval behind the leading
  edge), and incremental map batches (`current_scan`) -- realized entirely in
  the compat layer, no patch to a publish call site.
- **ikd-tree/`NUM_OF_THREAD`/`NUM_MATCH_POINTS`**: file-scope globals in
  RESPLE.cpp/`common_utils.h` with no `extern`/`inline`. `cpp/bindings.cpp`
  `#include`s the staged `src/RESPLE.cpp` directly (one translation unit,
  the same technique `frameworks/voxel_slam` uses) rather than linking it as
  a second object file, which would ODR-violate on those globals.

This means the only patches needed are visibility and a shutdown hook --
see below -- not a hand-authored split of ROS I/O vs. algorithm body.

## Patch series (`patches/integration/`)

| Patch | What |
|---|---|
| `01-bridge-visibility.patch` | Widens `RESPLE`'s `private:` section (buffers, `spline`, callbacks) to `public:` so `cpp/bindings.cpp` can inject sensor data and read spline state from outside the class. No behavior change. |
| `02-finish-condition.patch` | `processData()`'s loop is `while (true)` upstream -- real ROS nodes are killed, not joined, so there is no exit condition at all. Adds one `#include` and changes the loop condition to `!resple_bridge::finish_requested()`, a bridge-owned atomic, so `RespleOdometry::finish()` can stop the worker thread deterministically instead of relying on process teardown. |

## Known limitations / open validation

- **Trajectory sampling is a bridge judgment call, not verified against
  upstream's own convention.** RESPLE publishes incremental *spline control
  points* (`est_window`/`estimate_msgs::msg::Estimate`), not one pose per
  processed sweep like DALI's `/Odometry`. This bridge samples
  `spline->itpPosition/itpQuaternion` once per new knot, one knot interval
  behind the leading edge. The upstream remote has a `feature/benchmark`
  branch that adds a TUM-trajectory-export service -- it was not inspected
  before writing this bridge and should be read to confirm (or correct) the
  sampling convention before trusting exported trajectories quantitatively.
- **Real-dataset run done; no real-ROS2 parity run.** This sandbox has no
  ROS2 (Humble) install, so output has not been compared against upstream's
  own supported runtime. It has been run end-to-end on a real ~287 s
  handheld-lidar dataset (`nglamp_50CPatio_2025_08_06_14_31_28_handheldsimple`,
  53M points / 2873 sweeps / 57455 IMU samples) via `scripts/run_resple.py`
  and compared qualitatively against a trusted `voxel_slam` reference run on
  the same dataset: processed in 48.2 s (5.96x faster than the dataset's own
  duration); 28377 poses (RESPLE samples ~knot_hz=100 Hz vs. voxel_slam's
  10 Hz per-sweep rate); trajectory extent `[23.2, 14.5, 1.6]` m vs.
  voxel_slam's `[22.0, 16.7, 1.6]` m (close); same duration (287.18 s vs.
  286.9 s); top-down map/trajectory shape visually matches voxel_slam's
  (same corridor layout, same out-and-back-plus-loop path, no
  runaway/smearing/duplication). `path_length_m` came out ~2x voxel_slam's
  (170 m vs. 89 m) with a correspondingly higher median/max instantaneous
  speed. **Resolved**: computed directly from both frameworks'
  `trajectory.csv` on this dataset -- RESPLE native (~100 Hz, 28377 poses)
  path_length 170.3 m vs. voxel_slam native (~10 Hz, 2870 poses) 89.5 m;
  resampling RESPLE (linear interpolation) onto voxel_slam's own timestamps
  gives 99.0 m, and onto a plain fixed 10 Hz grid gives 98.9 m. So ~9/10 of
  the apparent "2x" gap is a sampling-rate artifact (RESPLE's native median
  step is 0.0053 m at 100 Hz vs. voxel_slam's 0.0283 m at 10 Hz; summing many
  small noisy steps inflates measured arc length faster than true
  displacement as sampling rate increases -- expected for any noisy
  trajectory sampled ~10x denser, not a bug). The remaining ~11% gap after
  rate-matching (99.0 m vs. 89.5 m) does look like genuine extra jitter in
  RESPLE's estimate: measured jitter RMS (residual vs. a 0.5 s local-mean
  smooth, at RESPLE's native 100 Hz) is ~1.1 cm -- not a blocker, but
  `manifest.json`'s `speed_median_mps`/`speed_max_mps`/`path_length_m`
  fields should not be compared directly against other frameworks' without
  accounting for RESPLE's much higher native pose rate (a rate-matched
  path-length metric in the manifest would be a reasonable future addition,
  not done here). Two real bugs surfaced and were fixed by this run
  (not by the earlier synthetic-only pass): a `push_lidar` `ValueError` for
  the dataset's final sweep (no trailing IMU past it -- expected per
  ARCHITECTURE.md, `run_resple.py` now treats it as end-of-input rather than
  a fatal error) that was, before the second fix, crashing the whole
  process with `terminate called without an active exception` because the
  still-running native worker thread was left for the interpreter to tear
  down mid-shutdown instead of being drained via `finish()` in a `finally`.
  Still run the ARCHITECTURE.md validation ladder's bounded-real-data-parity
  step against a real ROS2 build (upstream's own `Dockerfile`) before
  trusting this for benchmark numbers.
- **Determinism: one bug found and fixed, ikd-tree race not yet ruled out.**
  `RespleOdometry::finish()` originally set the bridge's shutdown flag
  immediately, but `processData()`'s loop only checks it between full drain
  passes -- pushing a whole synthetic dataset (which, with no backpressure
  wait ever triggered, can outrun the worker thread) and calling `finish()`
  right after could stop the worker mid-drain and silently lose already-
  pushed sweeps. This produced actually-observed run-to-run differences in
  poses-committed and the final pose on an identical synthetic input.
  `shutdown()` now waits for the mutex-protected per-lidar sweep queue to
  empty plus a fixed grace period before signalling finish; 3 repeated runs
  of the box-room synthetic scene (`core/tests/synthetic.py`) now produce
  bit-identical trajectories. The grace-period wait is not watertight under
  extreme load (`pt_buff`, processData's internal per-point queue, has no
  mutex of its own to poll safely from the shutdown thread) -- tighten if a
  real dataset run ever reproduces the original symptom.
  ikd-Tree separately runs a background rebuild thread (confirmed:
  importing `_core` alone starts and stops one at static init) -- the same
  lineage that caused DA-LIO's offline-replay nondeterminism
  (`dali-slam-nondeterminism-race` in project memory). DALI's fix
  (`patches/fixes/05-deterministic-ikdtree-rebuild.patch` +
  `integration/05-event-driven-loop.patch` there) was not ported here; the
  synthetic determinism result above does not rule this out on a larger,
  slower, real dataset where rebuilds are actually triggered mid-sweep.

## Upgrade procedure

1. Record the currently pinned revision (above) and this patch series.
2. Update `upstream/RESPLE` to the new revision.
3. Re-run CMake configure: a patch that fails to apply means upstream has
   drifted under it -- resolve as an upstream API/behavior change, not a
   blind re-generation.
4. Rebuild, re-run `core/tests/synthetic.py` and `tests/test_run_resple.py`.
5. Re-run the parity/determinism checks above before trusting output.
6. Update the pin table and this file.

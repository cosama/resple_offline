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

Rather than hand-splitting that file, this bridge stages it with the small
patch series below and compiles it against a compat `rclcpp` shim that
structurally replaces the ROS graph:

- **Parameters**: `rclcpp::Node`'s param store is bridge-populated before
  RESPLE's constructor runs (`Node::set_parameter<T>`), typed exactly as
  `CommonUtils::readParam<T>` expects -- `std::any_cast<T>` fails silently on
  a type mismatch (e.g. `double` where upstream reads `float`), so
  `cpp/bindings.cpp`'s `apply_parameters`/`configure_lidar` set every key
  with its exact upstream C++ type rather than sniffing types from a generic
  dict.
- **Sensor input**: `create_subscription<T>()` is a no-op. Input is the
  benchmark's canonical/preprocessed sweep representation, and injection
  begins at RESPLE's internal `imu_int_buff` and
  `lidars_data[type].{pc_buff,t_buff}` buffers rather than at ROS sensor
  callbacks. `push_lidar` reproduces `ousterLidarCallback`'s per-point filter
  chain in the same order and against the same reference values: raw-index
  `point_filter_num`, blind range, the strictly-increasing absolute-stamp test
  against the previous frame's last *kept* point (upstream's `static int64_t
  last_t_ns`), ms relative-time encoding in `intensity`, reflectivity in
  `curvature`, and the Ouster `lidar_time_offset` shift. `blind` is read from
  the config RESPLE itself parsed, not passed in alongside it. Livox line/tag
  packet fields do not exist in the prepared schema and are not synthesized.
  Everything downstream -- `processData`'s PointData construction, IEKF
  update, and map maintenance -- remains upstream logic.
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

This avoids a hand-authored split of ROS I/O from the algorithm body.

## Patch series (`patches/integration/`)

| Patch | What |
|---|---|
| `01-bridge-visibility.patch` | Widens `RESPLE`'s `private:` section (buffers, `spline`, callbacks) to `public:` so `cpp/bindings.cpp` can inject sensor data and read spline state from outside the class. No behavior change. |
| `02-finish-condition.patch` | `processData()`'s loop is `while (true)` upstream -- real ROS nodes are killed, not joined, so there is no exit condition at all. Gives it a state-based one: the worker stops after the first pass that *began* with input closed and moved nothing. The flag is sampled at the top of the pass, never at the bottom, so a push landing after the pass scanned the buffers cannot be dropped. Also pops the raw queues under their own mutex, notifies the bridge on each dequeue (this is what releases producer backpressure), and calls the barrier from patch 03. |
| `03-deterministic-ikdtree-rebuild.patch` | Ports DALI-SLAM's compatible ikd-tree rebuild quiescence barrier and initializes the rebuild thread handle. The processing loop waits at each measurement-batch boundary so correspondence searches never run against an in-flight background rebuild. |

## Known limitations / open validation

- **Trajectory sampling**: RESPLE publishes incremental spline control points,
  so publication timing is not authoritative. After the worker fully drains
  and joins, the bridge resamples the finalized spline at its knot interval in
  the benchmark's world-to-body convention. This follows the finalized-spline
  strategy on upstream's `feature/benchmark` branch while retaining body
  rather than that branch's optional lidar-extrinsic output. Both ends of
  `[minTimeNs(), maxTimeNs()]` are excluded (`spline_pose_window` in
  `bindings.cpp`, shared with the live capture slot): the leading interval is
  backward extrapolation into the idle knots, and the trailing one never
  receives a measurement update because `collectMeasurements()` needs data
  beyond `maxTimeNs() + dt_ns` to form a batch. That trailing partial batch is
  reported as `metrics()["residual_points"]`.
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
- **Determinism**: three known scheduler-dependent inputs are closed off.
  (1) Completion is state-based -- input closure sampled at pass start, not a
  queue-depth observation plus grace sleep -- and producer backpressure waits
  on worker progress rather than a wall clock. (2) The ikd-tree rebuild
  barrier removes the correspondence/rebuild scheduling race (scope caveat in
  the patch header). (3) `push_lidar` refuses a first sweep preceded by fewer
  than 15 IMU samples: `initialization()` averages `std::min(15,
  imu_buff.size())` samples for gravity, and below 15 that count depends on
  how far the producer ran before the worker reached init, which would make
  the initial orientation -- and the whole trajectory -- scheduler-dependent.
  Batch composition itself is *not* timing-dependent: `collectMeasurements()`
  cuts on a spline-derived time window, and the OpenMP loops in `Estimator.h`
  are element-wise writes with serial assembly, so no FP-reduction ordering is
  involved. `core/tests/test_lifecycle.py` asserts bit-identical trajectories
  over 3 subprocess runs of the synthetic scene. Large real-data repeated
  replay is still recommended: that scene may not trigger large subtree
  rebuilds.

## Upgrade procedure

1. Record the currently pinned revision (above) and this patch series.
2. Update `upstream/RESPLE` to the new revision.
3. Re-run CMake configure: a patch that fails to apply means upstream has
   drifted under it -- resolve as an upstream API/behavior change, not a
   blind re-generation.
4. Rebuild, re-run `core/tests/synthetic.py` and `tests/test_run_resple.py`.
5. Re-run the parity/determinism checks above before trusting output.
6. Update the pin table and this file.

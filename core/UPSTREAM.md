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
- **Sensor input**: `create_subscription<T>()` is a no-op. Input is an offline
  sweep representation, and injection
  begins at RESPLE's internal `imu_int_buff` and
  `lidars_data[type].{pc_buff,t_buff}` buffers rather than at ROS sensor
  callbacks. One producer-side stream per configured LiDAR carries the state
  each upstream callback keeps in a function-local `static` (the cross-frame
  `last_t_ns`), which is per-type because upstream has one callback per type;
  `push_lidar(..., lidar=<name>)` selects it, and the name may be omitted only
  for a single-LiDAR session. `push_lidar` selects the configured upstream
  callback behavior:
  Ouster/Hesai/Mid360Boxi raw-index thinning and Mid70Avia/HAP360/AviaResple
  line/tag validation, valid-point thinning, first-point skip, and duplicate
  suppression, plus each callback's blind/time rules and Ouster-only
  `lidar_time_offset`. Raw Livox `line`/`tag` arrays are optional at the Python
  API for future direct bag replay. The current prepared Parquet schema lacks
  them and packet identity, so omission means "already driver-valid" and
  cannot reconstruct filtering discarded during preparation.
  Everything downstream -- `processData`'s PointData construction, IEKF
  update, and map maintenance -- remains upstream logic. Bridge-only ticket
  sidecars run in lockstep with the raw-cloud, downsampled-point, and estimator
  batch queues; they do not alter `PointData` or any estimator API. A sweep
  ticket completes only after every surviving point has completed its estimator
  batch (or initial-map build), or upstream has definitively discarded it as
  older than the active spline window.
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
- **Explicit synchronization**: each worker pass holds
  `bridge_processing_mutex` and publishes the sensor-submission epoch only
  after raw input is drained and initialization/measurement collection is
  blocked on more input. After publication the worker releases the pass mutex;
  if `synchronize()` has requested that epoch, it waits for the host to acquire
  and release the boundary before starting another pass. Without an explicit
  request this hook returns immediately. This handoff is required because the
  worker's short repeated passes can otherwise continually reacquire the mutex
  ahead of the awakened host, especially while upstream sleeps inside
  `collectMeasurements()`. `synchronize()` snapshots an epoch, waits for its
  stable-blocked publication, excludes the worker with the same mutex, and
  only then waits for a pending ikd-tree rebuild. `finish()` closes and joins
  the worker before placing the same tree barrier. The barrier is intentionally
  absent from ordinary estimator batches: deterministic stepwise replay calls
  `synchronize()` between LiDAR submissions, while clients that run freely
  retain upstream's background-rebuild scheduling.

This avoids a hand-authored split of ROS I/O from the algorithm body.

## Configuration compatibility

The Python resolver accepts upstream ROS2 parameter files directly, including
the `/** -> ros__parameters`, named `lidars` profiles, transport topics, and
`if_lidar_only` layout. The legacy flat benchmark config remains accepted.
Explicit profiles are authoritative; metadata/URDF fallback is used only to
synthesize a profile for a legacy config without `lidars`. A profile mapping
that no `lidars` entry selects is rejected (upstream's own
`config_jungfraujoch_tunnel_small.yaml` keeps a `livox:` block while
selecting only `["hesai"]`, and upstream simply never reads it): a stale
profile is reported by name rather than ignored.

Multi-LiDAR configurations are supported: upstream fuses every configured
LiDAR into one spline (its own `config_heap_testsite_hoenggerberg.yaml`
selects `["livox", "hesai"]`), and the bridge carries them through as
independent producer streams. Two profiles of the *same* `lidar_type` are
refused by both layers -- upstream's `lidars`/`lidars_data` are `std::map`s
keyed by type, so `emplace` would silently drop the second profile and fuse
its sweeps into the first one's buffers with the wrong extrinsics. Every
configured LiDAR must actually be fed: `collectMeasurements()` forms a batch
only once *all* of them have buffered points, so a starved stream stalls the
estimator rather than degrading to the others.
`metrics()["per_lidar"]` reports each stream's sweep and point counts so that
is visible. `scripts/run_resple.py` rejects a multi-LiDAR config outright,
because a prepared dataset carries a single `points.parquet` and there is no
second stream to feed.

`resolve()` also fills the `/offline/<name>` and `/offline/imu` topic
placeholders, so `report()`, the materialized upstream YAML and the native
run all name the same topic instead of native inventing a fallback for an
empty string nobody chose (ARCHITECTURE.md #7). They are placeholders: set
real topics before handing a materialized config to upstream RESPLE.

In LiDAR-only mode `push_imu` raises rather than accepting samples upstream
would never consume -- `processData`'s IMU drain is behind
`if (!if_lidar_only ...)`, so they would accumulate unboundedly.

## Patch series (`patches/integration/`)

One patch per upstream file, named after it. This keeps the mapping from
"upstream file changed" to "patch to re-derive" one-to-one when the pin moves.

A patch therefore carries several unrelated concerns. So that grouping by file
does not hide the "why", **every hunk explains itself in the source**, tagged
`resple_bridge:`. After staging, `grep -rn 'resple_bridge:' build/upstream_staged/`
enumerates the entire local delta -- added calls and their rationale -- in the
file a reader is actually compiling. The patch headers are only an index; the
reasoning has a single home, at the sites, so the two cannot drift apart.

Hunks are generated with `diff -U3` against the pinned source and never
hand-trimmed, and CMake applies them with `-F0`. Fuzz is what lets `patch`
place a hunk whose context no longer matches, which is precisely the silent
drift the staged copy exists to catch -- and because `patch` charges *missing*
context against the same fuzz budget, a hunk with fewer than three context
lines per side can only ever apply by fuzzing.

| Patch | Upstream file(s) | What |
|---|---|---|
| `01-resple-node.patch` | `src/RESPLE.cpp` | Seven grouped changes, enumerated in the patch header: (1) widens `private:` to `public:` so `cpp/bindings.cpp` can inject sensor data and read spline state; (2) gives `processData()`'s `while (true)` loop a state-based exit and stable-blocked epoch publication, with each pass guarded by `bridge_processing_mutex`; (3) pops raw queues under their own mutex, notifies producer backpressure, and carries global sweep tickets through lockstep raw/point/batch sidecars (checked with `resple_bridge::check_invariant`, not `assert`: the shipped build is Release, so `assert` would be compiled out exactly where a desynced sidecar becomes undefined behavior), acknowledging initialization points only after `ikdtree.Build` and measurement points only after all work in their estimator batch; (4) three `initialization()` correctness fixes -- LIO waits for the fixed 15-sample gravity prefix, the initial map is built only from the *complete* fixed 100 ms window, and `propRCP(start_t_ns)` moves below both retry gates so it runs exactly once (on a spline that already covers `t` its whole effect is `cov_rcp += cov_sys`, so one call per retry pass scaled the initial covariance with a scheduling-dependent pass count); (5) fails immediately, with the relevant density settings, when a complete window holds fewer than the required 100 downsampled points; (6) initializes upstream's spline member to nullptr so a session that never reaches `initFilter` can finish and report no finalized pose safely; (7) guards upstream's unconditional `pt_meas.back()` in `processData()` -- `collectMeasurements()` returns true whenever it consumed anything, including a pass that discarded every point it popped as older than the active spline window, leaving `pt_meas` empty and the read out of bounds. |
| `02-ikd-tree.patch` | `include/ikd-Tree/ikd_Tree.{h,cpp}` | Exposes the compatible ikd-tree rebuild quiescence barrier used only at explicit `synchronize()`/`finish()` boundaries and initializes the rebuild thread handle. Declaration and definition are one change and neither compiles alone, so both files share a patch. |

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
  not done here). The current explicit-synchronization API supersedes the
  original acceptance-time IMU check used for that run: a final sweep is
  accepted and ticketed, `synchronize(ticket)` returns false when trailing IMU
  or LiDAR lookahead is absent, and `finish()` reports the incomplete suffix.
  The runner always closes the native worker in `finally`, including on an
  unrelated replay error.
  Still run the ARCHITECTURE.md validation ladder's bounded-real-data-parity
  step against a real ROS2 build (upstream's own `Dockerfile`) before
  trusting this for benchmark numbers.
- **Determinism**: three known scheduler-dependent inputs are closed off for
  deterministic stepwise replay, which places `synchronize()` boundaries
  between LiDAR submissions.
  (1) Completion is state-based -- input closure sampled at pass start, not a
  queue-depth observation plus grace sleep -- and producer backpressure waits
  on worker progress rather than a wall clock. Ticket completion additionally
  distinguishes raw dequeue from full estimator/discard completion. (2) The
  explicit synchronization boundary drains any pending ikd-tree rebuild before
  the next submitted sweep can enter correspondence search. Free-running use
  without those boundaries intentionally retains upstream scheduling (scope
  caveat in the patch header). (3) In LIO mode `initialization()` waits until
  `imu_buff` contains its fixed 15-sample gravity prefix. `push_lidar` may
  accept and queue a sweep before then; `synchronize(ticket)` reaches a stable
  incomplete boundary and returns false rather than initializing from zero or
  a scheduler-dependent short prefix. Once the missing IMU is submitted,
  initialization averages exactly 15 samples, making the initial orientation
  independent of how far the producer ran before the worker reached it.
  Batch composition itself is *not* timing-dependent: `collectMeasurements()`
  cuts on a spline-derived time window, and the OpenMP loops in `Estimator.h`
  are element-wise writes with serial assembly, so no FP-reduction ordering is
  involved. Two more were found and closed when multi-LiDAR support was
  added: several LiDARs make `initialization()` take a scheduling-dependent
  number of passes, which exposed both (see `01-resple-node.patch`) --
  (4) the initial map was seeded from a partially arrived 100 ms window, and
  (5) `propRCP` inflated the initial covariance once per retry pass (2 to 6
  passes across runs on the two-LiDAR synthetic scene, giving 4 distinct
  trajectories). Neither changes the single-LiDAR result: that scene's
  trajectory hash is unchanged across the fix.
  `core/tests/test_lifecycle.py` asserts bit-identical trajectories
  over 3 subprocess runs of the synthetic scene, single- and two-LiDAR.
  Large real-data repeated replay is still recommended: that scene may not
  trigger large subtree rebuilds.

## Upgrade procedure

1. Record the currently pinned revision (above) and this patch series.
2. Update `upstream/RESPLE` to the new revision.
3. Re-run CMake configure: a patch that fails to apply means upstream has
   drifted under it -- resolve as an upstream API/behavior change, not a
   blind re-generation. Regenerate the patch by staging the pinned source,
   editing it, and running `diff -U3`; the configure step applies with `-F0`,
   so hand-trimmed context will be rejected outright.
4. Rebuild, re-run `core/tests/synthetic.py` and `tests/test_run_resple.py`.
5. Re-run the parity/determinism checks above before trusting output.
6. Update the pin table and this file.

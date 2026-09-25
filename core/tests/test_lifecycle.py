"""Construction/lifecycle/determinism tests against the compiled _core module.

Requires the extension to be built (`uv sync` from the repo root, or a
manual `cmake --build`). Each session is a fresh subprocess: RESPLE's
estimator state (the ikd-tree) is process-global and never reset, so only
one RespleOdometry may run per process (see cpp/bindings.cpp).
"""

from __future__ import annotations

import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

CORE_DIR = Path(__file__).resolve().parents[1]

pytest.importorskip("resple._core", reason="build _core first (uv sync)")

_SESSION_SCRIPT = textwrap.dedent("""
    import hashlib
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import run_stationary_session

    cfg = cfgmod.resolve({
        "lidars": [{
            "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
            "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
        }],
    })
    odom = RespleOdometry(cfg)
    result = run_stationary_session(odom)
    traj = np.ascontiguousarray(result["trajectory"], dtype=np.float64)
    metrics = result["metrics"]
    print("POSES", traj.shape[0])
    print("FINITE", bool(np.isfinite(traj).all()))
    print("RESIDUAL", metrics["residual_sweeps"], metrics["residual_points"])
    # Compare the complete trajectory.
    print("TRAJ", hashlib.sha256(traj.tobytes()).hexdigest())
""")

_SPARSE_INITIAL_MAP_SCRIPT = textwrap.dedent("""
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import run_stationary_session

    cfg = cfgmod.resolve({
        "ds_scan_voxel": 100.0,
        "lidars": [{
            "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
            "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
        }],
    })
    odom = RespleOdometry(cfg)
    run_stationary_session(odom)
""")

_SPARSE_PREFIX_SCRIPT = textwrap.dedent("""
    import numpy as np
    from collections import deque
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import box_room_sweep, GRAVITY, IMU_LEAD_SECONDS

    cfg = cfgmod.resolve({"lidars": [{
        "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
        "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
    }]})
    odom = RespleOdometry(cfg)
    position = np.zeros(3)
    unresolved = deque()
    duration = 2.0
    imu_hz = 200.0
    lidar_hz = 10.0
    next_lidar_t = 0.0
    sweep_index = 0
    n_imu = int(duration * imu_hz) + 50
    for i in range(n_imu):
        stamp = i / imu_hz
        odom.push_imu(stamp, [0.0, 0.0, GRAVITY], [0.0, 0.0, 0.0])
        if i == 14:
            assert odom.synchronize()
        while next_lidar_t <= stamp - IMU_LEAD_SECONDS and next_lidar_t < duration:
            if sweep_index < 5:
                # Sparse sweep: only 5 points
                points = np.zeros((5, 3), dtype=np.float32)
                points[:, 0] = np.linspace(1.0, 2.0, 5)
                relative_times = np.zeros(5, dtype=np.float64)
            else:
                # Dense sweep
                points, relative_times = box_room_sweep(
                    position, n_azimuth=120, n_rings=8, seed=sweep_index
                )
            unresolved.append(odom.push_lidar(next_lidar_t, points, relative_times, lidar="lidar0"))
            sweep_index += 1
            next_lidar_t += 1.0 / lidar_hz
            while unresolved and odom.synchronize(unresolved[0]):
                unresolved.popleft()
    res = odom.finish()
    print("POSES", res["trajectory"].shape[0])
    print("FINITE", bool(np.isfinite(res["trajectory"]).all()))
""")

_LIDAR_ONLY_SCRIPT = textwrap.dedent("""
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import run_stationary_session

    cfg = cfgmod.resolve({
        "if_lidar_only": True,
        "lidars": [{
            "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
            "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
        }],
    })
    result = run_stationary_session(RespleOdometry(cfg), with_imu=False)
    print("POSES", result["trajectory"].shape[0])
    print("FINITE", bool(np.isfinite(result["trajectory"]).all()))
""")

_SYNCHRONIZATION_SCRIPT = textwrap.dedent("""
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import box_room_sweep

    cfg = cfgmod.resolve({"lidars": [{
        "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
        "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
    }]})
    odom = RespleOdometry(cfg)
    points0, times0 = box_room_sweep(np.zeros(3), seed=0)
    ticket1 = odom.push_lidar(0.0, points0, times0)
    print("TICKET", ticket1)
    print("NO_IMU", odom.synchronize(ticket1))
    for i in range(15):
        odom.push_imu(i / 200.0, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
    print("BLOCKED_SNAPSHOT", odom.synchronize())
    print("NO_LOOKAHEAD", odom.synchronize(ticket1))
    points1, times1 = box_room_sweep(np.zeros(3), seed=1)
    print("TICKET2", odom.push_lidar(0.1, points1, times1))
    print("FIRST_DONE", odom.synchronize(ticket1))
    result = odom.finish()
    print("LATEST", result["metrics"]["latest_ticket"])
    print("INCOMPLETE", result["metrics"]["incomplete_tickets"])
""")

_INITIALIZATION_BOUNDARY_SCRIPT = textwrap.dedent("""
    from resple import RespleOdometry, config as cfgmod
    cfg = cfgmod.resolve({"lidars": [{
        "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
        "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
    }]})
    odom = RespleOdometry(cfg)
    for i in range(15):
        odom.push_imu(i / 200.0, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
    print("BOUNDARY", odom.synchronize())
    print("LATEST_POSE", odom.latest_pose())
    odom.finish()
""")

_TICKET_HOLE_SCRIPT = textwrap.dedent("""
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import box_room_sweep

    cfg = cfgmod.resolve({"lidars": [
        {"name": "front", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
         "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01},
        {"name": "side", "lidar_type": "Hesai", "scan_line": 32, "blind": 0.3,
         "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01},
    ]})
    odom = RespleOdometry(cfg)
    for i in range(15):
        odom.push_imu(i / 200.0, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
    assert odom.synchronize()
    points, times = box_room_sweep(np.zeros(3), seed=0)
    first = odom.push_lidar(0.0, points, times, lidar="front")
    # Preserve earlier ticket gaps.
    empty = odom.push_lidar(
        0.0, np.asarray([[0.01, 0.0, 0.0]], np.float32),
        np.asarray([0.0], np.float64), lidar="side",
    )
    print("TICKETS", first, empty)
    print("STARVED", odom.synchronize(empty))
    print("PREFIX", odom.metrics()["completed_ticket"])
    result = odom.finish()
    print("INCOMPLETE", result["metrics"]["incomplete_tickets"])
""")

_FILTERED_SWEEP_SCRIPT = textwrap.dedent("""
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    cfg = cfgmod.resolve({"if_lidar_only": True, "lidars": [{
        "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 1.0,
        "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
    }]})
    odom = RespleOdometry(cfg)
    ticket = odom.push_lidar(
        0.0, np.asarray([[0.01, 0.0, 0.0]], np.float32), np.asarray([0.0], np.float64)
    )
    print("DONE", odom.synchronize(ticket))
    result = odom.finish()
    print("INCOMPLETE", result["metrics"]["incomplete_tickets"])
""")


def _run_session() -> str:
    proc = subprocess.run(
        [sys.executable, "-c", _SESSION_SCRIPT],
        cwd=CORE_DIR, capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


def test_stationary_session_commits_finite_poses():
    output = _run_session()
    lines = dict(line.split(" ", 1) for line in output.splitlines() if " " in line)
    assert int(lines["POSES"]) > 0, "expected at least one committed pose"
    assert lines["FINITE"] == "True"


def test_stationary_session_drains_every_pushed_sweep():
    """A residual sweep would mean the worker stopped with input still queued."""
    lines = dict(line.split(" ", 1) for line in _run_session().splitlines() if " " in line)
    residual_sweeps, _residual_points = lines["RESIDUAL"].split()
    assert residual_sweeps == "0", "worker stopped with sweeps still buffered"


def test_stationary_session_is_deterministic():
    runs = [_run_session() for _ in range(3)]
    assert len(set(runs)) == 1, "identical synthetic input produced different output"


def test_sparse_initial_map_fails_with_actionable_error():
    proc = subprocess.run(
        [sys.executable, "-c", _SPARSE_INITIAL_MAP_SCRIPT],
        cwd=CORE_DIR, capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode != 0
    assert "initial-map initialization cannot proceed" in proc.stderr
    assert "first 100 ms contains" in proc.stderr
    assert "ds_scan_voxel=100.000000" in proc.stderr
    assert "point_filter_num=1" in proc.stderr


def test_sparse_prefix_is_skipped_and_initializes_on_dense_data():
    proc = subprocess.run(
        [sys.executable, "-c", _SPARSE_PREFIX_SCRIPT],
        cwd=CORE_DIR, capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode == 0, proc.stderr
    assert "Skipping sparse window(s) and retrying" in proc.stderr
    assert "RESPLE initial map successfully initialized" in proc.stderr
    lines = dict(line.split(" ", 1) for line in proc.stdout.splitlines() if " " in line)
    assert int(lines["POSES"]) > 0
    assert lines["FINITE"] == "True"


def test_upstream_lidar_only_mode_commits_finite_poses():
    proc = subprocess.run(
        [sys.executable, "-c", _LIDAR_ONLY_SCRIPT], cwd=CORE_DIR,
        capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode == 0, proc.stderr
    lines = dict(line.split(" ", 1) for line in proc.stdout.splitlines() if " " in line)
    assert int(lines["POSES"]) > 0
    assert lines["FINITE"] == "True"


def _run_inline(script: str, timeout: int = 60) -> dict[str, str]:
    proc = subprocess.run(
        [sys.executable, "-u", "-c", script], cwd=CORE_DIR,
        capture_output=True, text=True, timeout=timeout,
    )
    assert proc.returncode == 0, proc.stderr
    return dict(line.split(" ", 1) for line in proc.stdout.splitlines() if " " in line)


def test_explicit_synchronization_reports_missing_input_then_completes_prefix():
    lines = _run_inline(_SYNCHRONIZATION_SCRIPT)
    assert lines["TICKET"] == "1"
    assert lines["NO_IMU"] == "False"
    assert lines["BLOCKED_SNAPSHOT"] == "False"
    assert lines["NO_LOOKAHEAD"] == "False"
    assert lines["TICKET2"] == "2"
    assert lines["FIRST_DONE"] == "True"
    assert lines["LATEST"] == "2"


def test_initialization_only_synchronization_absorbs_imu_before_first_sweep():
    lines = _run_inline(_INITIALIZATION_BOUNDARY_SCRIPT)
    assert lines["BOUNDARY"] == "True"
    assert lines["LATEST_POSE"] == "None"


def test_global_completed_watermark_does_not_skip_a_starved_stream_hole():
    lines = _run_inline(_TICKET_HOLE_SCRIPT)
    assert lines["TICKETS"] == "1 2"
    assert lines["STARVED"] == "False"
    assert lines["PREFIX"] == "0"
    assert int(lines["INCOMPLETE"]) == 2  # neither is in the completed public prefix


def test_fully_filtered_sweep_completes_at_raw_ingestion():
    lines = _run_inline(_FILTERED_SWEEP_SCRIPT)
    assert lines["DONE"] == "True"
    assert lines["INCOMPLETE"] == "0"


# Exercise producer backpressure notifications.
_BACKPRESSURE_SCRIPT = textwrap.dedent("""
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import box_room_sweep, GRAVITY

    cfg = cfgmod.resolve({"lidars": [{
        "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
        "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
    }]})
    odom = RespleOdometry(cfg, max_pending_sweeps=1)
    next_t, index = 0.0, 0
    for i in range(350):
        stamp = i / 200.0
        odom.push_imu(stamp, [0.0, 0.0, GRAVITY], [0.0, 0.0, 0.0])
        while next_t <= stamp - 0.1 and next_t < 1.5:
            points, relative_times = box_room_sweep(
                np.zeros(3), n_azimuth=120, n_rings=8, seed=index,
            )
            odom.push_lidar(next_t, points, relative_times)
            index += 1
            next_t += 0.1
    result = odom.finish()
    print("POSES", result["trajectory"].shape[0])
    print("SWEEPS", result["metrics"]["lidar_sweeps_pushed"])
    print("RESIDUAL", result["metrics"]["residual_sweeps"])
""")


def test_backpressure_wait_is_released_by_worker_progress():
    lines = _run_inline(_BACKPRESSURE_SCRIPT, timeout=120)
    assert int(lines["SWEEPS"]) == 15, "producer did not get through its sweeps"
    assert int(lines["POSES"]) > 0
    assert lines["RESIDUAL"] == "0"


_IMU_BATCH_SCRIPT = textwrap.dedent("""
    from resple import RespleOdometry, config as cfgmod

    cfg = cfgmod.resolve({"lidars": [{
        "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
        "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
    }]})
    odom = RespleOdometry(cfg)
    samples = [
        (i / 200.0, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
        for i in range(20)
    ]
    print("COUNT", odom.push_imu_batch(samples))
    print("SYNC", odom.synchronize())
    print("FINISH", odom.finish()["metrics"]["imu_samples"])
""")


def test_imu_batch_commits_complete_interval_and_synchronizes():
    lines = _run_inline(_IMU_BATCH_SCRIPT)
    assert lines["COUNT"] == "20"
    assert lines["SYNC"] == "True"
    assert lines["FINISH"] == "20"


# Configure distinct LiDAR types.
_MULTI_LIDAR_SCRIPT = textwrap.dedent("""
    import hashlib
    import json
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import run_stationary_multi_lidar_session, second_lidar_extrinsics

    q_lb, t_lb = second_lidar_extrinsics()
    cfg = cfgmod.resolve({
        "lidars": [
            {"name": "front", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
             "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01},
            {"name": "side", "lidar_type": "Hesai", "scan_line": 32, "blind": 0.3,
             "q_lb": q_lb, "t_lb": t_lb, "w_pt": 0.01},
        ],
    })
    odom = RespleOdometry(cfg)
    result = run_stationary_multi_lidar_session(odom)
    traj = np.ascontiguousarray(result["trajectory"], dtype=np.float64)
    metrics = result["metrics"]
    print("POSES", traj.shape[0])
    print("FINITE", bool(np.isfinite(traj).all()))
    print("RESIDUAL", metrics["residual_sweeps"])
    # Extrinsics preserve the stationary origin.
    print("MAXDIST", float(np.abs(traj[:, 1:4]).max()))
    print("PERLIDAR", json.dumps({
        name: entry["sweeps_pushed"] for name, entry in metrics["per_lidar"].items()
    }, sort_keys=True))
    print("TRAJ", hashlib.sha256(traj.tobytes()).hexdigest())
""")

_SAME_TYPE_SCRIPT = textwrap.dedent("""
    from resple import RespleOdometry, config as cfgmod

    cfg = cfgmod.resolve({
        "lidars": [
            {"name": "front", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
             "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01},
            {"name": "rear", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
             "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (1.0, 0.0, 0.0), "w_pt": 0.01},
        ],
    })
    print("PROBLEMS", "|".join(cfg.validate()))
    RespleOdometry(cfg)
""")


def _run_multi_lidar() -> dict:
    proc = subprocess.run(
        [sys.executable, "-c", _MULTI_LIDAR_SCRIPT],
        cwd=CORE_DIR, capture_output=True, text=True, timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    return dict(line.split(" ", 1) for line in proc.stdout.splitlines() if " " in line)


def test_two_lidars_fuse_into_one_trajectory():
    lines = _run_multi_lidar()
    assert int(lines["POSES"]) > 0, "expected at least one committed pose"
    assert lines["FINITE"] == "True"
    assert lines["RESIDUAL"] == "0", "worker stopped with sweeps still buffered"
    # Require both input streams.
    import json as _json
    per_lidar = _json.loads(lines["PERLIDAR"])
    assert set(per_lidar) == {"front", "side"}
    assert all(count > 0 for count in per_lidar.values()), per_lidar
    # Bound extrinsic compensation error.
    assert float(lines["MAXDIST"]) < 0.05, "stationary rig drifted; check extrinsics handling"


def test_two_lidars_are_deterministic():
    """Bit-exact over the whole trajectory, like the single-LiDAR case.

    Two LiDARs make initialization take a scheduling-dependent number of
    passes, which is what exposed propRCP's per-call covariance inflation
    (see 01-resple-node.patch); a MAXDIST-only check would have caught
    it here only by luck.
    """
    runs = [_run_multi_lidar()["TRAJ"] for _ in range(3)]
    assert len(set(runs)) == 1, "identical two-lidar input produced different output"


def test_two_lidars_of_the_same_type_are_rejected():
    """Upstream's lidars/lidars_data maps are keyed by type, not by name.

    A second profile of the same type is dropped by std::map::emplace, so its
    sweeps would be silently fused into the first one's buffers with the wrong
    extrinsics. Both layers must say so by name.
    """
    proc = subprocess.run(
        [sys.executable, "-c", _SAME_TYPE_SCRIPT],
        cwd=CORE_DIR, capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode != 0
    problems = next(line for line in proc.stdout.splitlines() if line.startswith("PROBLEMS"))
    assert "both use lidar_type 'Ouster'" in problems, problems
    assert "invalid RespleConfig" in proc.stderr


# Exercise sparse spline extension.
_EXTENSION_COVERAGE_SCRIPT = textwrap.dedent("""
    import json
    from collections import deque
    import numpy as np
    from resple import RespleOdometry, config as cfgmod
    from tests.synthetic import box_room_sweep, GRAVITY, IMU_LEAD_SECONDS

    SWEEP_SPAN = 0.01
    cfg = cfgmod.resolve({
        "knot_hz": 20,
        "num_points_upd": 2000,
        "lidars": [{
            "name": "lidar0", "lidar_type": "Ouster", "scan_line": 64, "blind": 0.3,
            "q_lb": (1.0, 0.0, 0.0, 0.0), "t_lb": (0.0, 0.0, 0.0), "w_pt": 0.01,
        }],
    })
    odom = RespleOdometry(cfg, max_pending_sweeps=64)
    duration, imu_hz, lidar_hz = 1.5, 200.0, 10.0
    next_t, index, unresolved, lags = 0.0, 0, deque(), []
    for i in range(int(duration * imu_hz) + 50):
        stamp = i / imu_hz
        odom.push_imu(stamp, [0.0, 0.0, GRAVITY], [0.0, 0.0, 0.0])
        if i == 14:
            assert odom.synchronize(), "initialization IMU prefix did not synchronize"
        while next_t <= stamp - IMU_LEAD_SECONDS and next_t < duration:
            points, relative_times = box_room_sweep(
                np.zeros(3), n_azimuth=120, n_rings=8, seed=index,
            )
            unresolved.append((
                odom.push_lidar(next_t, points, relative_times * (SWEEP_SPAN / 0.1)),
                index,
            ))
            index += 1
            next_t += 1.0 / lidar_hz
            # Measure quiescent ticket lag.
            while unresolved and odom.synchronize(unresolved[0][0]):
                lags.append(index - 1 - unresolved.popleft()[1])
    metrics = odom.finish()["metrics"]
    print("LAGS", json.dumps(lags))
    print("RESIDUAL", metrics["residual_sweeps"], metrics["residual_points"])
""")


def _run_extension_coverage() -> dict:
    proc = subprocess.run(
        [sys.executable, "-c", _EXTENSION_COVERAGE_SCRIPT],
        cwd=CORE_DIR, capture_output=True, text=True, timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    return dict(line.split(" ", 1) for line in proc.stdout.splitlines() if " " in line)


def test_spline_extension_preserves_upstream_lidar_batch_boundary():
    """Complete atomic sweeps retain upstream's LiDAR readiness behavior.

    In this scene a sweep spans 10 ms and `dt_ns` is 50 ms. An added extension
    horizon would force every sweep to wait for a later one and regroup points
    across sweep boundaries. The offline producer instead submits each sweep
    atomically and synchronizes at a stable boundary, so at least one sweep is
    processed without artificial LiDAR lookahead (zero lag).
    """
    import json

    lines = _run_extension_coverage()
    lags = json.loads(lines["LAGS"])
    assert lags, "no sweep was consumed at all"
    assert min(lags) == 0, f"every batch was delayed across a sweep boundary: lags={lags}"


def test_spline_extension_drains_finite_tail_after_input_closes():
    """EOF drains every tail point that upstream readiness can process.

    Once input is explicitly closed, the worker keeps forming upstream batches
    until a complete pass makes no progress; only then may it exit.
    """
    lines = _run_extension_coverage()
    residual_sweeps, residual_points = lines["RESIDUAL"].split()
    assert residual_sweeps == "0", "a raw sweep was never even dequeued"
    assert residual_points == "0", "an upstream-processable tail was left unconsumed"

"""Profile-specific raw sweep preprocessing tests.

Each case is replayed in its own subprocess: upstream's estimator state (the
ikd-tree map) is process-global, so a process hosts exactly one session.

Cases are chosen to *discriminate*, not merely to pass -- several of the
behaviors reproduced in bindings.cpp' push_lidar differ between profiles only
in where a predicate sits relative to the stride counter, and a test whose
expected count is the same either way would not notice them regressing.
"""

from __future__ import annotations

import json
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

pytest.importorskip("resple._core", reason="build _core first (uv sync)")

CORE_DIR = Path(__file__).resolve().parents[1]

SCRIPT = textwrap.dedent("""
    import json
    import sys
    import numpy as np
    from resple import RespleOdometry, config

    case = json.loads(sys.argv[1])
    cfg = config.resolve({
        "if_lidar_only": True,
        "point_filter_num": case.get("point_filter_num", 1),
        "lidars": [{
            "name": "lidar0", "lidar_type": case["lidar_type"],
            "scan_line": case.get("scan_line", 4), "blind": case.get("blind", 1.0),
            "q_lb": [1.0, 0.0, 0.0, 0.0], "t_lb": [0.0, 0.0, 0.0],
        }],
    })
    odom = RespleOdometry(cfg)
    for sweep in case["sweeps"]:
        kwargs = {}
        if "lines" in sweep:
            kwargs["lines"] = np.asarray(sweep["lines"], dtype=np.int32)
        if "tags" in sweep:
            kwargs["tags"] = np.asarray(sweep["tags"], dtype=np.int32)
        odom.push_lidar(
            sweep.get("stamp", 1.0), np.asarray(sweep["points"], dtype=np.float32),
            np.asarray(sweep["times"], dtype=np.float64), **kwargs,
        )
    print("KEPT", odom.status()["lidar_points_after_preprocessing"])
    del odom
""")


def kept(case: dict) -> int:
    """Total points surviving preprocessing across every sweep in `case`."""
    if "sweeps" not in case:
        case = {**case, "sweeps": [{k: case[k] for k in ("points", "times") if k in case}
                                   | {k: case[k] for k in ("lines", "tags") if k in case}]}
    proc = subprocess.run(
        [sys.executable, "-c", SCRIPT, json.dumps(case)], cwd=CORE_DIR,
        capture_output=True, text=True, timeout=20,
    )
    assert proc.returncode == 0, proc.stderr
    line = next(line for line in proc.stdout.splitlines() if line.startswith("KEPT "))
    return int(line.split()[1])


def test_ouster_uses_raw_index_stride_blind_and_cross_frame_time():
    assert kept({
        "lidar_type": "Ouster", "point_filter_num": 2, "blind": 1.0,
        "points": [[2, 0, 0], [3, 0, 0], [0.5, 0, 0], [3, 0, 0], [4, 0, 0]],
        "times": [0.0, 0.01, 0.02, 0.03, 0.04],
    }) == 1


@pytest.mark.parametrize("lidar_type", ["Mid70Avia", "HAP360", "AviaResple"])
def test_livox_profiles_honor_line_tag_and_valid_point_stride(lidar_type):
    assert kept({
        "lidar_type": lidar_type, "point_filter_num": 2, "scan_line": 2,
        "points": [[2.0 + i * 0.1, 0, 0] for i in range(6)],
        "times": [i * 0.01 for i in range(6)],
        "lines": [0, 0, 5, 0, 0, 0],
        "tags": [0, 0, 0, 0x30, 0, 0],
    }) == 1


def test_livox_profiles_suppress_consecutive_duplicate_candidates():
    assert kept({
        "lidar_type": "HAP360",
        "points": [[2, 0, 0], [2, 0, 0], [2.1, 0, 0], [2.1, 0, 0]],
        "times": [0.0, 0.01, 0.02, 0.03],
        "lines": [0, 0, 0, 0], "tags": [0, 0, 0, 0],
    }) == 1


@pytest.mark.parametrize("lidar_type", ["Hesai", "Mid360Boxi"])
def test_timestamped_pointcloud_profiles_use_raw_index_stride_and_blind(lidar_type):
    assert kept({
        "lidar_type": lidar_type, "point_filter_num": 2, "blind": 1.0,
        "points": [[2, 0, 0], [3, 0, 0], [4, 0, 0], [3, 0, 0], [0.5, 0, 0]],
        "times": [0.0, 0.01, 0.02, 0.03, 0.04],
    }) == 1


def test_lidar_only_mode_requires_no_imu_samples():
    assert kept({
        "lidar_type": "Ouster", "blind": 0.1,
        "points": [[2, 0, 0], [3, 0, 0]], "times": [0.0, 0.01],
    }) == 1


# Test profile predicate order.

_STRIDE_DIVERGENCE = {
    # Reject the zero-offset point.
    "point_filter_num": 2, "scan_line": 8, "blind": 0.1,
    "points": [[2.0, 0, 0], [2.1, 0, 0], [2.2, 0, 0], [2.3, 0, 0], [2.4, 0, 0]],
    "times": [0.0, 0.0, 0.01, 0.02, 0.03],
    "lines": [0, 0, 0, 0, 0], "tags": [0, 0, 0, 0, 0],
}


@pytest.mark.parametrize("lidar_type", ["Mid70Avia", "HAP360"])
def test_mid70avia_and_hap360_count_time_rejected_points_toward_the_stride(lidar_type):
    # Rejected points advance stride.
    assert kept({**_STRIDE_DIVERGENCE, "lidar_type": lidar_type}) == 2


def test_avia_resple_excludes_time_rejected_points_from_the_stride():
    # Timestamp rejection precedes stride.
    assert kept({**_STRIDE_DIVERGENCE, "lidar_type": "AviaResple"}) == 1


def test_livox_duplicate_reference_advances_over_rejected_candidates():
    # Rejected candidates remain duplicate references.
    assert kept({
        "lidar_type": "HAP360", "point_filter_num": 1, "blind": 1.0,
        "points": [[5, 0, 0], [2, 0, 0], [2, 0, 0], [3, 0, 0]],
        "times": [0.0, 0.0, 0.01, 0.02],
        "lines": [0, 0, 0, 0], "tags": [0, 0, 0, 0],
    }) == 1


# Test cross-frame timestamps.

def test_cross_frame_timestamp_filter_carries_over_between_sweeps():
    # Preserve the prior frame threshold.
    assert kept({
        "lidar_type": "Ouster", "blind": 0.1, "sweeps": [
            {"stamp": 1.0, "points": [[2, 0, 0], [3, 0, 0]], "times": [0.0, 0.05]},
            {"stamp": 1.02, "points": [[4, 0, 0], [5, 0, 0], [6, 0, 0]],
             "times": [0.0, 0.01, 0.05]},
        ],
    }) == 2


def test_sweep_that_keeps_nothing_rewinds_last_t_ns_to_its_own_start():
    # Empty frames reset the threshold.
    assert kept({
        "lidar_type": "Ouster", "blind": 0.1, "sweeps": [
            {"stamp": 1.0, "points": [[2, 0, 0], [3, 0, 0]], "times": [0.0, 0.05]},
            {"stamp": 1.01, "points": [[0.01, 0, 0], [0.01, 0, 0]], "times": [0.0, 0.01]},
            {"stamp": 1.02, "points": [[4, 0, 0], [5, 0, 0]], "times": [0.0, 0.005]},
        ],
    }) == 3

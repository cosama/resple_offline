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
    # Bit-exact over the whole trajectory, not just its last row: a scheduling
    # difference early in the run can be re-absorbed by later knots.
    print("TRAJ", hashlib.sha256(traj.tobytes()).hexdigest())
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

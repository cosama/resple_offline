"""Thin Python wrapper around the compiled RESPLE offline bridge (_core)."""

from __future__ import annotations

import numpy as np

from . import config as config  # re-exported
from ._core import RespleOdometry as _RespleOdometry

__all__ = ["RespleOdometry", "config"]


class RespleOdometry:
    """Runs one RESPLE session for the lifetime of this object.

    Upstream's estimator state (the ikd-tree map) is process-global and never
    reset, so exactly one session may run per process -- construct a second
    one and the native layer raises. Run one dataset (or segment) per
    subprocess.
    """

    def __init__(self, cfg: "config.RespleConfig"):
        problems = cfg.validate()
        if problems:
            raise ValueError("invalid RespleConfig: " + "; ".join(problems))
        lidar = cfg.lidars[0]
        self._cfg = cfg
        self._native = _RespleOdometry(
            parameters=cfg.native_parameters(),
            lidar_name=lidar.name,
            lidar_type=lidar.lidar_type,
            scan_line=lidar.scan_line,
            blind=lidar.blind,
            topic_lidar=lidar.topic_lidar,
            q_lb=list(lidar.q_lb),
            t_lb=list(lidar.t_lb),
            w_pt=lidar.w_pt,
            max_pending_sweeps=cfg.max_pending_sweeps,
            min_imu_lead_seconds=cfg.min_imu_lead_seconds,
        )

    def push_imu(self, timestamp: float, acceleration, angular_velocity) -> int:
        """Push one IMU sample. Timestamps must be strictly increasing.

        Raises ValueError in LiDAR-only mode (``if_lidar_only=True``):
        upstream never drains its IMU buffer there, so samples pushed in that
        mode would accumulate without ever reaching the estimator.
        """
        return self._native.push_imu(
            float(timestamp), list(map(float, acceleration)), list(map(float, angular_velocity))
        )

    def push_lidar(
        self,
        timestamp: float,
        points: np.ndarray,
        relative_times: np.ndarray,
        *,
        lines: np.ndarray | None = None,
        tags: np.ndarray | None = None,
    ) -> int:
        """`points` is (N, 3) or (N, 4) [x, y, z, (intensity)]; `relative_times`
        is (N,) seconds from the source packet header timestamp. Injection
        begins at RESPLE's internal cloud buffer after applying the configured
        upstream callback's profile-specific predicates.

        ``lines`` and ``tags`` carry the raw Livox packet fields. They may be
        omitted for canonical inputs whose invalid driver points have already
        been removed; direct bag replay should provide them.

        Raises ValueError mentioning "insufficient IMU lead" when the IMU
        pushed so far does not cover this sweep -- including for the first
        sweep, which additionally needs enough samples for RESPLE's gravity
        initialization to be reproducible.
        """
        points = np.ascontiguousarray(points, dtype=np.float32)
        relative_times = np.ascontiguousarray(relative_times, dtype=np.float64)
        if lines is not None:
            lines = np.ascontiguousarray(lines, dtype=np.int32)
        if tags is not None:
            tags = np.ascontiguousarray(tags, dtype=np.int32)
        return self._native.push_lidar(
            float(timestamp), points, relative_times, lines=lines, tags=tags
        )

    def trajectory(self) -> np.ndarray:
        """(N, 8): timestamp, x, y, z, qx, qy, qz, qw."""
        return self._native.trajectory()

    def drain_map_batches(self) -> list[np.ndarray]:
        return self._native.drain_map_batches()

    def metrics(self) -> dict:
        return self._native.metrics()

    def events(self) -> list[dict]:
        return self._native.events()

    def status(self) -> dict:
        return self._native.status()

    def finish(self) -> dict:
        """Close input, drain the worker, and return the finalized results.

        There is no timeout: completion is a state condition, so a deadline
        could only truncate a still-progressing replay.
        """
        return self._native.finish()

    def __enter__(self) -> "RespleOdometry":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if exc_type is None:
            self.finish()

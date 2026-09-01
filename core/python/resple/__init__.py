"""Thin Python wrapper around the compiled RESPLE offline bridge (_core)."""

from __future__ import annotations

import numpy as np

from . import config as config  # re-exported
from ._core import RespleOdometry as _RespleOdometry

__all__ = ["RespleOdometry", "config"]


class RespleOdometry:
    """Run one isolated RESPLE session."""

    def __init__(self, cfg: "config.RespleConfig", *, max_pending_sweeps: int = 8):
        """Configure upstream parameters and backpressure."""
        problems = cfg.validate()
        if problems:
            raise ValueError("invalid RespleConfig: " + "; ".join(problems))
        if int(max_pending_sweeps) < 1:
            raise ValueError("max_pending_sweeps must be at least 1")
        self._cfg = cfg
        self._native = _RespleOdometry(
            parameters=cfg.native_parameters(),
            lidars=[
                {
                    "name": lidar.name,
                    "lidar_type": lidar.lidar_type,
                    "topic_lidar": lidar.topic_lidar,
                    "scan_line": int(lidar.scan_line),
                    "blind": float(lidar.blind),
                    "q_lb": list(map(float, lidar.q_lb)),
                    "t_lb": list(map(float, lidar.t_lb)),
                    "w_pt": float(lidar.w_pt),
                }
                for lidar in cfg.lidars
            ],
            max_pending_sweeps=int(max_pending_sweeps),
        )

    @property
    def lidar_names(self) -> tuple[str, ...]:
        """Return configured LiDAR names."""
        return tuple(lidar.name for lidar in self._cfg.lidars)

    def push_imu(self, timestamp: float, acceleration, angular_velocity) -> int:
        """Push one ordered IMU sample."""
        return self._native.push_imu(
            float(timestamp), list(map(float, acceleration)), list(map(float, angular_velocity))
        )

    def push_imu_batch(self, samples) -> int:
        """Commit one ordered IMU interval."""
        rows = list(samples)
        return self._native.push_imu_batch(
            [float(row[0]) for row in rows],
            [list(map(float, row[1])) for row in rows],
            [list(map(float, row[2])) for row in rows],
        )

    def push_lidar(
        self,
        timestamp: float,
        points: np.ndarray,
        relative_times: np.ndarray,
        *,
        lines: np.ndarray | None = None,
        tags: np.ndarray | None = None,
        lidar: str | None = None,
    ) -> int:
        """Push one profiled LiDAR sweep.

        Points use XYZ or XYZI columns. Relative times use seconds. Raw Livox
        input should include lines and tags. Multi-LiDAR sessions require the
        profile name and input from every configured stream.
        """
        points = np.ascontiguousarray(points, dtype=np.float32)
        relative_times = np.ascontiguousarray(relative_times, dtype=np.float64)
        if lines is not None:
            lines = np.ascontiguousarray(lines, dtype=np.int32)
        if tags is not None:
            tags = np.ascontiguousarray(tags, dtype=np.int32)
        return self._native.push_lidar(
            float(timestamp), points, relative_times, lines=lines, tags=tags,
            lidar="" if lidar is None else str(lidar),
        )

    def synchronize(self, ticket: int | None = None) -> bool:
        """Drain input to a stable boundary.

        False indicates missing sensor lookahead. Without a ticket, the current
        submission snapshot defines the boundary.
        """
        return bool(self._native.synchronize(ticket))

    def trajectory(self) -> np.ndarray:
        """(N, 8): timestamp, x, y, z, qx, qy, qz, qw."""
        return self._native.trajectory()

    def latest_pose(self) -> np.ndarray | None:
        """Return the latest pose snapshot."""
        return self._native.latest_pose()

    def drain_map_batches(self) -> list[np.ndarray]:
        return self._native.drain_map_batches()

    def metrics(self) -> dict:
        return self._native.metrics()

    def events(self) -> list[dict]:
        return self._native.events()

    def status(self) -> dict:
        return self._native.status()

    def finish(self) -> dict:
        """Close input and finalize results."""
        return self._native.finish()

    def __enter__(self) -> "RespleOdometry":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if exc_type is None:
            self.finish()
            return
        try:
            self.finish()
        except Exception:
            pass

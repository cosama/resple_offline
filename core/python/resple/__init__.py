"""Thin Python wrapper around the compiled RESPLE offline bridge (_core)."""

from __future__ import annotations

import numpy as np

from . import config as config  # re-exported
from ._core import RespleOdometry as _RespleOdometry

__all__ = ["RespleOdometry", "config"]


class RespleOdometry:
    """Run one RESPLE session.

    The upstream map is process-global, so each session needs a fresh process.
    """

    def __init__(self, cfg: "config.RespleConfig"):
        problems = cfg.validate()
        if problems:
            raise ValueError("invalid RespleConfig: " + "; ".join(problems))
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
            max_pending_sweeps=cfg.max_pending_sweeps,
            deterministic_replay=cfg.deterministic_replay,
        )

    @property
    def lidar_names(self) -> tuple[str, ...]:
        """Configured lidar names, in `lidars` order -- push_lidar's `lidar=`."""
        return tuple(lidar.name for lidar in self._cfg.lidars)

    def push_imu(self, timestamp: float, acceleration, angular_velocity) -> int:
        """Push one IMU sample. Timestamps must be strictly increasing.

        Raises ValueError in LiDAR-only mode (``if_lidar_only=True``):
        upstream never drains its IMU buffer there, so samples pushed in that
        mode would accumulate without ever reaching the estimator.
        """
        return self._native.push_imu(
            float(timestamp), list(map(float, acceleration)), list(map(float, angular_velocity))
        )

    def push_imu_batch(self, samples) -> int:
        """Atomically commit a timestamp-ordered finite interval of IMU input."""
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
        """`points` is (N, 3) or (N, 4) [x, y, z, (intensity)]; `relative_times`
        is (N,) seconds from the source packet header timestamp. Injection
        begins at RESPLE's internal cloud buffer after applying the configured
        upstream callback's profile-specific predicates.

        ``lines`` and ``tags`` carry the raw Livox packet fields. They may be
        omitted for canonical inputs whose invalid driver points have already
        been removed; direct bag replay should provide them.

        ``lidar`` names the configured profile this sweep came from (see
        :attr:`lidar_names`); it may be omitted only when the session has a
        single LiDAR. With several, each one's sweeps must actually be pushed:
        upstream forms a measurement batch only once *every* configured LiDAR
        has buffered points, so a starved stream stalls the estimator rather
        than degrading to the others.

        Returns a globally monotonic ticket shared by every configured LiDAR.
        Missing IMU or LiDAR lookahead never rejects an ordinary push; use
        :meth:`synchronize` when a deterministic processed boundary is needed.
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
        """Drain currently supplied work to a deterministic boundary.

        With a LiDAR ticket, returns true only when the complete global ticket
        prefix through it has been processed. False means upstream is stably
        blocked pending more IMU, LiDAR lookahead, or another configured LiDAR.
        Without a ticket, synchronizes the current sensor-submission snapshot;
        this is useful for committing the initialization IMU prefix before the
        first sweep. The native wait releases the GIL.
        """
        return bool(self._native.synchronize(ticket))

    def trajectory(self) -> np.ndarray:
        """(N, 8): timestamp, x, y, z, qx, qy, qz, qw."""
        return self._native.trajectory()

    def latest_pose(self) -> np.ndarray | None:
        """Return the most recent published pose snapshot without blocking."""
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
        """Close input, drain the worker, and return the finalized results.

        There is no timeout: completion is a state condition. Metrics report
        any accepted trailing tickets that upstream could not process.
        """
        return self._native.finish()

    def __enter__(self) -> "RespleOdometry":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if exc_type is None:
            self.finish()

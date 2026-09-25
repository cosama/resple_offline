"""Thin Python wrapper around the compiled RESPLE offline bridge (_core)."""

from __future__ import annotations

import numpy as np

import collections

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
        self._max_pending_sweeps = int(max_pending_sweeps)
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
            max_pending_sweeps=self._max_pending_sweeps,
        )
        self._staged_imu: list[tuple[float, list[float], list[float]]] = []
        self._imu_pushed_count: int = 0
        self._unresolved_tickets: collections.deque[int] = collections.deque()
        self._previous_submitted_ticket: int | None = None
        self._latest_submitted_ticket: int = 0

    @property
    def lidar_names(self) -> tuple[str, ...]:
        """Return configured LiDAR names."""
        return tuple(lidar.name for lidar in self._cfg.lidars)

    def _flush_staged_imu(self) -> int:
        """Atomically commit staged IMU samples to the native worker."""
        if not self._staged_imu or self._cfg.if_lidar_only:
            return self._imu_pushed_count
        batch = self._staged_imu
        self._staged_imu = []
        self._native.push_imu_batch(
            [r[0] for r in batch],
            [r[1] for r in batch],
            [r[2] for r in batch],
        )
        return self._imu_pushed_count

    def push_imu(self, timestamp: float, acceleration, angular_velocity) -> int:
        """Push one ordered IMU sample (staged internally for atomic commit)."""
        if self._cfg.if_lidar_only:
            return 0
        self._staged_imu.append((
            float(timestamp),
            [float(v) for v in acceleration],
            [float(v) for v in angular_velocity],
        ))
        self._imu_pushed_count += 1
        return self._imu_pushed_count

    def push_imu_batch(self, samples) -> int:
        """Commit one ordered IMU interval."""
        if self._cfg.if_lidar_only:
            return 0
        for row in samples:
            self._staged_imu.append((
                float(row[0]),
                [float(v) for v in row[1]],
                [float(v) for v in row[2]],
            ))
            self._imu_pushed_count += 1
        return self._flush_staged_imu()

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
        self._flush_staged_imu()
        points = np.ascontiguousarray(points, dtype=np.float32)
        relative_times = np.ascontiguousarray(relative_times, dtype=np.float64)
        if lines is not None:
            lines = np.ascontiguousarray(lines, dtype=np.int32)
        if tags is not None:
            tags = np.ascontiguousarray(tags, dtype=np.int32)
        ticket = int(self._native.push_lidar(
            float(timestamp), points, relative_times, lines=lines, tags=tags,
            lidar="" if lidar is None else str(lidar),
        ))
        self._unresolved_tickets.append(ticket)
        self._previous_submitted_ticket = (
            self._latest_submitted_ticket if self._latest_submitted_ticket > 0 else None
        )
        self._latest_submitted_ticket = ticket

        while len(self._unresolved_tickets) > self._max_pending_sweeps:
            oldest = self._unresolved_tickets.popleft()
            self._native.synchronize(oldest)

        return ticket

    def synchronize(self, ticket: int | None = None) -> bool:
        """Drain input to a stable boundary.

        False indicates missing sensor lookahead. Without a ticket, the current
        submission snapshot defines the boundary.
        """
        self._flush_staged_imu()

        status = self._native.status()
        completed_ticket = status.get("completed_ticket", 0)

        while self._unresolved_tickets and self._unresolved_tickets[0] <= completed_ticket:
            self._unresolved_tickets.popleft()

        if ticket is None:
            if len(self._unresolved_tickets) > 1:
                target = self._unresolved_tickets[-2]
                self._native.synchronize(target)
            return bool(self._native.synchronize())

        target_ticket = int(ticket)
        if target_ticket <= completed_ticket:
            return True

        if target_ticket == self._latest_submitted_ticket:
            if self._previous_submitted_ticket is not None:
                if self._previous_submitted_ticket > completed_ticket:
                    success = bool(self._native.synchronize(self._previous_submitted_ticket))
                    if success:
                        while self._unresolved_tickets and self._unresolved_tickets[0] <= self._previous_submitted_ticket:
                            self._unresolved_tickets.popleft()
                    return success
                return True
            return bool(self._native.synchronize(target_ticket))

        success = bool(self._native.synchronize(target_ticket))
        if success:
            while self._unresolved_tickets and self._unresolved_tickets[0] <= target_ticket:
                self._unresolved_tickets.popleft()
        return success

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
        self._flush_staged_imu()
        while self._unresolved_tickets:
            ticket = self._unresolved_tickets.popleft()
            self._native.synchronize(ticket)
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


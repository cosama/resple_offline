"""Deterministic synthetic sensor stream for lifecycle/determinism tests.

A stationary sensor inside a closed box room gives well-conditioned plane
geometry in all three axes, so the estimator has a solvable problem without
real data. (An earlier version of this test used scattered random-plane
points with no walls in x/y -- that under-constrained geometry converged to
zero committed poses even though the estimator ran without error. Box walls
are what actually exercises the IEKF update path.)
"""

from __future__ import annotations

import numpy as np

GRAVITY = 9.81


def box_room_sweep(position: np.ndarray, n_azimuth: int = 360, n_rings: int = 16,
                   half_extent: float = 8.0, half_height: float = 2.5,
                   seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Ray-cast a closed box from `position`. Returns (points[N,3] f32, relative_times[N] f64)."""
    rng = np.random.default_rng(seed)
    azimuth = np.linspace(0.0, 2.0 * np.pi, n_azimuth, endpoint=False)
    elevation = np.linspace(np.deg2rad(-15.0), np.deg2rad(15.0), n_rings)
    az, el = np.meshgrid(azimuth, elevation, indexing="ij")
    direction = np.stack(
        (np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)), axis=-1
    ).reshape(-1, 3)
    limits = np.array([half_extent, half_extent, half_height])
    with np.errstate(divide="ignore", invalid="ignore"):
        t_plane = (np.sign(direction) * limits - position) / direction
    t_plane[~np.isfinite(t_plane)] = np.inf
    t_plane[t_plane <= 0] = np.inf
    t = np.min(t_plane, axis=1)
    points = (position + direction * t[:, None]).astype(np.float32)
    relative_times = np.sort(rng.uniform(0.0, 0.1, points.shape[0])).astype(np.float64)
    return points, relative_times


def run_stationary_session(odom, duration: float = 3.0, imu_hz: float = 200.0,
                           lidar_hz: float = 10.0) -> dict:
    """Feed a stationary box-room scene through `odom` (a resple.RespleOdometry) and finish()."""
    next_lidar_t = 0.0
    sweep_index = 0
    n_imu = int(duration * imu_hz) + 50
    for i in range(n_imu):
        stamp = i / imu_hz
        odom.push_imu(stamp, [0.0, 0.0, GRAVITY], [0.0, 0.0, 0.0])
        while next_lidar_t <= stamp - 0.05 and next_lidar_t < duration:
            points, relative_times = box_room_sweep(np.zeros(3), seed=sweep_index)
            odom.push_lidar(next_lidar_t, points, relative_times)
            sweep_index += 1
            next_lidar_t += 1.0 / lidar_hz
    return odom.finish(30.0)

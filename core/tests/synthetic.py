"""Deterministic, well-constrained box-room stream for lifecycle tests."""

from __future__ import annotations

from collections import deque

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


# Supply the gravity prefix.
IMU_LEAD_SECONDS = 0.1


def run_stationary_session(odom, duration: float = 1.5, imu_hz: float = 200.0,
                           lidar_hz: float = 10.0, with_imu: bool = True) -> dict:
    """Feed a stationary box-room scene through `odom` (a resple.RespleOdometry) and finish().

    `with_imu=False` drives a LiDAR-only (if_lidar_only=true) session, which
    refuses push_imu outright: upstream never drains the IMU buffer in that
    mode. The IMU clock still paces sweep release so both modes replay the
    same sweeps at the same stamps.
    """
    assert imu_hz * IMU_LEAD_SECONDS >= 15, "first sweep would race gravity initialization"
    next_lidar_t = 0.0
    sweep_index = 0
    unresolved = deque()
    n_imu = int(duration * imu_hz) + 50
    for i in range(n_imu):
        stamp = i / imu_hz
        if with_imu:
            odom.push_imu(stamp, [0.0, 0.0, GRAVITY], [0.0, 0.0, 0.0])
            if i == 14:
                assert odom.synchronize(), "initialization IMU prefix did not synchronize"
        while next_lidar_t <= stamp - IMU_LEAD_SECONDS and next_lidar_t < duration:
            points, relative_times = box_room_sweep(
                np.zeros(3), n_azimuth=120, n_rings=8, seed=sweep_index,
            )
            unresolved.append(odom.push_lidar(next_lidar_t, points, relative_times))
            sweep_index += 1
            next_lidar_t += 1.0 / lidar_hz
            while unresolved and odom.synchronize(unresolved[0]):
                unresolved.popleft()
    return odom.finish()


# Offset the second LiDAR.
SECOND_LIDAR_OFFSET = np.array([0.3, -0.2, 0.1])


def second_lidar_extrinsics() -> tuple[tuple[float, float, float, float], list[float]]:
    """(q_lb, t_lb) placing a LiDAR at SECOND_LIDAR_OFFSET in the body frame."""
    return (1.0, 0.0, 0.0, 0.0), list(-SECOND_LIDAR_OFFSET)


def run_stationary_multi_lidar_session(odom, duration: float = 1.5, imu_hz: float = 200.0,
                                       lidar_hz: float = 10.0) -> dict:
    """Feed the same stationary box room through two co-mounted LiDARs.

    The two streams are staggered by half a sweep period rather than pushed in
    lockstep, so the run actually exercises interleaved arrival. Both must be
    fed: upstream's collectMeasurements() forms a batch only once *every*
    configured LiDAR has buffered points.

    `odom.lidar_names` supplies the names; the second one is placed at
    SECOND_LIDAR_OFFSET, so its points only agree with the first one's walls if
    its extrinsics are actually applied.
    """
    assert imu_hz * IMU_LEAD_SECONDS >= 15, "first sweep would race gravity initialization"
    first, second = odom.lidar_names
    # Track each sensor schedule.
    streams = [[first, np.zeros(3), 0.0], [second, SECOND_LIDAR_OFFSET, 0.5 / lidar_hz]]
    sweep_index = 0
    unresolved = deque()
    for i in range(int(duration * imu_hz) + 50):
        stamp = i / imu_hz
        odom.push_imu(stamp, [0.0, 0.0, GRAVITY], [0.0, 0.0, 0.0])
        if i == 14:
            assert odom.synchronize(), "initialization IMU prefix did not synchronize"
        while True:
            stream = min(streams, key=lambda entry: entry[2])
            name, position, next_t = stream
            if next_t > stamp - IMU_LEAD_SECONDS or next_t >= duration:
                break
            world_points, relative_times = box_room_sweep(
                position, n_azimuth=120, n_rings=8, seed=sweep_index,
            )
            unresolved.append(odom.push_lidar(
                next_t, world_points - position.astype(np.float32),
                relative_times, lidar=name,
            ))
            sweep_index += 1
            stream[2] = next_t + 1.0 / lidar_hz
            while unresolved and odom.synchronize(unresolved[0]):
                unresolved.popleft()
    return odom.finish()

"""Typed configuration for the RESPLE offline bridge.

Mirrors dali_slam's config.py shape: one dataclass per requested config, a
`resolve()` that fills in profile defaults, `validate()` that collects every
problem instead of failing on the first, and `native_parameters()` that
returns exactly what bindings.cpp's RespleOdometry constructor consumes --
so the manifest can report requested vs. effective values (ARCHITECTURE.md
#7) instead of letting native fall back to a value nobody chose.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

# One lidar entry today (design decision: single LiDAR for phase 1), kept as
# a list because upstream's own "lidars" parameter is a list and
# Estimator/Association already support fusing several -- adding a second
# entry later is a config/runner change, not a bridge rewrite.
_KNOWN_LIDAR_TYPES = {"Ouster", "Mid70Avia", "HAP360", "AviaResple", "Hesai", "Mid360Boxi"}


@dataclass(frozen=True)
class LidarProfile:
    name: str
    lidar_type: str
    scan_line: int
    blind: float
    q_lb: tuple[float, float, float, float]  # (w, x, y, z)
    t_lb: tuple[float, float, float]
    w_pt: float = 0.01


@dataclass(frozen=True)
class RespleConfig:
    """One resolved RESPLE run configuration (LIO mode; Estimator<30>)."""

    lidars: tuple[LidarProfile, ...]

    knot_hz: int = 100
    ds_scan_voxel: float = 0.5
    ds_lm_voxel: float = 0.5
    point_filter_num: int = 1
    nn_thresh: float = 0.5
    coeff_cov: float = 5.0
    num_nn: int = 8
    cube_len: float = 1000.0

    cov_P0: float = 0.02
    cov_RCP_pos_old: float = 0.5
    cov_RCP_ort_old: float = 0.5
    cov_RCP_pos_new: float = 1.0
    cov_RCP_ort_new: float = 1.0
    std_sys_pos: float = 0.1
    std_sys_ort: float = 0.1
    cov_acc: tuple[float, float, float] = (1.0, 1.0, 1.0)
    cov_gyro: tuple[float, float, float] = (0.1, 0.1, 0.1)
    cov_ba: tuple[float, float, float] = (0.2, 0.2, 0.2)
    cov_bg: tuple[float, float, float] = (0.2, 0.2, 0.2)
    n_iter: int = 3
    num_points_upd: int = 100
    acc_ratio: bool = False  # True if IMU linear_acceleration is in g, not m/s^2
    lidar_time_offset: float = 0.0

    max_pending_sweeps: int = 8
    min_imu_lead_seconds: float = 0.0

    def validate(self) -> list[str]:
        problems: list[str] = []
        if not self.lidars:
            problems.append("at least one lidar profile is required")
        elif len(self.lidars) > 1:
            problems.append(
                "multi-lidar fusion is not implemented yet (single LiDAR only); "
                f"got {len(self.lidars)} entries"
            )
        for lidar in self.lidars:
            if lidar.lidar_type not in _KNOWN_LIDAR_TYPES:
                problems.append(
                    f"lidar '{lidar.name}': unknown lidar_type {lidar.lidar_type!r}, "
                    f"expected one of {sorted(_KNOWN_LIDAR_TYPES)}"
                )
            if len(lidar.q_lb) != 4:
                problems.append(f"lidar '{lidar.name}': q_lb must have length 4 (w, x, y, z)")
            if len(lidar.t_lb) != 3:
                problems.append(f"lidar '{lidar.name}': t_lb must have length 3")
            if lidar.blind < 0:
                problems.append(f"lidar '{lidar.name}': blind must be >= 0")
        if self.knot_hz <= 0:
            problems.append("knot_hz must be positive")
        if self.num_points_upd <= 0:
            problems.append("num_points_upd must be positive")
        if self.point_filter_num <= 0:
            problems.append("point_filter_num must be positive")
        if len(self.cov_acc) != 3 or len(self.cov_gyro) != 3:
            problems.append("cov_acc and cov_gyro must have length 3")
        if len(self.cov_ba) != 3 or len(self.cov_bg) != 3:
            problems.append("cov_ba and cov_bg must have length 3")
        return problems

    def native_parameters(self) -> dict[str, Any]:
        """Exactly the keys/types RespleOdometry's `parameters` dict consumes.

        Per-lidar keys (topic_lidar, lidar_type, scan_line, blind, q_lb, t_lb,
        w_pt) and `lidars` itself are supplied separately by RespleOdometry's
        constructor arguments, not through this dict -- see __init__.py.
        """
        return {
            "knot_hz": self.knot_hz,
            "ds_scan_voxel": self.ds_scan_voxel,
            "ds_lm_voxel": self.ds_lm_voxel,
            "point_filter_num": self.point_filter_num,
            "nn_thresh": self.nn_thresh,
            "coeff_cov": self.coeff_cov,
            "num_nn": self.num_nn,
            "cube_len": self.cube_len,
            "cov_P0": self.cov_P0,
            "cov_RCP_pos_old": self.cov_RCP_pos_old,
            "cov_RCP_ort_old": self.cov_RCP_ort_old,
            "cov_RCP_pos_new": self.cov_RCP_pos_new,
            "cov_RCP_ort_new": self.cov_RCP_ort_new,
            "std_sys_pos": self.std_sys_pos,
            "std_sys_ort": self.std_sys_ort,
            "cov_acc": list(self.cov_acc),
            "cov_gyro": list(self.cov_gyro),
            "cov_ba": list(self.cov_ba),
            "cov_bg": list(self.cov_bg),
            "n_iter": self.n_iter,
            "num_points_upd": self.num_points_upd,
            "acc_ratio": self.acc_ratio,
            "lidar_time_offset": self.lidar_time_offset,
        }

    def report(self) -> dict[str, Any]:
        """Requested config for the run manifest (ARCHITECTURE.md #7)."""
        data = {f.name: getattr(self, f.name) for f in fields(self)}
        data["lidars"] = [
            {
                "name": lidar.name,
                "lidar_type": lidar.lidar_type,
                "scan_line": lidar.scan_line,
                "blind": lidar.blind,
                "q_lb": list(lidar.q_lb),
                "t_lb": list(lidar.t_lb),
                "w_pt": lidar.w_pt,
            }
            for lidar in self.lidars
        ]
        return data


def resolve(overrides: dict[str, Any] | None = None) -> RespleConfig:
    """Build a RespleConfig from defaults plus a flat dict of overrides.

    `overrides["lidars"]`, if present, must already be a list of dicts with
    LidarProfile's fields; everything else overrides a scalar RespleConfig
    field by name.
    """
    overrides = dict(overrides or {})
    lidars_raw = overrides.pop("lidars", None)
    if not lidars_raw:
        raise ValueError("resolve() requires at least one entry in 'lidars'")
    lidars = tuple(
        LidarProfile(
            name=entry["name"],
            lidar_type=entry["lidar_type"],
            scan_line=int(entry.get("scan_line", 1)),
            blind=float(entry.get("blind", 0.5)),
            q_lb=tuple(entry.get("q_lb", (1.0, 0.0, 0.0, 0.0))),
            t_lb=tuple(entry.get("t_lb", (0.0, 0.0, 0.0))),
            w_pt=float(entry.get("w_pt", 0.01)),
        )
        for entry in lidars_raw
    )
    known = {f.name for f in fields(RespleConfig)} - {"lidars"}
    unknown = set(overrides) - known
    if unknown:
        raise ValueError(f"unknown RespleConfig field(s): {sorted(unknown)}")
    return RespleConfig(lidars=lidars, **overrides)

"""Typed configuration for the RESPLE offline bridge."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

# Types identify LiDAR buffers.
_KNOWN_LIDAR_TYPES = {
    "AviaResple", "HAP360", "Hesai", "Mid360Boxi", "Mid70Avia", "Ouster"
}


@dataclass(frozen=True)
class LidarProfile:
    name: str
    lidar_type: str
    scan_line: int
    blind: float
    q_lb: tuple[float, float, float, float]  # Quaternion uses WXYZ order.
    t_lb: tuple[float, float, float]
    topic_lidar: str = ""
    w_pt: float = 0.01

    def upstream_parameters(self) -> dict[str, Any]:
        return {
            "topic_lidar": self.topic_lidar,
            "lidar_type": self.lidar_type,
            "scan_line": self.scan_line,
            "blind": self.blind,
            "q_lb": list(self.q_lb),
            "t_lb": list(self.t_lb),
            "w_pt": self.w_pt,
        }


@dataclass(frozen=True)
class RespleConfig:
    """One resolved RESPLE run configuration."""

    lidars: tuple[LidarProfile, ...]

    topic_imu: str = ""
    if_lidar_only: bool = False
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
    acc_ratio: bool = False  # Acceleration may use gravity units.
    lidar_time_offset: float = 0.0

    def validate(self) -> list[str]:
        problems: list[str] = []
        if not self.lidars:
            problems.append("at least one lidar profile is required")
        seen_names: set[str] = set()
        seen_types: dict[str, str] = {}
        for lidar in self.lidars:
            if lidar.name in seen_names:
                problems.append(f"duplicate lidar name {lidar.name!r}")
            seen_names.add(lidar.name)
            if lidar.lidar_type in seen_types:
                problems.append(
                    f"lidars {seen_types[lidar.lidar_type]!r} and {lidar.name!r} both use "
                    f"lidar_type {lidar.lidar_type!r}; upstream RESPLE keys its per-lidar "
                    "configuration and buffers by type, so at most one profile per type"
                )
            else:
                seen_types[lidar.lidar_type] = lidar.name
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
            if lidar.scan_line <= 0:
                problems.append(f"lidar '{lidar.name}': scan_line must be positive")
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
        """Return typed native parameters."""
        return {
            "topic_imu": self.topic_imu,
            "if_lidar_only": self.if_lidar_only,
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
        """Return the resolved configuration."""
        data = {f.name: getattr(self, f.name) for f in fields(self)}
        data["lidars"] = [
            {"name": lidar.name, **lidar.upstream_parameters()}
            for lidar in self.lidars
        ]
        return data

    def upstream_parameters(self) -> dict[str, Any]:
        """Return the ROS2 parameter mapping accepted by upstream RESPLE."""
        params = self.native_parameters()
        params["lidars"] = [lidar.name for lidar in self.lidars]
        for lidar in self.lidars:
            params[lidar.name] = lidar.upstream_parameters()
        return params

    def upstream_yaml_mapping(self) -> dict[str, Any]:
        return {"/**": {"ros__parameters": self.upstream_parameters()}}


def normalize_overrides(payload: dict[str, Any] | None) -> dict[str, Any]:
    """Normalize legacy-flat or upstream ROS2 configuration to one mapping.

    Upstream represents ``lidars`` as a list of profile names whose mappings
    are siblings under ``ros__parameters``.  The bridge's typed model stores
    the same information as a list of profile mappings.
    """
    if payload is None:
        return {}
    if not isinstance(payload, dict):
        raise ValueError("RESPLE config must be a mapping")
    envelopes = [
        key for key, value in payload.items()
        if isinstance(value, dict) and "ros__parameters" in value
    ]
    if envelopes:
        if "/**" in envelopes:
            envelope = payload["/**"]
        elif len(envelopes) == 1:
            envelope = payload[envelopes[0]]
        else:
            raise ValueError("RESPLE config has multiple matching ROS2 parameter envelopes")
        params = envelope["ros__parameters"]
        if not isinstance(params, dict):
            raise ValueError("ros__parameters must be a mapping")
        result = dict(params)
    else:
        result = dict(payload)

    lidars_raw = result.get("lidars")
    if isinstance(lidars_raw, (list, tuple)) and lidars_raw and all(
        isinstance(name, str) for name in lidars_raw
    ):
        profiles = []
        seen: set[str] = set()
        for name in lidars_raw:
            if name in seen:
                raise ValueError(f"duplicate lidar profile name: {name!r}")
            seen.add(name)
            profile = result.pop(name, None)
            if not isinstance(profile, dict):
                raise ValueError(f"lidar profile {name!r} must be a mapping")
            required = {"topic_lidar", "lidar_type", "scan_line", "q_lb", "t_lb", "w_pt"}
            missing = sorted(required - set(profile))
            if missing:
                raise ValueError(
                    f"upstream lidar profile {name!r} is missing required field(s): {missing}"
                )
            profiles.append({"name": name, **profile})
        result["lidars"] = profiles
        ignored = [key for key, value in result.items() if isinstance(value, dict)]
        for key in ignored:
            result.pop(key)
    return result


def resolve(overrides: dict[str, Any] | None = None) -> RespleConfig:
    """Build a RespleConfig from defaults plus a flat dict of overrides.

    `overrides["lidars"]`, if present, must already be a list of dicts with
    LidarProfile's fields; everything else overrides a scalar RespleConfig
    field by name.
    """
    overrides = normalize_overrides(overrides)
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
            # Supply stable offline topics.
            topic_lidar=str(entry.get("topic_lidar") or f"/offline/{entry['name']}"),
            w_pt=float(entry.get("w_pt", 0.01)),
        )
        for entry in lidars_raw
    )
    if not overrides.get("topic_imu"):
        overrides["topic_imu"] = "/offline/imu"
    # Preserve immutable vector fields.
    for key in ("cov_acc", "cov_gyro", "cov_ba", "cov_bg"):
        value = overrides.get(key)
        if isinstance(value, (list, tuple)):
            overrides[key] = tuple(float(component) for component in value)
    known = {f.name for f in fields(RespleConfig)} - {"lidars"}
    unknown = set(overrides) - known
    if unknown:
        raise ValueError(f"unknown RespleConfig field(s): {sorted(unknown)}")
    return RespleConfig(lidars=lidars, **overrides)

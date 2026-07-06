"""YAML → dataclass config loader.

One source of truth for robot + force-estimator + sandbox configuration.
Used by every entry-point script so that command-line invocations all
read from the same YAML files committed under config/.
"""
from __future__ import annotations

import yaml
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class RobotConfig:
    model: str
    variant: str
    payload_kg: float
    reach_m: float
    repeatability_m: float
    dof: int


@dataclass
class DriverConfig:
    backend: str
    pinned_version: str
    port: str = "/dev/ttyACM0"
    baudrate: int = 1_000_000


@dataclass
class ForceEstimatorConfig:
    type: str                       # "kinematic_only" | "joint_current" | "wrench_sensor"
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class URDFConfig:
    path: str
    base_link: str
    ee_link: str


@dataclass
class RobotYAML:
    robot: RobotConfig
    driver: DriverConfig
    force_estimator: ForceEstimatorConfig
    publish_rate_hz: int
    urdf: URDFConfig


@dataclass
class SandboxYAML:
    name: str
    E_pa: float
    nu: float
    phi_deg: float
    rho_bulk: float
    d50_m: float
    cohesion_pa: float
    alpha_calibrated: float
    recommended_dx_m: float
    recommended_dt_s: float


# ─── Loaders ──────────────────────────────────────────────────────────────

def load_robot_yaml(path: str | Path) -> RobotYAML:
    raw = _read_yaml(path)
    robot_cfg = RobotConfig(**raw["robot"])
    driver_cfg = DriverConfig(**raw["driver"])
    fe_raw = raw["force_estimator"]
    fe_cfg = ForceEstimatorConfig(
        type=fe_raw.pop("type"),
        extra=fe_raw,
    )
    urdf_cfg = URDFConfig(**raw["urdf"])
    return RobotYAML(
        robot=robot_cfg,
        driver=driver_cfg,
        force_estimator=fe_cfg,
        publish_rate_hz=raw["publish_rate_hz"],
        urdf=urdf_cfg,
    )


def load_sandbox_yaml(path: str | Path) -> SandboxYAML:
    raw = _read_yaml(path)
    sand = raw["sand"]
    sim_map = raw["sim_mapping"]
    cfl = raw["cfl"]
    return SandboxYAML(
        name=sand["name"],
        E_pa=float(sand["E_pa"]),
        nu=float(sand["nu"]),
        phi_deg=float(sand["phi_deg"]),
        rho_bulk=float(sand["rho_bulk"]),
        d50_m=float(sand["d50_m"]),
        cohesion_pa=float(sand["cohesion_pa"]),
        alpha_calibrated=float(sim_map["alpha_calibrated"]),
        recommended_dx_m=float(cfl["recommended_dx_m"]),
        recommended_dt_s=float(cfl["recommended_dt_s"]),
    )


def _read_yaml(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"config not found: {p}")
    with p.open(encoding="utf-8") as f:
        return yaml.safe_load(f)

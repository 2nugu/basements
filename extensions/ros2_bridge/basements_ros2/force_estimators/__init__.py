"""Force estimator strategy registry.

YAML config picks the implementation at runtime. Anti-foreclosure: no
compile-time type-binding so the same code base supports
(280/320/Pro 600) × (kinematic/joint_current/F-T) at experiment time.
"""
from __future__ import annotations

from .base import ForceEstimator, ForceMeasurement
from .kinematic_only import KinematicOnlyEstimator
from .joint_current import JointCurrentEstimator
from .wrench_sensor import WrenchSensorEstimator


REGISTRY: dict[str, type[ForceEstimator]] = {
    "kinematic_only": KinematicOnlyEstimator,
    "joint_current":  JointCurrentEstimator,
    "wrench_sensor":  WrenchSensorEstimator,
}


def build(estimator_type: str, extra_config: dict) -> ForceEstimator:
    if estimator_type not in REGISTRY:
        raise KeyError(
            f"unknown force_estimator type '{estimator_type}'. "
            f"available: {list(REGISTRY.keys())}"
        )
    return REGISTRY[estimator_type](extra_config)


__all__ = [
    "ForceEstimator",
    "ForceMeasurement",
    "KinematicOnlyEstimator",
    "JointCurrentEstimator",
    "WrenchSensorEstimator",
    "REGISTRY",
    "build",
]

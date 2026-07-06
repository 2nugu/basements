"""KinematicOnlyEstimator — no force sensing, depth tracking only.

For Experiment A (sphere drop), the ball's penetration depth is measured
from external video; no end-effector force estimation is needed. This
estimator returns None on every read() so downstream nodes know to
skip force diff reporting.
"""
from __future__ import annotations

from .base import ForceEstimator, ForceMeasurement


class KinematicOnlyEstimator(ForceEstimator):
    def read(self) -> ForceMeasurement | None:
        return None

    def relative_accuracy(self) -> float:
        # Kinematic-only is used with depth measurement (not force),
        # so this value is the depth-measurement uncertainty floor
        # (240 fps video) per literature_uncertainty.md §1.
        return 0.15

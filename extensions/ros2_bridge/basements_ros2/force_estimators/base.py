"""ForceEstimator ABC — strategy interface."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class ForceMeasurement:
    """End-effector wrench in world frame, with calibrated uncertainty."""
    force_world:       np.ndarray         # shape (3,), N
    torque_world:      np.ndarray         # shape (3,), N·m
    std_dev_estimate:  float              # relative uncertainty (e.g. 0.15)
    timestamp_sec:     float


class ForceEstimator(ABC):
    """Interface every force-sensing backend must implement.

    Implementations are picked by YAML config at runtime, not compile
    time. Virtual call overhead is < 1 μs at 100 Hz — negligible vs
    MPMSolver step time (see docs/notes/l3_pipeline_design.md §O1).
    """

    def __init__(self, extra_config: dict):
        self._cfg = extra_config

    @abstractmethod
    def read(self) -> ForceMeasurement | None:
        """Return latest force measurement, or None if not available.

        None is reserved for `kinematic_only` where no force data exists.
        Callers must handle None — replay/replayer downstream skips force
        diff when None is returned.
        """

    @abstractmethod
    def relative_accuracy(self) -> float:
        """Expected ± fractional accuracy (e.g. 0.15 = ±15%).

        Used by external_validator.py to pick the pass criterion floor.
        """

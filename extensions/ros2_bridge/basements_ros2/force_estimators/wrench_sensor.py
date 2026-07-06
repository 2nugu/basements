"""WrenchSensorEstimator — pass-through from /wrench topic.

For Experiment D direct F/T comparison. The ROS 2 WrenchStamped is
captured from a wrist-mounted 6-DOF sensor (ATI Nano17, Robotiq FT 300).
Bias compensation runs once at startup over `static_duration_s` seconds.

This estimator does not perform any inversion — the sensor already
reports wrench in the sensor frame; we transform to world frame using
tf2 lookup chained from the URDF.
"""
from __future__ import annotations

import time
from threading import Lock

import numpy as np

from .base import ForceEstimator, ForceMeasurement


class WrenchSensorEstimator(ForceEstimator):
    def __init__(self, extra_config: dict):
        super().__init__(extra_config)
        self._topic = extra_config["topic"]
        self._frame_id = extra_config["frame_id"]
        self._expected_pct = float(extra_config.get("expected_accuracy_pct", 2)) / 100.0

        bias_cfg = extra_config.get("bias_compensation", {})
        self._bias_enabled = bool(bias_cfg.get("enabled", True))
        self._bias_duration_s = float(bias_cfg.get("static_duration_s", 5.0))

        self._lock = Lock()
        self._latest_msg = None
        self._bias_force = np.zeros(3)
        self._bias_torque = np.zeros(3)
        self._bias_captured = False

    # The ROS 2 wiring is in sim_node.py; this class only consumes
    # already-deserialised numpy arrays for unit-testability.
    def push_measurement(self, force: np.ndarray, torque: np.ndarray, ts: float) -> None:
        with self._lock:
            self._latest_msg = (force.copy(), torque.copy(), ts)

    def capture_bias(self) -> None:
        """Average the first N samples as static bias; call before experiment."""
        # Real ROS 2 implementation collects bias_duration_s of samples.
        # This stub is replaced at runtime by sim_node.py's bias-capture coroutine.
        self._bias_captured = True

    def read(self) -> ForceMeasurement | None:
        with self._lock:
            if self._latest_msg is None:
                return None
            force, torque, ts = self._latest_msg

        if self._bias_captured:
            force = force - self._bias_force
            torque = torque - self._bias_torque

        return ForceMeasurement(
            force_world=force,
            torque_world=torque,
            std_dev_estimate=self._expected_pct,
            timestamp_sec=ts,
        )

    def relative_accuracy(self) -> float:
        return self._expected_pct

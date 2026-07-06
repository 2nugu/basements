"""JointCurrentEstimator — pymycobot motor current → end-effector wrench.

Math:
    τ_joint_i = K_torque_i · I_motor_i      (motor torque constant model)
    τ_gravity = J^T · (m_link · g)          (subtracted via URDF lookup)
    τ_ext    = τ_joint - τ_gravity
    F_ext    = (J·J^T)^(-1) · J · τ_ext     (Jacobian inversion to ee wrench)

Expected accuracy: ±15% (myCobot manual + Brzinski-Durian 2010 envelope).

Note: this is a *runtime estimator*. For the offline replay flow it is
unused — joint torque is logged via trajectory_logger and the Jacobian
inversion is computed during replay so that sim and real both see the
same numerical recipe. See trajectory_replayer.py for offline reduction.
"""
from __future__ import annotations

import time

import numpy as np

from .base import ForceEstimator, ForceMeasurement


class JointCurrentEstimator(ForceEstimator):
    def __init__(self, extra_config: dict):
        super().__init__(extra_config)
        self._torque_constants = np.array(
            extra_config["torque_constants_nm_per_a"], dtype=float
        )
        self._gravity_comp = bool(extra_config.get("gravity_compensation", True))
        self._expected_pct = float(extra_config.get("expected_accuracy_pct", 15)) / 100.0

        # The driver/jacobian dependency is injected so that this class
        # stays unit-testable without pymycobot installed. See
        # driver_wrapper.py for the live wiring.
        self._driver = None
        self._jacobian_fn = None

    def attach(self, driver, jacobian_fn) -> None:
        """Late-binding driver + Jacobian provider, called after MyCobotDriver init."""
        self._driver = driver
        self._jacobian_fn = jacobian_fn

    def read(self) -> ForceMeasurement | None:
        if self._driver is None or self._jacobian_fn is None:
            return None

        currents_a = np.asarray(self._driver.read_servo_currents(), dtype=float)
        joint_torques = self._torque_constants * currents_a   # N·m per joint

        if self._gravity_comp:
            joint_torques = joint_torques - self._driver.gravity_torque()

        jacobian = self._jacobian_fn(self._driver.read_joint_angles())
        # Pseudo-inverse: τ = Jᵀ F  ⇒  F = (J Jᵀ)^(-1) J τ
        # equivalent to F = (Jᵀ)^+ τ via SVD; numpy.linalg.pinv handles it.
        wrench = np.linalg.pinv(jacobian.T) @ joint_torques     # shape (6,)

        return ForceMeasurement(
            force_world=wrench[:3],
            torque_world=wrench[3:],
            std_dev_estimate=self._expected_pct,
            timestamp_sec=time.time(),
        )

    def relative_accuracy(self) -> float:
        return self._expected_pct

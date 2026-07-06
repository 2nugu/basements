"""MyCobotDriver — vendored pymycobot wrapper.

Three-tier fallback per docs/notes/l3_pipeline_design.md §Q2:
    Tier 1 — pymycobot (preferred, easiest)
    Tier 2 — direct serial protocol (if pymycobot unavailable)
    Tier 3 — external ROS 2 driver (last resort, not implemented here)

Provides the minimal interface the rest of basements_ros2 depends on,
isolating elephantrobotics SDK changes to this single module.
"""
from __future__ import annotations

import logging
from typing import Protocol

import numpy as np

log = logging.getLogger(__name__)


class _MyCobotProtocol(Protocol):
    def read_joint_angles(self) -> list[float]: ...
    def read_servo_currents(self) -> list[float]: ...
    def send_joint_angles(self, angles_deg: list[float], speed: int) -> None: ...
    def gravity_torque(self) -> np.ndarray: ...
    def power_on(self) -> None: ...
    def power_off(self) -> None: ...


class MyCobotDriver:
    """Tier-1 implementation: thin wrapper over pymycobot.

    All methods raise ConnectionError on serial-failure; the caller
    decides whether to retry, drop to Tier 2, or abort the experiment.
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000):
        try:
            from pymycobot import MyCobot
        except ImportError as e:
            raise ImportError(
                "pymycobot not installed. install via:\n"
                "    pip install pymycobot==3.6.1\n"
                "or fall back to direct serial (Tier 2)."
            ) from e

        self._mc = MyCobot(port, baudrate)
        self._port = port

    def power_on(self) -> None:
        self._mc.power_on()

    def power_off(self) -> None:
        self._mc.power_off()

    def read_joint_angles(self) -> list[float]:
        angles = self._mc.get_angles()
        if angles is None:
            raise ConnectionError(f"failed to read joint angles from {self._port}")
        return angles

    def read_servo_currents(self) -> list[float]:
        # pymycobot exposes get_servo_currents() since 3.5; safety check below.
        if not hasattr(self._mc, "get_servo_currents"):
            raise NotImplementedError(
                "this pymycobot version does not expose get_servo_currents; "
                "upgrade to >=3.5 or use kinematic_only force estimator."
            )
        currents = self._mc.get_servo_currents()
        if currents is None:
            raise ConnectionError(f"failed to read servo currents from {self._port}")
        return currents

    def send_joint_angles(self, angles_deg: list[float], speed: int = 20) -> None:
        self._mc.send_angles(angles_deg, speed)

    def gravity_torque(self) -> np.ndarray:
        """Static gravity torque per joint, computed offline from URDF.

        Stub: returns zeros. Real impl reads URDF, runs RNEA with zero
        joint velocity / acceleration and current joint angles. Wire in
        the URDF parser when the experiment requires gravity-compensated
        joint-current force estimation.
        """
        return np.zeros(6)


# ─── Driver verification entry-point ──────────────────────────────────────

def verify_main():
    """Console-script entry point for the gating ArUco verification.

    Stub implementation. The real verification flow lives in W4-prep
    task #120 and is wired up just before sphere-drop experiment.
    """
    print("[verify_driver] stub — wire ArUco capture per W4-prep task #120")
    print("[verify_driver] required PASS criteria:")
    print("    RMS position error  < 1.0 mm")
    print("    RMS orientation err < 1.0 deg")
    print("    repeatability       < 0.5 mm")


if __name__ == "__main__":
    verify_main()

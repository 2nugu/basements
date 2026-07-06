"""trajectory_logger — record myCobot pose + (optional) wrench to CSV.

Used during real experiment phase: arm executes a programmed trajectory
(or is teleoperated), and this logger writes per-tick rows to CSV that
trajectory_replayer.py can later replay through MPMSolver.

Output CSV schema (matches docs/notes/external_validation_protocol.md §5):
    t_sec, j0_deg, j1_deg, j2_deg, j3_deg, j4_deg, j5_deg,
    ee_x_m, ee_y_m, ee_z_m, ee_qw, ee_qx, ee_qy, ee_qz,
    cur0_a, cur1_a, cur2_a, cur3_a, cur4_a, cur5_a,
    f_x_n, f_y_n, f_z_n, t_x_nm, t_y_nm, t_z_nm
"""
from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from .config_loader import load_robot_yaml
from .driver_wrapper import MyCobotDriver
from .force_estimators import build as build_force_estimator


CSV_HEADER = [
    "t_sec",
    "j0_deg", "j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg",
    "ee_x_m", "ee_y_m", "ee_z_m",
    "ee_qw", "ee_qx", "ee_qy", "ee_qz",
    "cur0_a", "cur1_a", "cur2_a", "cur3_a", "cur4_a", "cur5_a",
    "f_x_n", "f_y_n", "f_z_n",
    "t_x_nm", "t_y_nm", "t_z_nm",
]


def main():
    p = argparse.ArgumentParser(description="Record myCobot trajectory to CSV.")
    p.add_argument("--config",   required=True, help="YAML robot config path")
    p.add_argument("--output",   required=True, help="output CSV path")
    p.add_argument("--duration", type=float, default=30.0, help="seconds")
    p.add_argument("--rate",     type=int,   default=50,   help="log rate Hz")
    args = p.parse_args()

    cfg = load_robot_yaml(args.config)
    driver = MyCobotDriver(cfg.driver.port, cfg.driver.baudrate)
    driver.power_on()

    fe = build_force_estimator(cfg.force_estimator.type, cfg.force_estimator.extra)
    # Joint-current estimator needs late-binding to driver; wrench-sensor
    # estimator is wired through ROS 2 (not this offline path).
    if hasattr(fe, "attach"):
        fe.attach(driver, _build_jacobian_fn(cfg.urdf.path))

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)

        t0 = time.time()
        period = 1.0 / args.rate
        while (now := time.time() - t0) < args.duration:
            try:
                angles = driver.read_joint_angles()
                currents = driver.read_servo_currents() if cfg.force_estimator.type != "kinematic_only" else [0.0] * 6
            except (ConnectionError, NotImplementedError) as e:
                print(f"[logger] {e}; aborting")
                break

            # End-effector pose via URDF FK — stub: identity for now.
            ee_pos = np.zeros(3)
            ee_quat = np.array([1.0, 0.0, 0.0, 0.0])  # (w, x, y, z)

            fm = fe.read() if cfg.force_estimator.type != "kinematic_only" else None
            f_vec = fm.force_world if fm else np.zeros(3)
            t_vec = fm.torque_world if fm else np.zeros(3)

            w.writerow([
                f"{now:.6f}",
                *[f"{a:.4f}" for a in angles],
                *[f"{x:.6f}" for x in ee_pos],
                *[f"{x:.6f}" for x in ee_quat],
                *[f"{c:.4f}" for c in currents],
                *[f"{x:.6f}" for x in f_vec],
                *[f"{x:.6f}" for x in t_vec],
            ])

            time.sleep(max(0.0, period - (time.time() - t0 - now)))

    driver.power_off()
    print(f"[logger] wrote {out_path} ({args.duration}s @ {args.rate}Hz)")


def _build_jacobian_fn(urdf_path: str):
    """Stub Jacobian provider — returns identity until URDF parser is wired."""
    def jacobian_fn(joint_angles_deg):
        return np.eye(6)
    return jacobian_fn


if __name__ == "__main__":
    main()

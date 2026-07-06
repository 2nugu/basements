"""trajectory_replayer — replay logged trajectory through MPMSolver, output predicted F/T.

Reads trajectory CSV (from trajectory_logger.py), runs MPMSolver step-by-step
mirroring the recorded end-effector pose as a rigid collider, and writes
predicted force/torque per tick. Downstream `compare_real_vs_sim.py` then
diffs predicted vs measured for paper §7.

This is the *offline-coupled* path. No real-time constraint — sim can take
arbitrarily long per step.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from .config_loader import load_sandbox_yaml


def main():
    p = argparse.ArgumentParser(description="Replay logged trajectory in sim.")
    p.add_argument("--trajectory", required=True, help="trajectory CSV from logger")
    p.add_argument("--sandbox",    required=True, help="YAML sandbox config")
    p.add_argument("--output",     required=True, help="output predicted F/T CSV")
    args = p.parse_args()

    sandbox = load_sandbox_yaml(args.sandbox)

    # MPMSolver Python bindings will live in `basements_py` (Phase 4
    # pybind11 module). For W3 we expose a placeholder runner that
    # writes a zero-force CSV with the correct schema, demonstrating
    # the data flow. Replace with real binding when pybind11 lands.
    print(f"[replayer] sandbox: {sandbox.name}")
    print(f"[replayer] E={sandbox.E_pa:.2e} Pa, ν={sandbox.nu}, ρ={sandbox.rho_bulk}")
    print(f"[replayer] α_calibrated={sandbox.alpha_calibrated}")
    print(f"[replayer] dx={sandbox.recommended_dx_m} m, dt={sandbox.recommended_dt_s} s")

    in_path = Path(args.trajectory)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rows_read = 0
    with in_path.open() as fin, out_path.open("w", newline="") as fout:
        r = csv.DictReader(fin)
        w = csv.writer(fout)
        w.writerow([
            "t_sec",
            "ee_x_m", "ee_y_m", "ee_z_m",
            "predicted_f_x_n", "predicted_f_y_n", "predicted_f_z_n",
            "predicted_t_x_nm", "predicted_t_y_nm", "predicted_t_z_nm",
        ])
        for row in r:
            rows_read += 1
            # PLACEHOLDER: zero force until pybind11 binding lands.
            # The trajectory data is preserved so downstream compare.py
            # has matching timebase.
            w.writerow([
                row["t_sec"],
                row["ee_x_m"], row["ee_y_m"], row["ee_z_m"],
                "0.0", "0.0", "0.0",
                "0.0", "0.0", "0.0",
            ])
    print(f"[replayer] wrote {out_path} ({rows_read} rows)")
    print("[replayer] NOTE: predicted F/T are placeholders pending pybind11 MPMSolver.")


if __name__ == "__main__":
    main()

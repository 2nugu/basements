"""paper_table_emit.py — auto-generate §5 markdown tables from bench CSVs.

Eliminates manual transcription error between bench output and paper
text. Run after every full `bench_compare_all --all` to refresh the
tables in `docs/paper_outline.md` programmatically.

Output sections (markdown printed to stdout, optionally to file):
  - §5.S1 box_drop          (max|Σp_y|, avg, vs M0, |ΔE/E₀|, step μs, dwell)
  - §5.A  box_drop_dense    (max|Σp_y|, vs M1, dwell)
  - §5.B  rover_wheel       (max|Σp|, slip, drawbar, sinkage)
  - §5.S2 foot_step         (stopping speed = -v_down_peak)

Usage:
    python research/mpm_rigid_coupling/scripts/paper_table_emit.py
    python ...                                              > tables.md
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
CSV_DIR = HERE / "outputs" / "csv"

METHODS = ["M0", "M1", "M2", "M3", "M4", "M4.5a", "M4.5b", "M4.5"]


def _read_metrics(scenario: str, method: str) -> dict[str, str] | None:
    path = CSV_DIR / f"{scenario}_{method}_metrics.csv"
    if not path.exists():
        return None
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        return next(r)


def _read_roll(method: str) -> dict[str, str] | None:
    path = CSV_DIR / f"rover_wheel_{method}_roll.csv"
    if not path.exists():
        return None
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        return next(r)


def _read_rebound(method: str) -> dict[str, str] | None:
    path = CSV_DIR / f"foot_step_{method}_rebound.csv"
    if not path.exists():
        return None
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        return next(r)


# ─── §5.S1 box_drop ───────────────────────────────────────────────────────

def emit_box_drop(out):
    print("### §5.S1 box_drop — light body (auto-generated)", file=out)
    print(file=out)
    print("| Method | max\\|Σp_y\\| | avg\\|Σp_y\\| | vs M0 | \\|ΔE/E₀\\| | step μs | dwell % |", file=out)
    print("|---|---|---|---|---|---|---|", file=out)
    m0 = _read_metrics("box_drop", "M0")
    if m0 is None:
        print("_(missing box_drop_M0_metrics.csv — run `bench --all` first)_", file=out)
        return
    m0_max = float(m0["max_abs_total_p_y"])
    for m in METHODS:
        row = _read_metrics("box_drop", m)
        if row is None:
            continue
        max_p = float(row["max_abs_total_p_y"])
        avg_p = float(row["avg_abs_total_p_y"])
        de    = float(row["max_abs_energy_drift"])
        step  = float(row["step_time_avg_us"])
        dwell = float(row["dwell_fraction"]) * 100
        bold = "**" if m == "M4.5a" else ""
        print(f"| {bold}{m:<5}{bold} | {bold}{max_p:.4g}{bold} | {avg_p:.4g} | "
              f"{max_p/m0_max:.3f}× | {de:.3f} | {step:.1f} | {dwell:.0f} |", file=out)
    print(file=out)


# ─── §5.A box_drop_dense ──────────────────────────────────────────────────

def emit_box_drop_dense(out):
    print("### §5.A box_drop_dense — mass-invariance (auto-generated)", file=out)
    print(file=out)
    print("| Method | max\\|Σp_y\\| | vs M1 | dwell % |", file=out)
    print("|---|---|---|---|", file=out)
    m1 = _read_metrics("box_drop_dense", "M1")
    if m1 is None:
        print("_(missing box_drop_dense_M1_metrics.csv)_", file=out)
        return
    m1_max = float(m1["max_abs_total_p_y"])
    for m in METHODS:
        row = _read_metrics("box_drop_dense", m)
        if row is None:
            continue
        max_p = float(row["max_abs_total_p_y"])
        dwell = float(row["dwell_fraction"]) * 100
        bold = "**" if m == "M4.5a" else ""
        print(f"| {bold}{m:<5}{bold} | {bold}{max_p:.4g}{bold} | "
              f"{max_p/m1_max:.3f}× | {dwell:.0f} |", file=out)
    print(file=out)


# ─── §5.B rover_wheel ─────────────────────────────────────────────────────

def emit_rover_wheel(out):
    print("### §5.B rover_wheel — cylinder rolling (auto-generated)", file=out)
    print(file=out)
    print("| Method | max\\|Σp\\| | slip | drawbar (N) | sinkage (m) |", file=out)
    print("|---|---|---|---|---|", file=out)
    for m in METHODS:
        roll = _read_roll(m)
        mtr  = _read_metrics("rover_wheel", m)
        if roll is None or mtr is None:
            continue
        max_p = float(mtr["max_abs_total_p_y"])
        slip  = float(roll["avg_slip"])
        draw  = float(roll["avg_drawbar_N"])
        sink  = float(roll["avg_sinkage_m"])
        bold  = "**" if m == "M4.5a" else ""
        sink_note = " (saturated)" if abs(sink - 0.30) < 1e-3 else ""
        print(f"| {bold}{m:<5}{bold} | {max_p:.3g} | {slip:.2f} | "
              f"{draw:+.3f} | {sink:.3f}{sink_note} |", file=out)
    print(file=out)


# ─── §5.S2 foot_step ──────────────────────────────────────────────────────

def emit_foot_step(out):
    print("### §5.S2 foot_step — stopping speed (auto-generated)", file=out)
    print(file=out)
    print("| Method | stopping speed (m/s) | rebound e |", file=out)
    print("|---|---|---|", file=out)
    for m in METHODS:
        reb = _read_rebound(m)
        if reb is None:
            continue
        v_down = float(reb["v_down_peak"])
        e      = float(reb["rebound_coefficient_e"])
        bold   = "**" if m == "M4.5a" else ""
        print(f"| {bold}{m:<5}{bold} | {-v_down:.2f} | {e:.3f} |", file=out)
    print(file=out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", "-o", default=None,
                        help="write to file instead of stdout")
    args = parser.parse_args()

    if args.output:
        out = open(args.output, "w", encoding="utf-8")
    else:
        # Force UTF-8 on Windows cp949 default — em-dashes and Greek
        # letters in headings would otherwise crash.
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8")
        out = sys.stdout
    try:
        print("<!-- AUTO-GENERATED by paper_table_emit.py; do not edit -->", file=out)
        print(file=out)
        emit_box_drop(out)
        emit_box_drop_dense(out)
        emit_rover_wheel(out)
        emit_foot_step(out)
    finally:
        if args.output:
            out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

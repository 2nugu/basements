"""reproduce_findings.py — automated verification of paper findings 1-5.

Reads the per-method metrics + rolling + rebound CSVs that
`bench_compare_all --all` already produced, and checks that each
finding's predicate holds. Exits 0 if all pass, non-zero otherwise.

This is the *minimum viable reproducibility check*: it verifies that
the most recent measurement run satisfies the qualitative claims
made in paper Section 5/6. A future version should iterate over
dt ∈ {1/60, 1/120, 1/240} × initial offsets and re-build before
each pass — that requires extending `bench_compare_all` to accept
`--dt` / `--offset-*` flags. For now we verify the qualitative
ranking, which is the load-bearing claim of every Finding.

Usage:
    python scripts/reproduce_findings.py
    (No arguments. Reads CSVs from research/.../outputs/csv/.)
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
CSV_DIR = HERE / "outputs" / "csv"


def _read_metric(scenario: str, method: str, key: str) -> float:
    path = CSV_DIR / f"{scenario}_{method}_metrics.csv"
    if not path.exists():
        raise FileNotFoundError(f"missing {path}")
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        row = next(r)
        return float(row[key])


def _read_roll(method: str, key: str) -> float:
    path = CSV_DIR / f"rover_wheel_{method}_roll.csv"
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        row = next(r)
        return float(row[key])


def _read_rebound(method: str, key: str) -> float:
    path = CSV_DIR / f"foot_step_{method}_rebound.csv"
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        row = next(r)
        return float(row[key])


# ─── Each finding is a callable returning (ok, message) ───────────────────


def finding_1_m45a_beats_m1_box_drop():
    """M4.5a reduces max|Σp_y| vs M1 on box_drop. Expected ratio ≈ 0.46×."""
    m1   = _read_metric("box_drop",       "M1",    "max_abs_total_p_y")
    m45a = _read_metric("box_drop",       "M4.5a", "max_abs_total_p_y")
    ratio = m45a / m1
    return (ratio < 0.60,
            f"box_drop  M4.5a/M1 = {ratio:.3f}  (target < 0.60)")


def finding_1b_ratio_invariant_under_dense():
    """Same ratio holds when body mass is 9×."""
    m1d   = _read_metric("box_drop_dense", "M1",    "max_abs_total_p_y")
    m45ad = _read_metric("box_drop_dense", "M4.5a", "max_abs_total_p_y")
    ratio = m45ad / m1d
    return (ratio < 0.60,
            f"box_drop_dense  M4.5a/M1 = {ratio:.3f}  (target < 0.60)")


def finding_2_rover_wheel_layer_tradeoff():
    """Rover wheel: M4.5a maximises surface support, M4.5b minimises slip."""
    sink_a = _read_roll("M4.5a", "avg_sinkage_m")
    sink_b = _read_roll("M4.5b", "avg_sinkage_m")
    slip_a = _read_roll("M4.5a", "avg_slip")
    slip_b = _read_roll("M4.5b", "avg_slip")
    ok_support  = sink_a < sink_b
    ok_traction = slip_b < slip_a
    return (ok_support and ok_traction,
            f"M4.5a sinkage {sink_a:.3f} < M4.5b sinkage {sink_b:.3f} "
            f"({'pass' if ok_support else 'FAIL'});  "
            f"M4.5b slip {slip_b:.2f} < M4.5a slip {slip_a:.2f} "
            f"({'pass' if ok_traction else 'FAIL'})")


def finding_3_m0_degrades_with_mass():
    """M0 max|Σp| inflates 10× when body mass scales 1→9 kg."""
    m0_light = _read_metric("box_drop",       "M0", "max_abs_total_p_y")
    m0_dense = _read_metric("box_drop_dense", "M0", "max_abs_total_p_y")
    factor = m0_dense / m0_light
    return (factor > 10.0,
            f"M0 dense/light = {factor:.1f}×  (target > 10×)")


def finding_4_composition_not_free():
    """M4.5b and M4.5c regress vs M4.5a on box_drop conservation."""
    p_a = _read_metric("box_drop", "M4.5a", "max_abs_total_p_y")
    p_b = _read_metric("box_drop", "M4.5b", "max_abs_total_p_y")
    p_c = _read_metric("box_drop", "M4.5",  "max_abs_total_p_y")
    ok_b = p_b > p_a
    ok_c = p_c > 10.0 * p_a
    return (ok_b and ok_c,
            f"M4.5b ({p_b:.3e}) > M4.5a ({p_a:.3e}) {'pass' if ok_b else 'FAIL'};  "
            f"M4.5c ({p_c:.3e}) > 10·M4.5a ({10*p_a:.3e}) {'pass' if ok_c else 'FAIL'}")


def finding_5_no_rebound_in_dry_sand():
    """All methods produce e ≈ 0 on foot_step (sand is elastic-only)."""
    es = {m: _read_rebound(m, "rebound_coefficient_e")
          for m in ("M0", "M1", "M4.5a", "M4.5b", "M4.5")}
    ok = all(e < 0.05 for e in es.values())
    return (ok, f"foot_step rebound e by method: {es}")


# ─── Driver ───────────────────────────────────────────────────────────────

CHECKS = [
    ("Finding 1   M4.5a beats M1 (box_drop)",          finding_1_m45a_beats_m1_box_drop),
    ("Finding 1   Mass-invariance (box_drop_dense)",   finding_1b_ratio_invariant_under_dense),
    ("Finding 2   Rover-wheel layer trade-off",        finding_2_rover_wheel_layer_tradeoff),
    ("Finding 3   M0 degrades with mass",              finding_3_m0_degrades_with_mass),
    ("Finding 4   Composition is not free",            finding_4_composition_not_free),
    ("Finding 5   No rebound in dry sand",             finding_5_no_rebound_in_dry_sand),
]


def main() -> int:
    n_pass = 0
    n_fail = 0
    print(f"{'':<5}{'finding':<48}{'verdict':<10}message")
    print("-" * 100)
    for label, fn in CHECKS:
        try:
            ok, msg = fn()
        except FileNotFoundError as e:
            print(f"[??]  {label:<48}{'MISSING':<10}{e}")
            n_fail += 1
            continue
        verdict = "PASS" if ok else "FAIL"
        print(f"[{verdict}] {label:<48}{'':<10}{msg}")
        if ok: n_pass += 1
        else:  n_fail += 1
    print("-" * 100)
    print(f"summary: {n_pass} pass / {n_fail} fail")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

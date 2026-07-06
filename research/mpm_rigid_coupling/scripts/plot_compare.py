"""plot_compare.py — produces a 4-panel analysis figure per scenario.

Panels:
  (a) Top-left   — box height time series (kinematic narrative)
  (b) Top-right  — |Σp_y| time series on log y-axis (conservation focus)
  (c) Bot-left   — Pareto: step_time_avg vs max|Σp|, one dot per method
  (d) Bot-right  — bar chart: max|Σp| per method (sorted)

Also emits a summary table in <name>_desc.md backed by per-method
metrics CSVs written by bench_compare_all.
"""
from __future__ import annotations

import csv
import sys
from datetime import date
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent.parent
CSV_DIR = HERE / "outputs" / "csv"
FIG_DIR = HERE / "outputs" / "figures"
METHODS = ["M0", "M1", "M2", "M3", "M4", "M4.5a", "M4.5b", "M4.5"]
SCENARIOS = ["box_drop", "box_drop_dense", "rover_wheel", "foot_step"]

# Sinkage saturation cap = 2 × wheel_radius. Hardcoded to match
# bench_compare_all.cpp's clamping. If you change the wheel radius in
# scenario_rover_wheel.h, update this too.
SINKAGE_SATURATION_M = 0.30   # 2 × 0.15 (wheel_radius)
SINKAGE_SATURATION_EPS = 1e-4


def _read_ts(path: Path):
    with path.open(newline="") as f:
        r = csv.reader(f); next(r)
        cols = [[] for _ in range(10)]
        for row in r:
            for i, v in enumerate(row[:10]):
                cols[i].append(float(v))
    return cols


def _read_roll(path: Path):
    """Read the rolling-specific summary CSV (one row per method)."""
    if not path.exists():
        return None
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        row = next(r, None)
        if not row:
            return None
        out = {}
        for k, v in row.items():
            if k == "method":
                continue
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
        return out


def _format_sinkage(value: float) -> str:
    """Mark saturated sinkage explicitly so paper readers do not mistake the
    clamp value for a real physical measurement."""
    if abs(value - SINKAGE_SATURATION_M) <= SINKAGE_SATURATION_EPS:
        return f"≥{SINKAGE_SATURATION_M:.3f} (saturated)"
    return f"{value:.3f}"


def _read_metrics(path: Path):
    if not path.exists():
        return None
    with path.open(newline="") as f:
        r = csv.DictReader(f)
        row = next(r, None)
        if not row:
            return None
        out = {
            "max_abs_total_p_y":    float(row["max_abs_total_p_y"]),
            "max_abs_energy_drift": float(row["max_abs_energy_drift"]),
            "step_time_avg_us":     float(row["step_time_avg_us"]),
            "max_penetration_m":    float(row["max_penetration_m"]),
        }
        # New columns (post-Round 10); tolerate older CSVs.
        for k in ("avg_abs_total_p_y", "avg_penetration_m",
                  "dwell_fraction", "ratio_p_y_vs_M0"):
            if k in row:
                out[k] = float(row[k])
        return out


def _xticks(t):
    if not t: return []
    n = 5
    step = (t[-1] - t[0]) / (n - 1) if n > 1 else 0
    return [t[0] + i * step for i in range(n)]


def plot_scenario(scenario: str):
    methods_with_data = []
    series = {}
    summary = {}
    for m in METHODS:
        ts_path = CSV_DIR / f"{scenario}_{m}.csv"
        mt_path = CSV_DIR / f"{scenario}_{m}_metrics.csv"
        if not ts_path.exists():
            print(f"[plot] missing time series for {m}")
            continue
        cols = _read_ts(ts_path)
        if not cols[0]:
            continue
        series[m] = cols
        summary[m] = _read_metrics(mt_path) or {}
        methods_with_data.append(m)
    if not methods_with_data:
        return None

    # Stable colour per method.
    palette = plt.cm.tab10.colors
    colour = {m: palette[i % len(palette)] for i, m in enumerate(methods_with_data)}

    fig = plt.figure(figsize=(12, 9), constrained_layout=True)
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 2, 2])
    # Panel (a) is split into two sub-axes (M0 lives way below the rest)
    ax_y_low  = fig.add_subplot(gs[0, 0])
    ax_y_high = fig.add_subplot(gs[1, 0], sharex=ax_y_low)
    ax_p      = fig.add_subplot(gs[0:2, 1])
    ax_pareto = fig.add_subplot(gs[2, 0])
    ax_bar    = fig.add_subplot(gs[2, 1])

    # Panel (a) — broken y-axis. Top sub-axis: "stays on surface" group;
    # bottom sub-axis: "penetrates" group (M0 in particular).
    # We pick the split at -0.1 m: anything that goes below counts as
    # punching through the sand.
    BREAK = -0.1
    for m in methods_with_data:
        cols = series[m]
        ax_y_low.plot(cols[0], cols[2], label=m, color=colour[m])
        ax_y_high.plot(cols[0], cols[2], color=colour[m])
    ax_y_low.set_ylim(top=1.0)              # auto-bottom; show full upper range
    ax_y_low.set_ylim(bottom=BREAK)
    ax_y_high.set_ylim(top=BREAK)
    ax_y_high.set_ylim(bottom=-3.0)
    # Hide spines at the break.
    ax_y_low.spines["bottom"].set_visible(False)
    ax_y_high.spines["top"].set_visible(False)
    ax_y_low.tick_params(labelbottom=False)
    ax_y_high.set_xlabel("time (s)")
    ax_y_low.set_ylabel("y (m) — surface")
    ax_y_high.set_ylabel("y (m) — penetrating")
    ax_y_low.set_title("(a) Kinematic — body height (broken y-axis)")
    ax_y_low.grid(alpha=0.3)
    ax_y_high.grid(alpha=0.3)
    ax_y_low.legend(bbox_to_anchor=(1.01, 1), loc="upper left", fontsize=8)

    # Panel (b) — |Σp_y| log-scale
    for m in methods_with_data:
        cols = series[m]
        absp = [abs(v) + 1e-12 for v in cols[7]]
        ax_p.plot(cols[0], absp, label=m, color=colour[m])
    ax_p.set_yscale("log")
    ax_p.set_ylabel("|Σ p_y|  (kg·m/s, log)")
    ax_p.set_xlabel("time (s)")
    ax_p.set_title("(b) Momentum conservation error")
    ax_p.grid(alpha=0.3, which="both")

    # Panel (c) — Pareto: step_time vs max|Σp|
    for m in methods_with_data:
        s = summary.get(m, {})
        if "step_time_avg_us" in s and "max_abs_total_p_y" in s:
            ax_pareto.scatter([s["step_time_avg_us"]],
                              [s["max_abs_total_p_y"] + 1e-12],
                              color=colour[m], s=80)
            ax_pareto.annotate(m,
                               (s["step_time_avg_us"], s["max_abs_total_p_y"] + 1e-12),
                               xytext=(5, 5), textcoords="offset points", fontsize=9)
    ax_pareto.set_xlabel("avg step time (μs)")
    ax_pareto.set_ylabel("max |Σ p_y| (kg·m/s, log)")
    ax_pareto.set_yscale("log")
    ax_pareto.set_title("(c) Cost vs accuracy — Pareto")
    ax_pareto.grid(alpha=0.3, which="both")

    # Panel (d) — bar chart of max|Σp| by method, sorted asc
    items = sorted(
        [(m, summary[m].get("max_abs_total_p_y", float("nan")))
         for m in methods_with_data if "max_abs_total_p_y" in summary[m]],
        key=lambda kv: kv[1])
    if items:
        names = [kv[0] for kv in items]
        vals  = [kv[1] for kv in items]
        bars = ax_bar.barh(names, vals,
                           color=[colour[n] for n in names])
        ax_bar.set_xscale("log")
        ax_bar.set_xlabel("max |Σ p_y| (kg·m/s, log)")
        ax_bar.set_title("(d) Conservation error per method")
        ax_bar.grid(alpha=0.3, axis="x", which="both")

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    png = FIG_DIR / f"compare_{scenario}.png"
    fig.savefig(png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plot] wrote {png}")

    # ----- desc.md summary table -----
    # Compute ratio_vs_M0 (Python-side) for the summary table.
    base_max_p = summary.get("M0", {}).get("max_abs_total_p_y")

    md_lines = [
        f"# Figure: compare_{scenario}",
        "",
        "## Purpose",
        f"Four-panel quantitative comparison of M0..M4.5 couplers on the "
        f"{scenario} scenario. See THEORY.md for governing equations and "
        f"the assumption-removal ladder M0→M4.5.",
        "",
        "## Data Source",
        f"- Time series: outputs/csv/{scenario}_M*.csv",
        f"- Per-method metrics: outputs/csv/{scenario}_M*_metrics.csv",
        f"- Generated: bench_compare_all + scripts/plot_compare.py",
        f"- Date: {date.today().isoformat()}",
        "",
        "## Panels",
        "- (a) box height y(t)",
        "- (b) |Σ p_y|(t) on log y-axis — pure conservation lens",
        "- (c) Pareto: step time (μs) vs max|Σ p_y| — cost-accuracy trade-off",
        "- (d) bar chart: max|Σ p_y| per method, sorted ascending",
        "",
        "## Per-method metrics",
        "| Method | max\\|Σp_y\\| | avg\\|Σp_y\\| | vs M0 | \\|ΔE/E₀\\| | step μs | pen_max | pen_avg | dwell % |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for m in methods_with_data:
        s = summary.get(m, {})
        max_p = s.get("max_abs_total_p_y", float("nan"))
        ratio = (max_p / base_max_p) if (base_max_p and base_max_p > 0) else float("nan")
        md_lines.append(
            "| {m} | {p:.3e} | {pa:.3e} | {r:.2f}× | {e:.3e} | {t:.1f} | {dm:.3e} | {da:.3e} | {dw:.1f} |".format(
                m=m,
                p=max_p,
                pa=s.get("avg_abs_total_p_y", float("nan")),
                r=ratio,
                e=s.get("max_abs_energy_drift", float("nan")),
                t=s.get("step_time_avg_us", float("nan")),
                dm=s.get("max_penetration_m", float("nan")),
                da=s.get("avg_penetration_m", float("nan")),
                dw=100.0 * s.get("dwell_fraction", float("nan"))))
    md_lines += [
        "",
        "## Interpretation cheat sheet",
        "- max\\|Σp_y\\| is the headline number. Target: 1e-5 for M4 (float roundoff).",
        "- Bar chart (d) shows the ranking; Pareto (c) shows whether cheaper",
        "  methods are dominated.",
        "- Stub methods (those currently delegating to M1) appear at the same",
        "  position as M1 — see THEORY.md status table for which are stubs.",
        "",
        "## Reproduction",
        "```",
        "cmake --build build --config Release --target bench_compare_all",
        "./build/Release/bench_compare_all.exe --all",
        "python research/mpm_rigid_coupling/scripts/plot_compare.py",
        "```",
        "",
    ]
    (FIG_DIR / f"compare_{scenario}_desc.md").write_text(
        "\n".join(md_lines), encoding="utf-8")
    print(f"[plot] wrote {FIG_DIR / f'compare_{scenario}_desc.md'}")


def main(argv):
    for s in SCENARIOS:
        plot_scenario(s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

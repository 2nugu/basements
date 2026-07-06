# Figure: compare_foot_step

## Purpose
Four-panel quantitative comparison of M0..M4.5 couplers on the foot_step scenario. See THEORY.md for governing equations and the assumption-removal ladder M0→M4.5.

## Data Source
- Time series: outputs/csv/foot_step_M*.csv
- Per-method metrics: outputs/csv/foot_step_M*_metrics.csv
- Generated: bench_compare_all + scripts/plot_compare.py
- Date: 2026-05-15

## Panels
- (a) box height y(t)
- (b) |Σ p_y|(t) on log y-axis — pure conservation lens
- (c) Pareto: step time (μs) vs max|Σ p_y| — cost-accuracy trade-off
- (d) bar chart: max|Σ p_y| per method, sorted ascending

## Per-method metrics
| Method | max\|Σp_y\| | avg\|Σp_y\| | vs M0 | \|ΔE/E₀\| | step μs | pen_max | pen_avg | dwell % |
|---|---|---|---|---|---|---|---|---|
| M0 | 1.902e+01 | 9.312e+00 | 1.00× | 3.519e-01 | 72.9 | 1.970e-02 | 2.493e-04 | 1.9 |
| M1 | 1.921e+00 | 1.470e-01 | 0.10× | 5.346e-01 | 75.8 | 1.999e-02 | 9.683e-03 | 70.2 |
| M2 | 1.921e+00 | 1.470e-01 | 0.10× | 5.346e-01 | 74.8 | 1.999e-02 | 9.683e-03 | 70.2 |
| M3 | 1.921e+00 | 1.470e-01 | 0.10× | 5.346e-01 | 75.3 | 1.999e-02 | 9.683e-03 | 70.2 |
| M4 | 1.921e+00 | 1.470e-01 | 0.10× | 5.346e-01 | 74.8 | 1.999e-02 | 9.683e-03 | 70.2 |
| M4.5a | 1.921e+00 | 2.043e-01 | 0.10× | 7.429e-01 | 77.0 | 1.998e-02 | 9.785e-03 | 64.6 |
| M4.5b | 2.330e+00 | 4.006e-01 | 0.12× | 9.535e-01 | 75.4 | 2.000e-02 | 7.078e-03 | 47.7 |
| M4.5 | 1.635e+01 | 7.063e+00 | 0.86× | 9.101e-01 | 74.6 | 1.955e-02 | 5.555e-04 | 4.0 |

## Interpretation cheat sheet
- max\|Σp_y\| is the headline number. Target: 1e-5 for M4 (float roundoff).
- Bar chart (d) shows the ranking; Pareto (c) shows whether cheaper
  methods are dominated.
- Stub methods (those currently delegating to M1) appear at the same
  position as M1 — see THEORY.md status table for which are stubs.

## Reproduction
```
cmake --build build --config Release --target bench_compare_all
./build/Release/bench_compare_all.exe --all
python research/mpm_rigid_coupling/scripts/plot_compare.py
```

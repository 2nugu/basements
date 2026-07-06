# Figure: compare_rover_wheel

## Purpose
Four-panel quantitative comparison of M0..M4.5 couplers on the rover_wheel scenario. See THEORY.md for governing equations and the assumption-removal ladder M0→M4.5.

## Data Source
- Time series: outputs/csv/rover_wheel_M*.csv
- Per-method metrics: outputs/csv/rover_wheel_M*_metrics.csv
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
| M0 | 2.708e+01 | 1.262e+01 | 1.00× | 8.953e-01 | 169.1 | 3.198e-04 | 1.324e-05 | 0.0 |
| M1 | 9.416e+00 | 1.902e+00 | 0.35× | 1.513e+01 | 175.2 | 2.113e-02 | 3.724e-03 | 2.5 |
| M2 | 9.416e+00 | 1.902e+00 | 0.35× | 1.513e+01 | 175.4 | 2.113e-02 | 3.724e-03 | 2.5 |
| M3 | 9.416e+00 | 1.902e+00 | 0.35× | 1.513e+01 | 176.2 | 2.113e-02 | 3.724e-03 | 2.5 |
| M4 | 9.416e+00 | 1.902e+00 | 0.35× | 1.513e+01 | 177.8 | 2.113e-02 | 3.724e-03 | 2.5 |
| M4.5a | 4.604e+00 | 3.041e-01 | 0.17× | 7.523e+01 | 197.2 | 2.938e-02 | 9.745e-03 | 27.5 |
| M4.5b | 1.126e+01 | 2.324e+00 | 0.42× | 3.745e+00 | 195.1 | 4.697e-02 | 8.604e-03 | 26.2 |
| M4.5 | 2.656e+01 | 1.217e+01 | 0.98× | 9.450e-01 | 173.4 | 1.194e-03 | 4.601e-05 | 0.0 |

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

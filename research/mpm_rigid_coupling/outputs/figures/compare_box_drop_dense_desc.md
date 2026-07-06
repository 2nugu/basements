# Figure: compare_box_drop_dense

## Purpose
Four-panel quantitative comparison of M0..M4.5 couplers on the box_drop_dense scenario. See THEORY.md for governing equations and the assumption-removal ladder M0→M4.5.

## Data Source
- Time series: outputs/csv/box_drop_dense_M*.csv
- Per-method metrics: outputs/csv/box_drop_dense_M*_metrics.csv
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
| M0 | 2.416e+02 | 1.116e+02 | 1.00× | 8.760e-01 | 76.3 | 1.500e-01 | 1.363e-02 | 9.7 |
| M1 | 1.611e+00 | 7.980e-01 | 0.01× | 4.833e-01 | 83.0 | 1.500e-01 | 1.251e-01 | 87.9 |
| M2 | 1.611e+00 | 7.980e-01 | 0.01× | 4.833e-01 | 83.1 | 1.500e-01 | 1.251e-01 | 87.9 |
| M3 | 1.611e+00 | 7.980e-01 | 0.01× | 4.833e-01 | 84.7 | 1.500e-01 | 1.251e-01 | 87.9 |
| M4 | 1.611e+00 | 7.980e-01 | 0.01× | 4.833e-01 | 84.0 | 1.500e-01 | 1.251e-01 | 87.9 |
| M4.5a | 7.358e-01 | 3.724e-01 | 0.00× | 2.254e-01 | 84.2 | 1.241e-01 | 6.267e-02 | 52.2 |
| M4.5b | 1.167e+00 | 9.200e-01 | 0.00× | 5.565e-01 | 85.6 | 1.500e-01 | 1.082e-01 | 76.4 |
| M4.5 | 2.423e+02 | 1.123e+02 | 1.00× | 8.975e-01 | 75.4 | 1.500e-01 | 1.606e-02 | 11.5 |

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

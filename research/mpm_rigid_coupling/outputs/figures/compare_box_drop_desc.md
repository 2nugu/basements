# Figure: compare_box_drop

## Purpose
Four-panel quantitative comparison of M0..M4.5 couplers on the box_drop scenario. See THEORY.md for governing equations and the assumption-removal ladder M0→M4.5.

## Data Source
- Time series: outputs/csv/box_drop_M*.csv
- Per-method metrics: outputs/csv/box_drop_M*_metrics.csv
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
| M0 | 6.256e+00 | 8.786e-01 | 1.00× | 1.166e+00 | 78.2 | 1.500e-01 | 1.026e-01 | 71.4 |
| M1 | 1.790e-01 | 8.861e-02 | 0.03× | 4.830e-01 | 83.8 | 1.500e-01 | 1.251e-01 | 87.9 |
| M2 | 1.790e-01 | 8.861e-02 | 0.03× | 4.830e-01 | 84.5 | 1.500e-01 | 1.251e-01 | 87.9 |
| M3 | 1.790e-01 | 8.861e-02 | 0.03× | 4.830e-01 | 85.0 | 1.500e-01 | 1.251e-01 | 87.9 |
| M4 | 1.790e-01 | 8.861e-02 | 0.03× | 4.830e-01 | 86.0 | 1.500e-01 | 1.251e-01 | 87.9 |
| M4.5a | 8.175e-02 | 4.137e-02 | 0.01× | 2.254e-01 | 92.4 | 1.241e-01 | 6.267e-02 | 52.2 |
| M4.5b | 1.290e-01 | 1.013e-01 | 0.02× | 5.513e-01 | 88.7 | 1.500e-01 | 1.081e-01 | 76.4 |
| M4.5 | 2.675e+01 | 1.233e+01 | 4.28× | 9.205e-01 | 76.4 | 1.500e-01 | 1.657e-02 | 11.9 |

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

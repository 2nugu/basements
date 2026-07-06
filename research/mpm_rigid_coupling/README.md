# MPM ↔ Rigid Coupling Benchmark Suite (M0 … M4.5)

> **Status (Pivot 6, 2026-05-16):** this sub-project is now the
> **forward-sim foundation for paper #2** (Differentiable MPM-Rigid
> Coupling for Agricultural Sim-to-Real). Paper #1 was dropped — see
> [`../../docs/PROJECT_STATUS.md::Pivot 6`](../../docs/PROJECT_STATUS.md).
> Existing scenarios (box_drop, box_drop_dense, rover_wheel, foot_step,
> tine_drag, repose_pile) + ablation methods (M0/M1/M4.5{a,b,c}) +
> validation tests are *unchanged* and carry into paper #2 §3.1 forward
> sim + §4.1 forward validation tables.
>
> **New in paper #2 (in-place autodiff layer):** each method's coupling
> kernel gains an opt-in tape-recording path
> (`include/.../mpm_autodiff_tape.h`) so the same `apply()` function
> serves both forward-only and differentiable rollouts.
> `test_mpm_autodiff_tape_neutral` enforces bit-exact equivalence in CI.
>
> See [`../../docs/notes/pivot6_differentiable_mpm.md`](../../docs/notes/pivot6_differentiable_mpm.md)
> for paper #2 architecture and the M1-M9 implementation roadmap.

Self-contained research sub-project. Eight coupling schemes evaluated on
six robotics-relevant scenarios. Produces the forward-validation figures
backing Paper #2 §4.1.

## Method matrix

| ID     | Name                                  | Idea                                            | Reference        | Status |
|--------|---------------------------------------|-------------------------------------------------|------------------|--------|
| M0     | Dirichlet BC                          | grid velocity clamped to body velocity          | engineering MVP  | ✅ real |
| M1     | PIC-Rigid (basic)                     | mass-weighted average exchange                  | Stomakhin 2014   | ✅ real |
| M2     | ASFLIP rigid extension                | F-bar split, momentum-exact normal exchange     | Fei 2021         | 🟡 stub |
| M3     | MLS-MPM rigid                         | Moving Least Squares kernel + rigid coupling    | Hu 2018          | 🟡 stub |
| M4     | Lagrange-multiplier saddle            | constrained solve, momentum exact in solver tol | Jiang 2020       | 🟡 stub |
| M4.5a  | Hybrid — Layer A only                 | M1 + renormalised cell-overlap mass weighting   | this work        | ✅ real |
| M4.5b  | Hybrid — Layer A + B (friction)       | A + Coulomb tangent projection                  | this work        | ✅ real |
| M4.5   | Hybrid — Layer A + B + C (full)       | B + quadratic B-spline kernel                   | this work        | ✅ real |

**Round 10 measured (`box_drop`, 3 s @ 240 Hz, 1 kg box on 0.75 kg sand patch):**

| Method | max\|Σp\| | vs M1 | step μs | dwell % |
|---|---|---|---|---|
| M0    | 6.256 | 35× worse | 21.4 | 73 |
| M1    | 0.179 | 1.00 × | 23.1 | 90 |
| **M4.5a** | **0.082** | **0.46×** ← **headline** | 24.5 | 60 |
| M4.5b | 0.129 | 0.72× | 25.7 | 80 |
| M4.5  | 26.75 | 149× worse (kernel breaks Σm_b)  | 20.0 | 12 |

**Paper headline:** M4.5a alone (renormalised adaptive mass) outperforms M1
by 2.2× at +6% step cost. M4.5b and full M4.5c are documented regressions —
these are the paper's Section 4 ablation evidence, NOT failure to suppress.

## Scenarios

| Scenario        | Stresses                              | Geometry              |
|-----------------|---------------------------------------|------------------------|
| `box_drop`      | impulse exchange, momentum conserv.   | 1 kg box, 0.4 m³       |
| `rover_wheel`   | friction-dominated rolling contact    | cylinder Ø0.2 m × 0.2  |
| `foot_step`     | high-deformation rebound              | flat 0.1 × 0.05 × 0.1  |
| `static_pile`   | settled stack, no-slip ground         | box on 5 cm sand bed   |

All scenarios share the same MPM grid (Klar 2016 sand parameters, dx = 0.1 m).
Each method runs each scenario identically — only the coupler changes.

## Output

All outputs live in `outputs/`:

```
outputs/
├── csv/                              ← per-method time series
│   ├── box_drop_M0.csv
│   ├── box_drop_M1.csv
│   └── ...
├── figures/                          ← paper-ready PNG + desc.md
│   ├── compare_box_drop.png          ← all 5 methods overlaid
│   ├── compare_rover_wheel.png
│   └── ...
└── videos/                           ← matplotlib animation + ffmpeg
    ├── box_drop_M1.mp4
    ├── compare_grid_M0_M4_5.mp4      ← 2×3 grid, side-by-side
    └── ...
```

## Build & run

```powershell
# From repo root:
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release --target bench_compare_all

# Single method, single scenario:
./build/Release/bench_compare_all.exe --method M1 --scenario box_drop

# All methods, all scenarios (long):
./build/Release/bench_compare_all.exe --all

# Regenerate figures and videos:
python research/mpm_rigid_coupling/scripts/plot_compare.py
python research/mpm_rigid_coupling/scripts/render_animation.py
```

## Reproducibility contract

1. All numerical parameters are in `scenarios/*.h` — no magic numbers in
   benchmark code.
2. Each CSV is keyed by `(scenario, method, dt, solver_iters)` so a CSV
   filename uniquely identifies the run.
3. Figure regeneration is one Python invocation; no manual layout tweaks.
4. Videos are produced from the same CSVs (no separate simulation run).

## Relationship to the rest of the repo

- Uses `basements_math`, `basements_coupling` (for M0/M1), `basements_physics`
  (MPMSolver, RigidBody) from the main engine.
- Does **not** require the editor (`basements_editor`) — runs headless.
- Does **not** modify any engine source. New couplers (M2..M4.5) live entirely
  inside this sub-directory.
- All work in this sub-project is independent of the main engine UI / GPU
  roadmap — see `../../docs/roadmap.md`. If main-engine work stalls, this
  sub-project keeps moving toward Paper #1.

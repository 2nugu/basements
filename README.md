# Basements

**Open Differentiable MPM-Rigid Coupling for Agricultural Robot Sim-to-Real**

[![DOI](https://zenodo.org/badge/1290724781.svg)](https://doi.org/10.5281/zenodo.21215512)
[![License: MIT (code) / CC-BY-4.0 (data)](https://img.shields.io/badge/license-MIT%20%2F%20CC--BY--4.0-blue.svg)](./LICENSE)
[![Tests](https://img.shields.io/badge/tests-110%2F110%20passing-brightgreen.svg)](#testing)
[![Phase](https://img.shields.io/badge/phase-2%20differentiable%20sim-blue.svg)](docs/PROJECT_STATUS.md)

Open Material Point Method (MPM) soil solver with Hencky-Kirchhoff stress
and Klar 2016 Drucker-Prager plasticity, coupled to rigid mechanisms
through a layered ablation (M0 Dirichlet, M1 PIC-rigid, M4.5{a,b,c}
hybrid). **Phase-2 active work: differentiable forward + backward sim
with adjoint method through plastic projection, enabling agricultural
robot policy learning with sim-to-real validation on myCobot hardware.**
Primary target: **agricultural robotics** (autonomous tractors,
precision seeders, robotic cultivators, soil-compaction prediction);
secondary regimes: planetary rover wheels on regolith, legged-robot
feet on sand.

**Pivot history:** see [`docs/PROJECT_STATUS.md::Pivot Record`](docs/PROJECT_STATUS.md).
Paper #1 (validated open benchmark) was dropped at Pivot 6 in favour of
paper #2 (differentiable MPM-rigid coupling, ICRA / CoRL / NeurIPS).
The Phase-1 forward sim + scenarios + validation tests are the
paper #2 foundation, not a sunk cost.

> **Project framing.** Basements is intended as a **growing
> platform**, not a paper-specific codebase. Each paper in the
> Pivot-6 series (#2 differentiable, #3 GPU + adaptive real-time,
> #4 field deployment) extends the *same* `MPMSolver` infrastructure
> rather than forking. External contributors who land a feature
> (new coupling method, new constitutive variant, new scenario via
> the case-study registry) gain co-authorship eligibility on future
> papers — see [`CONTRIBUTING.md`](CONTRIBUTING.md) authorship
> policy.

**Author:** Hong-Gu Lee (이홍구), `hgl@kangwon.ac.kr`
Interdisciplinary Program in Smart Agriculture, and
Department of Biosystems Engineering, Kangwon National University,
Chuncheon, Republic of Korea.

**Scope:** single-engineer research project, *not* a Bullet/MuJoCo/Genesis
alternative. The contribution is the *layered ablation* + *constitutive
calibration* + *pre-registered external validation protocol*, not a
production-grade engine.

See [`docs/paper_outline.md`](docs/paper_outline.md) for the paper #1 draft
and [`docs/roadmap.md`](docs/roadmap.md) for the 5-paper research series.

## Where to start

Read in this order:

1. [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md) — current phase, decisions, open questions.
2. [`docs/FORK_NOTE.md`](docs/FORK_NOTE.md) — main engine vs `research/` sub-project boundary.
3. [`docs/roadmap.md`](docs/roadmap.md) — Phase 0–5 plan and calendar-time estimates.
4. [`docs/checklist.md`](docs/checklist.md) — 30/60/90-day actionable items + submission gate.
5. [`docs/notes/limitations.md`](docs/notes/limitations.md) — honest accounting of two structural limits.
6. [`research/mpm_rigid_coupling/README.md`](research/mpm_rigid_coupling/README.md) — paper-track sub-project scope.
7. [`research/mpm_rigid_coupling/THEORY.md`](research/mpm_rigid_coupling/THEORY.md) — math/physics derivation per method.

## What's here

| Area | Status | Entry point |
|---|---|---|
| Math + MPM + rigid solver + joints (Phase 0) | ✅ done | `include/basements/`, `src/`, 110+ tests |
| **Robot-arm sim: URDF import + ROS 2 myCobot sim-to-real bridge** | ✅ done | `src/io/urdf_*`, `extensions/ros2_bridge/` |
| Editor (Phase 0) | ✅ usable, demoted to demo viewer | `apps/editor/` (build from source) |
| **MPM ↔ rigid coupling — M0 / M1 / M4.5{a,b,c} real, M2/M3/M4 stubs** | 🟡 Phase 1 | `research/mpm_rigid_coupling/` |
| **Paper #1 outline + M4.5a 2.2× headline finding** | 🟡 Phase 1 | `docs/paper_outline.md` |
| Robotics validation scenarios (rover_wheel / foot_step) | ⬜ next | gates the paper draft |

## Quick reproduction

```powershell
# Configure + build the benchmark suite (CPU only, ~30s on a modern laptop)
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release --target bench_compare_all

# Run all eight couplers on the box_drop scenario
./build/research/mpm_rigid_coupling/Release/bench_compare_all.exe --all

# Regenerate the 4-panel comparison figure + per-method summary table
python research/mpm_rigid_coupling/scripts/plot_compare.py
```

Outputs land in `research/mpm_rigid_coupling/outputs/{csv,figures,videos}/`.

## Robot-arm simulation & sim-to-real (URDF + ROS 2)

Basements imports a standard **URDF** robot description, builds each link as a
rigid body and each joint as a constraint in the physics world, and steps it in
the *same* solver used for the soil coupling — so a robot arm and the terrain it
acts on can live in one simulation.

- **URDF pipeline** (`include/basements/io/urdf_*.h`, `src/io/urdf_parser.cpp`,
  `urdf_physics_bridge.cpp`, `urdf_exporter.cpp`): parses links / joints /
  inertials / collision geometry, converts them to the internal rigid + joint
  model, and can round-trip back out to URDF. Sample models:
  `test_data/simple_arm.urdf` (3-joint arm), `turtlebot3_burger.urdf`.
- **ROS 2 bridge** (`extensions/ros2_bridge/`, `pymycobot` target): an
  *offline-coupled* sim-to-real loop — record a real arm's joint trajectory,
  replay it in the sim, and diff sim against measured. Pluggable end-effector
  **force estimators** (kinematic-only, motor-current, external wrench) selected
  by YAML (`config/mycobot_280_*.yaml`, `mycobot_320_*`, `mycobot_pro600_*`).
  Reference hardware: Elephant Robotics myCobot; adaptable to UR / Franka.
  Runtime-coupled (closed-loop) mode is out of scope for Phase 1.

Build and exercise the URDF path (CPU only):

```powershell
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release --target test_urdf_physics_bridge
./build/Release/test_urdf_physics_bridge.exe   # import simple_arm.urdf → build + step
```

The ROS 2 bridge is a standard `ament_python` package — see
[`extensions/ros2_bridge/README.md`](extensions/ros2_bridge/README.md) for the
node layout, `colcon` build, and the record → replay → diff workflow.

## Headline results (Pivot 4, 2026-05-15)

**Constitutive calibration.** Repose angle 35.06° at H/D = 0.71 (within
literature 30–35° band for dry sand). Lube 2004 H/D-dependence reproduced
across H/D ∈ [0.5, 3.0] at fixed α = 0.55.

**Coupling ablation on `box_drop` (1 kg box, sand patch, 3 s @ 240 Hz):**

| Method | max\|Σp_y\| | vs M1 | step μs | dwell % |
|---|---|---|---|---|
| M0 Dirichlet | 6.256 | 35× worse | 79.8 | 71 |
| M1 PIC-Rigid | 0.179 | 1.00× | 85.0 | 88 |
| **M4.5a** (Layer A only) | **0.082** | **0.46×** | 85.0 | **52** |
| M4.5b (A + friction) | 0.082 | 0.46× | 86.0 | 54 |
| M4.5c (full hybrid) | 25.28 | 141× worse | 78.7 | 15 |

**M4.5a reduces M1's peak momentum-conservation error by 2.2× at parity
step cost.** The same 0.46× ratio holds under 9× body-mass scaling
(`box_drop_dense`, Finding 1b mass-invariance).

**Pivot-4 meta-finding** (§6.6 of paper): two of five ablation findings
*reverse direction* when the same comparison is re-run under stub-stress
vs validated-stress continuum. Under real sand, M4.5b becomes
*indistinguishable from M4.5a* on box_drop and *regresses* on rover_wheel
(slip 1.17 → 1.40, sinkage 0.000 → 0.172 m). The Round-15 conclusion
that "Layer B's friction is necessary" was an artefact of the missing
constitutive stress force; with validated sand it is *redundant or
harmful*. This is a methodological caution for coupling-ablation studies.

## Layout

```
.
├── include/, src/, tests/             ← main engine (Phase 0; frozen except critical fixes)
├── apps/editor/                       ← ImGui editor / demo viewer (build from source)
├── extensions/ros2_bridge/            ← ROS 2 sim-to-real bridge (myCobot)
├── test_data/                         ← sample URDF robots (simple_arm, turtlebot3)
├── benchmarks/                        ← legacy drift benches (still useful)
├── docs/                              ← project documentation, decisions, paper outline
│   ├── PROJECT_STATUS.md
│   ├── FORK_NOTE.md
│   ├── roadmap.md, checklist.md
│   ├── paper_outline.md
│   ├── multi-scale_engine.md, api_reference.md
│   └── notes/                         ← element_tech / references / integration / risk / limitations
├── research/                          ← independent sub-projects
│   └── mpm_rigid_coupling/            ← Paper #1 benchmark suite (M0 ... M4.5)
│       ├── README.md, THEORY.md
│       ├── couplers/, scenarios/, scripts/, outputs/
│       └── bench_compare_all.cpp
└── outputs/figures/                   ← main-engine drift figures (separate from research outputs)
```

## Testing

```
build/Release/test_mpm_stress_isolation.exe      # MLS-MPM stress force
build/Release/test_mpm_constitutive.exe          # Young / bulk / D-P projection
build/Release/test_mpm_rigid_sweep.exe           # M4.5 sweep-template invariants
build/Release/test_mpm_rigid_cylinder.exe        # cylinder primitive
build/Release/test_mpm_rigid_coupler.exe         # M0 Dirichlet
build/Release/test_mpm_cpu.exe                   # MPM smoke test
# ... 110+ tests in total; see CMakeLists.txt
```

Re-validate paper findings programmatically:
```
python research/mpm_rigid_coupling/scripts/reproduce_findings.py
```

## Citing

If you use Basements in academic work, please cite both the software
and the paper. See [`CITATION.cff`](CITATION.cff). The paper is in
preparation; the citation entry will be updated with arXiv DOI on
submission.

## License

- **Code:** MIT — see [`LICENSE`](LICENSE)
- **Data** (validation submissions, output CSVs): CC-BY-4.0

## Status

Work in progress. Codebase is a single-engineer research project; APIs may
change at phase boundaries. See [`docs/PROJECT_STATUS.md::Pivot Record`](docs/PROJECT_STATUS.md)
for intentional scope changes. Five-paper series outlined in
[`docs/roadmap.md`](docs/roadmap.md); paper #1 in preparation, target
submission Q3 2026.

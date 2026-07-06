# FORK NOTE — Main Engine vs Research Sub-project

**Created:** 2026-05-13 (Round 8).
**Status (2026-05-16):** **fork distinction superseded by Pivot 6 "growing
platform" framing.** Track A (main engine `include/`) and Track B
(`research/mpm_rigid_coupling/`) now share a single evolving forward sim
under the in-place autodiff decision (see
[`PROJECT_STATUS.md::Pivot 6`](./PROJECT_STATUS.md) and
[`notes/pivot6_differentiable_mpm.md`](./notes/pivot6_differentiable_mpm.md)
§4). Paper #2 / #3 / #4 all extend the same `MPMSolver` class.

The boundary map below is retained as **historical record of the Phase 1
fork rationale**, not as an active separation. Practically:

- `include/basements/physics/mpm/` is the shared MPM core (was Track A).
- `research/mpm_rigid_coupling/` houses scenarios + bench dispatch
  (was Track B); coupler headers
  (`couplers/*.h`) live there but **integrate with the shared core**
  rather than parallel-implementing.
- `extensions/ros2_bridge/`, `extensions/basements_py/` (in progress)
  add capability layers on the same core, not parallel tracks.

No new code should treat the Track A / Track B distinction as load-bearing.

This document is the **canonical map** of what work belongs to which side of
the fork, why the fork exists, and when (if ever) to merge back. Read this
before starting a new task — pick the right side before touching code.

---

## Why the fork

The original "Basements multi-scale physics engine" scope (Bullet/MuJoCo
alternative, "Earthworms to Galaxies", CUDA + Metal + ROCm backends) is not
single-engineer-feasible on any reasonable timeline. Round 7 retrospective made
this explicit. Round 8 splits the work into two **independently shippable**
tracks so that paper publication (highest ROI) does not block on UI / GPU
work, and vice versa.

---

## Boundary map

```
G:/CPP/Basements/
├── include/                 ── MAIN ENGINE — frozen except for bug fixes ──┐
├── src/                                                                    │
├── tests/                                                                  │  Track A
├── benchmarks/                                                             │
├── apps/  examples/                                                        │
├── assets/                                                                 │
├── basements_editor_portable/                                              │
├── docs/                                                                   │
│   ├── multi-scale_engine.md  (v12, post-pivot)                            │
│   ├── PROJECT_STATUS.md      (overview both tracks)                       │
│   ├── roadmap.md             (phase plan both tracks)                     │
│   ├── checklist.md                                                        │
│   ├── paper_outline.md                                                    │
│   ├── api_reference.md                                                    │
│   ├── notes/                                                              │
│   └── FORK_NOTE.md          ← this file                                   │
│                                                                           │
└── research/                  ── PAPER #1 BENCHMARK SUITE ──────────────┐ │
    └── mpm_rigid_coupling/                                              │ │
        ├── README.md                                                    │ │  Track B
        ├── CMakeLists.txt                                               │ │
        ├── couplers/{m0..m4_5}.h                                        │ │
        ├── scenarios/scenario_*.h                                       │ │
        ├── bench_compare_all.cpp                                        │ │
        ├── frame_exporter.h                                             │ │
        ├── scripts/{plot_compare, render_animation}.py                  │ │
        └── outputs/{csv, figures, videos}/                              │ │
                                                                         ┘ ┘
```

### Rule of thumb

| If you are about to … | Which side? |
|---|---|
| Add or fix a coupling algorithm (M0..M4.5) | **research/mpm_rigid_coupling/** |
| Add a new scenario (rover wheel, foot, etc.) | **research/mpm_rigid_coupling/scenarios/** |
| Tune visualisation, generate figures or videos | **research/mpm_rigid_coupling/scripts/** |
| Fix a math bug in Vec3 / Quaternion / Matrix3 | main engine (`include/basements/core/math/`) |
| Fix a bug in `MPMSolver` itself | main engine (used by both sides) |
| Edit `RigidColliderState` schema | main engine, but it propagates to research/ couplers — coordinate |
| Editor UI work | main engine (and only when crash log arrives — currently parked) |
| GPU CUDA backend | main engine, Phase 3 — parked |
| Paper LaTeX writing | new `paper/` directory at repo root (does not exist yet) |

### Hard rules

1. **Research code never modifies main-engine headers.** If a coupler needs
   a new engine API, the engine change goes to a separate PR first, then the
   research code uses it.
2. **Main engine stays buildable + 84/84 tests green** at every commit, even
   while research code is in flux.
3. **Research code can be in arbitrary state** (skeletons, stubs, TODOs)
   as long as `bench_compare_all` still builds. The Paper #1 figures are
   produced from CSVs, so partial-stub couplers are acceptable — they just
   produce duplicate plots until upgraded.
4. **No cross-dependency the other way**: `research/` may include main-engine
   headers; main engine may NEVER include `research/` headers.

---

## What lives on each side, concretely

### Track A — Main engine (frozen except critical fixes)

- All of `include/basements/`, `src/`, `tests/`, `benchmarks/`.
- `basements_editor` executable + portable bundle. **UI work parked**
  pending crash log from user; only crash fixes allowed.
- `basements_coupling` static library (`MPMRigidCoupler`, `MPMRigidCouplerPIC`)
  — research/ aliases these as M0 / M1.
- Math, MPM solver, rigid solver, joints, URDF — all stable.
- Drift benchmark (`bench_energy_drift`, `bench_mpm_rigid_drift`) —
  superseded by `bench_compare_all` for paper purposes, but kept as legacy
  validation for the integrator/solver layers.

**Activity rule:** anything other than (a) bug fix, (b) crash-log driven
Editor fix, or (c) engine API needed by Track B requires a PROJECT_STATUS
"Active Decisions" update first.

### Track B — `research/mpm_rigid_coupling/` (where active work happens)

- M0 / M1 are re-exports — no duplicated code.
- M2 ASFLIP, M3 MLS-MPM, M4 Lagrange, M4.5 Hybrid — currently stubs that
  delegate to M1; each will be replaced by its own kernel as Phase 1.1 → 2
  progresses. Output figures use whichever kernel is wired at build time.
- All scenarios use the same canonical sand patch in
  `scenarios/scenario_box_drop.h`. New scenarios (rover_wheel, foot_step,
  static_pile) extend this directory.
- Visualisation is Python-only — no C++ side renders. Output `.png` for
  paper figures, `.mp4` (or `.gif` if no ffmpeg) for supplementary video.

**Activity rule:** all paper-track work happens here. Roadmap Phase 1 lives
entirely under `research/mpm_rigid_coupling/`.

---

## When to merge back (or not)

| Trigger | Action |
|---|---|
| Paper #1 accepted | Tag commit, keep research/ for reviewer reproduction; no merge |
| M2/M3/M4/M4.5 kernels stable + would benefit other engine users | Promote individual coupler to `basements_coupling` (Track A), keep research/ as test harness |
| Main engine resumes (Phase 3 GPU work) | Independent; no overlap — research/ is CPU-only |
| Research scope expands (new modality, e.g. SPH-rigid) | New `research/<topic>/` peer to `mpm_rigid_coupling` — keep them disjoint |
| Engine team finds the research/ benchmark useful for regression | Add a CI job that runs `bench_compare_all --all` weekly (already in roadmap as Phase 3.x) |

**Default assumption: never merge.** The two tracks have different audiences
(paper reviewers vs general engine users) and different rates of change.
Keeping them disjoint is the lower-cost equilibrium.

---

## Future-proofing checklist for the fork

- [x] Sub-project builds as its own target (`bench_compare_all`).
- [x] Sub-project does not modify any engine header.
- [x] Outputs go to `research/mpm_rigid_coupling/outputs/` — separate from
      main-engine `outputs/figures/`.
- [x] CMake option `BUILD_RESEARCH_MPM_RIGID` (default ON) lets a clean
      release build of the engine skip research/ entirely.
- [x] All 6 method IDs (M0..M4.5) declared in
      `couplers/{m0_dirichlet,m1_pic_rigid,m2_asflip,m3_mls_mpm,m4_lagrange,m4_5_hybrid}.h`.
- [x] Single-entry benchmark exe with `--method` / `--scenario` / `--all`.
- [x] Comparison figure auto-generated from CSVs.
- [x] Animation script with ffmpeg + Pillow fallback.
- [ ] Add `rover_wheel`, `foot_step`, `static_pile` scenarios (Phase 1.1).
- [ ] Replace M2..M4.5 stubs with real kernels (Phase 1.1 → 2).
- [ ] Lock canonical sand parameters once Klar 2016 set is consolidated
      into `material_presets.h` (engine side).

---

## How to navigate "what changed on which side?" later

When future-you (or a collaborator) needs to know the history:

1. **Start here** — this FORK_NOTE.md tells you which directory matters.
2. **PROJECT_STATUS.md "Pivot Record" section** — chronological pivots.
3. **roadmap.md** — phase-level intent.
4. **memory/project_basements.md** — round-by-round narrative (this is the
   most fine-grained log; if you want "why did we do X in Round 7?", look here).
5. **notes/integration_notes.md** — cross-system gotchas that survive across
   pivots.

Git history is the ultimate source of truth, but the documents above keep
the *intent* aligned. Git tells you what changed; these docs tell you why.

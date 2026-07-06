# Basements — Phase Checklist

**Document Version:** 2.0 (Pivot 6 extension; Phase 2 layered onto Phase 1 record).
**Last Updated:** 2026-05-16.
**Scope:** Phase 1 (paper #1 — *dropped at Pivot 6, items retained as
historical record*) + Phase 2 (paper #2 — Differentiable MPM-Rigid
Coupling for Agricultural Sim-to-Real, currently active).
See [`roadmap.md`](./roadmap.md) for the full multi-phase plan and
[`PROJECT_STATUS.md::Pivot 6`](./PROJECT_STATUS.md) for the audit
that drove the pivot.

---

## Phase 2 — Paper #2 milestones (active, Pivot 6)

Architectural commitment: **in-place autodiff** (no separate
extensions/diff_mpm/ codebase; opt-in `step(dt, tape*)` overload).
Bit-exact regression `test_mpm_autodiff_tape_neutral` enforces
forward-only path equivalence in CI on every commit.

### M1 — pybind11 binding + AutodiffTape skeleton
- [x] `include/.../mpm_autodiff_tape.h` — TapeEntry / AutodiffTape (M1 part 1/3, ✅ task #145)
- [x] `MPMSolver::step(dt, AutodiffTape*)` opt-in overload (M1 part 1/3, ✅ task #145)
- [x] `test_mpm_autodiff_tape_neutral` — bit-exact regression 2/2 PASS (M1 part 1/3, ✅ task #145)
- [ ] `extensions/basements_py/` directory + CMake target (M1 part 2/3, task #136)
- [ ] pybind11 forward-sim binding + smoke test (M1 part 2/3, task #136)
- [ ] PyTorch `autograd.Function` wrapper module (M1 part 3/3, task #136)

### M2 — Reverse-mode autodiff
- [ ] DiffTaichi-pattern tape recording inside p2g_single (task #137)
- [ ] update_grid recording (task #137)
- [ ] g2p_single recording (task #137)
- [ ] `test_diff_mpm` — finite-difference gradient check on simple compression (task #137)

### M3 — Plastic projection adjoint
- [ ] SVD gradient (Papadopoulo 2000) — analytical (task #138)
- [ ] Log-strain projection chain — analytical (task #138)
- [ ] Cone-surface subgradient handling — measure-zero documented (task #138)
- [ ] `test_diff_plastic` — gradient check at yield surface (task #138)

### M4 — M4.5a non-local coupling gradient
- [ ] Two-pass sweep backward in-place in `couplers/m4_5_hybrid.h` (task #139)
- [ ] Non-local Σ frac dependency tracking (task #139)
- [ ] `test_diff_coupling` — gradient check on tine_drag (task #139)
- [ ] **Pre-registration:** FD step sizes + N sweep + < 1% L₂ pass criterion
      deposited on Zenodo *before* the verification figure is rendered
      (paper §3.4 ground truth, see [`notes/pivot6_differentiable_mpm.md`](./notes/pivot6_differentiable_mpm.md) §6).

### M5 — Multi-phase constitutive (C3 fold)
- [ ] Capillary pressure term added to stress force (task #140)
- [ ] Cohesion in D-P yield projection (task #140)
- [ ] Forward validation against wet-sand literature (task #140)

### M6 — RL training infrastructure
- [ ] Policy network MLP (task #141)
- [ ] PPO / SAC training loop on tine_drag (task #141)
- [ ] Reward design: minimise draft force at constant depth (task #141)

### M7 — Sim-to-real on myCobot
- [ ] ArUco driver verification (task #142, hardware day 1)
- [ ] Tine_drag hardware setup (probe + soil container) (task #142, hardware day 2)
- [ ] Trained-policy deployment + gap measurement (task #142, hardware day 3)

### M8-M9 — Paper draft + submission
- [ ] LaTeX §3.2-§3.4 (adjoint derivations) (task #143)
- [ ] §4.2-§4.5 (gradient + policy + sim-to-real) (task #143)
- [ ] §6 (limitations, Pivot-4 → domain-randomization repackage) (task #143)
- [ ] arXiv preprint (cs.RO primary + cs.LG secondary)
- [ ] ICRA / CoRL / NeurIPS / IROS submission

## User action gates (Phase 2)

Persist regardless of doc cleanups:

- [ ] Zenodo account + GitHub repo link (30 min one-off, task #123 setup
      doc ready). Required before first pre-registration deposit.
- [ ] myCobot model + F/T sensor decision (M7 trigger; design doc
      `notes/l3_pipeline_design.md` supports all 4 model × sensor combos).
- [ ] Optional: agricultural soil container (modified for tine_drag).

---

## Phase 1 — Paper #1 record (historical, completed or rescoped at Pivot 6)

Format: each item is a *single sittable task* — under 4 hours focused work.
Daily standup style — open this file before starting, check off when done,
move "next" items up the stack.

---

## ⚠ Submission gate (from `notes/limitations.md` + Round 16 audit)

Phase 1.2 (paper draft) does NOT start until:

- [ ] `rover_wheel` scenario produces sinkage within 2× of Bekker's
      prediction for our sand parameters.
- [ ] `foot_step` produces a rebound coefficient in [0, 1].
- [ ] Section 6 (Limitations) drafted — see `notes/limitations.md`. ✅
- [ ] Python module / Taichi interop 1-page design note in
      `notes/element_tech.md`.

### Reproducibility gate (Round 16)

A single audit (Round 15) revealed two latent bugs in M4.5b/c. Going
forward, *every* paper finding must be reproduced under varied
conditions before it can carry weight. Concretely:

- [ ] Each finding (1–5 in `paper_outline.md`) verified under three
      distinct `dt` values: `1/60`, `1/120`, `1/240`. Same qualitative
      ranking, ratios within 2× across rates.
- [ ] Each finding verified under three distinct *initial body
      positions* (offset by ±¼ cell each axis). Ranking unchanged.
- [ ] Each finding survives a Release build without -ffast-math and
      a Debug build with assertions on. (Catches optimisation-only
      regressions.)
- [ ] `test_mpm_rigid_sweep` and `test_mpm_rigid_cylinder` green; the
      sweep template contract is the load-bearing scaffolding under
      every measurement.

Without these the paper is one reviewer-level audit away from retraction.

Without these the paper is synthetic-only and reject-risk.

## 30-day window — Robotics validation scenarios (Phase 1.1)

### Code

- [ ] Add `cylinder` primitive to `RigidColliderState` so the wheel scenario
      can use real cylindrical geometry, not an OBB approximation.
  - `include/basements/physics/coupling/mpm_rigid_coupler.h`: add
    `enum class ShapeKind { BOX, CYLINDER };` + `axis` + `radius` + `length`.
  - Update both `MPMRigidCoupler::apply` and `MPMRigidCouplerPIC::apply`
    inside-check to dispatch on shape kind.
  - Unit test: cylinder probe vs OBB probe equal when radius == half_extents.
- [ ] `benchmarks/bench_rover_wheel.cpp` — release a horizontal cylinder
      (radius 0.1 m, length 0.2 m) onto a 0.3 m deep sand bed, apply forward
      torque via `apply_torque`, record (sinkage, drawbar, slip) over 3 s.
- [ ] `benchmarks/bench_foot_step.cpp` — 0.1 m × 0.05 m × 0.1 m foot drops
      onto a 0.1 m sand layer, measure rebound height + dissipated energy.
- [ ] `scripts/plot_rover_wheel.py` (and same for foot_step) — paper-ready
      figure with three subplots (kinematics, force, energy).

### Decisions to lock down

- [ ] Choose **canonical sand parameters** (cohesion, internal friction,
      density) and document in `notes/element_tech.md` so all benchmarks
      use the same reference granular medium. Cite Stomakhin 2013 vs
      Klar 2016 for the chosen parameter set.
- [ ] Pick **canonical OBB grid resolution** (current `dx = 0.1`) and
      stick with it across all paper figures so cross-scenario comparison
      is apples-to-apples.

### Documentation

- [x] `docs/roadmap.md`
- [x] `docs/checklist.md` (updated each round)
- [x] `docs/PROJECT_STATUS.md`
- [x] `docs/FORK_NOTE.md`
- [x] `docs/notes/limitations.md`
- [x] `research/mpm_rigid_coupling/THEORY.md`
- [x] `notes/element_tech.md` — sand parameters + cite source (initial)
- [x] `notes/references.md` — Stomakhin / Klar / Fei / Hu / Jiang
- [x] Update `multi-scale_engine.md` Quick Status table
- [ ] Lock canonical sand parameters into `material_presets.h` (engine-side header)

---

## 60-day window — Paper draft (Phase 1.2)

### Paper

- [ ] `paper/main.tex` — EG short paper class (`\documentclass{egpubl}`).
- [ ] Section 1 (Intro) — 1 page, draft from existing `docs/paper_outline.md`.
- [ ] Section 2 (Related Work) — 0.5 page; key citations in
      `notes/references.md` already.
- [ ] Section 3 (Method) — 1 page; pseudocode block for M0, M1.
      ASFLIP/M2 acknowledged as future work.
- [ ] Section 4 (Experiments) — 1 page; 4 reference figures
      (`mpm_rigid_drift{,_pic}.png`, `bench_rover_wheel.png`,
      `bench_foot_step.png`).
- [ ] Section 5 (Discussion / Limitations) — 0.5 page.
- [ ] References (`.bib`) — only items physically cited in text.

### Figures (must be regeneratable by single command)

- [ ] All paper figures emit from `cmake --build build --target drift_bench_long_form`.
- [ ] Add Phase 1.1 benches into the custom target.
- [ ] Lock figure DPI (150) and aspect (8×4.5) so paper layout is stable.

### Internal review

- [ ] Send draft to **one** trusted reader. Their job: kill claims that
      aren't backed by figures. (1 reader > 0 reviewers; > 3 readers loses
      coherence in a 1-author paper.)

---

## 90-day window — Submission (Phase 1.3)

### Pre-submission

- [ ] arXiv upload — `cs.GR` primary, `cs.RO` secondary cross-list.
- [ ] Pick venue:
      - EG Short (deadline historically Dec for next year's spring conf).
      - SCA short (deadline historically Apr for Aug conf).
      - SIGGRAPH Asia Posters (deadline historically Aug).
- [ ] Submit.

### Post-submission, while waiting for reviews

Start Phase 2 (M2 ASFLIP) so paper revision time isn't dead time.

- [ ] Read Fei 2021 ASFLIP paper end-to-end. Notes into
      `notes/element_tech.md::asflip`.
- [ ] Prototype F-bar split unit test (no rigid integration yet).

---

## Stretch / parking lot (not on the critical path)

These items have been flagged across rounds but are NOT blocking Phase 1.
Logged here so they don't get lost.

- [ ] Editor crash reproducer from user (waiting on `editor_log.txt`).
- [ ] M4.5b friction-tangent regression analysis (apply before vs after
      mass-average) — pick the variant that does not regress.
- [ ] M4.5c kernel regression fix — third pass for re-normalisation
      after kernel weighting, or replace with sweep-internal kernel pass.
- [ ] Dwell-time threshold → dimensionless (half-body-extent based).
- [ ] M2 ASFLIP real impl (Fei 2021 F-bar split + history per particle).
- [ ] M3 MLS-MPM real impl (Hu 2018 kernel + APIC C_b for rigid).
- [ ] M4 Lagrange-multiplier real impl (Schur complement + PCG).
- [ ] **ReboundTracker → multi-peak version** for Paper #2 calibration.
      Current tracker only records first sign-change; hyperelastic sand
      will produce multi-bounce signatures (e₁, e₂, e₃, ...) that
      become a distinguishing axis between couplers under recovery.
- [ ] **Scenario time normalisation** for supplementary video.
      4 scenarios run for different durations (box_drop 3 s,
      rover_wheel 3 s, foot_step 2 s, dense 3 s); side-by-side video
      either trims to `min(T) = 2 s` or time-normalises each clip to
      `t / T_scenario`. Defer until video editing phase.
- [ ] SceneNode → ConstraintHandle map already done (Round 7 B-2).
- [ ] Multi-block sparse grid optimisation (Phase 3).
- [ ] Reduce per-frame MPM-Rigid coupling cost via incremental block tracking.
- [ ] GitHub Actions: actually push the repo + activate `drift_bench.yml`.
- [ ] `set_joint_velocity` smooth interpolation for ROS 2 input streams.
- [ ] Inspector: tree-view sub-grouping of JOINT nodes.

---

## Definition of done (this checklist)

A line is **done** when:
1. Code committed and 84/84 regression still green.
2. If documentation changed, registered in PROJECT_STATUS.md.
3. If figure changed, regenerated via the `cmake --build` target.

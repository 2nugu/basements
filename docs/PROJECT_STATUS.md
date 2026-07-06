# PROJECT STATUS

## Overview

**Basements** — Open MPM ↔ Rigid coupling benchmark suite for robotics in
granular media. Single-engineer, part-time, multi-year horizon. Niche scoping
deliberate — see `notes/integration_notes.md` and `roadmap.md` for why this
is not a Bullet/MuJoCo alternative.

## Public release & pause checkpoint (2026-07-06)

**Status: PUBLISHED + PAUSED (resume-ready).** Public at
https://github.com/2nugu/basements (MIT code / CC-BY-4.0 data); archived on
Zenodo — DOI [10.5281/zenodo.21215512](https://doi.org/10.5281/zenodo.21215512).

Done this session: `git init` + public release, with build artifacts (`build/`,
`bin/`, `lib/` = gtest/gmock, the prebuilt editor bundle) and the **unpublished
`paper/` manuscript** excluded via `.gitignore`; documented the URDF robot-arm +
ROS 2 myCobot sim-to-real bridge in the README; **clean-clone build verified
GREEN on Windows** (all FetchContent deps resolve; URDF parser 11/11 +
physics_bridge 11/11 pass; `bench_compare_all` builds — with a documented Windows
long-path caveat); added [`../ROADMAP.md`](../ROADMAP.md) (status + un-softened
open problems L-1..L-5 + where-to-help) and **8 labelled contributor issues**
(#1–#8: good-first-issue / help-wanted / open-problem).

**`paper/` is intentionally NOT public** (prior-disclosure risk for future venue
submission); it remains in the local tree, gitignored.

**Resume here (when reopening):**
- Traffic/discoverability (owner action): call-for-contributors post, arXiv / PwC.
- Decide the contributor-contact channel (Discussions / email public).
- Cross-verify issue file pointers resolve; fix the ROS 2 sub-README's dead
  `sim_node.py` reference; reconcile CITATION `date-released` with the release date.
- Next code step (roadmap Milestone 1.1): cylinder collider primitive → the
  `rover_wheel` + `foot_step` validation scenarios (unblocks the paper).

## Current Phase

**Phase 2: Paper #2 — Differentiable MPM-Rigid Coupling for
Agricultural Sim-to-Real** — Starting (Pivot 6, 2026-05-15).

**Carry-forward from Phase 1 (paper #1 dropped):** all forward-sim
code, validation tests, scenarios (box_drop, box_drop_dense,
rover_wheel, foot_step, tine_drag, repose_pile), Klar+Hu+M4.5a
calibrated continuum, L3 ros2_bridge scaffold, validation /
external_submissions protocol, Lube H/D sweep, ArUco verification
plan — all stay. **None of the engineering work is wasted.** They
form paper #2 §3.1 forward sim + Appendix A.

**Phase 2 deliverables:**
1. pybind11 Python binding (MPMSolver → PyTorch tensor I/O)
2. Backward gradient flow through MLS-MPM step (DiffTaichi pattern)
3. Adjoint method for Hencky-Drucker-Prager plastic projection
4. Differentiable M4.5a two-pass sweep
5. Multi-phase constitutive variants (dry / moist / cohesive)
   as domain randomization axis
6. Policy learning on `tine_drag` agricultural scenario
7. Sim-to-real validation on myCobot
8. Paper draft + ICRA / CoRL / NeurIPS submission

**Gating decisions:** finite-difference gradient correctness on
M4.5a forward sim before policy training infra is built; pybind11
binding before any backward-pass work.

## Phase Checklist

See [`checklist.md`](./checklist.md) for the daily 30/60/90-day breakdown.
High-level:

**Phase 0 + Phase 1 (paper #1 dropped at Pivot 6; record preserved):**

- [x] Phase 0: Foundation (math + MPM + rigid + editor) — done.
- [x] M0 Dirichlet coupling, M1 PIC-Rigid (35× momentum-error improvement).
- [x] M4.5 layered ablation (a/b/c) — Layer A is current headline (2.2× ↓).
- [x] Quantitative metrics framework (max/avg \|Σp\|, ΔE/E₀, dwell, vs-M0).
- [x] 4-panel comparison figure with broken y-axis + Pareto.
- [x] `THEORY.md` derivation; `limitations.md` adoption + validation paths.
- [x] Phase 1.1b: `rover_wheel` cylinder scenario + Bekker scaling check (full calibration deferred to paper #2 M5).
- [x] Phase 1.1c: `foot_step` scenario + rebound coefficient measurement (e=0 documented).
- [x] Pivot 4: Hencky stress + Klar D-P projection corrected.
- [x] Pivot 5: Agricultural robotics identity reframing.
- [x] Pivot 6: Paper #1 dropped; series re-ordered (paper #2 = differentiable).
- [x] tine_drag agricultural scenario operational (4 methods measured).
- [x] LaTeX paper #1 draft (archived, becomes paper #2 §3.1 + Appendix A).
- ~~Phase 1.2: Paper #1 draft~~ → archived (Pivot 6).
- ~~Phase 1.3: arXiv preprint paper #1~~ → dropped (Pivot 6).

**Phase 2 (paper #2 — Differentiable MPM-Rigid, active):**

- [x] In-place autodiff architecture decision lock-in (firewall rejected).
- [x] AutodiffTape skeleton + `step(dt, tape*)` overload.
- [x] `test_mpm_autodiff_tape_neutral` bit-exact regression (2/2 PASS).
- [ ] M1: pybind11 binding + PyTorch wrapper (task #136).
- [ ] M2: Reverse-mode autodiff (DiffTaichi pattern) (task #137).
- [ ] M3: Plastic projection adjoint (Papadopoulo + Huang 2022) (task #138).
- [ ] M4: M4.5a non-local coupling gradient + FD ground truth (task #139).
- [ ] M5: Multi-phase constitutive (capillary + cohesion) (task #140).
- [ ] M6: RL training infrastructure on tine_drag (task #141).
- [ ] M7: Sim-to-real on myCobot (ArUco + tine_drag hardware) (task #142).
- [ ] M8-9: Paper draft + ICRA / CoRL / NeurIPS submission (task #143).

**Phase 3 (paper #3 — GPU + adaptive real-time, after paper #2):**

- [ ] GPU CUDA backend for forward + backward MLS-MPM.
- [ ] Adaptive-dx coupling (coarse background + fine near rigid body).
- [ ] Ghost-node stability characterisation across coarse:fine ratios.
- [ ] Real-time RL training demo on tine_drag.

**Phase 4 / 5 (paper #4 — field deployment, optional):**

- [ ] Korean field site partnership identified.
- [ ] Full-scale agricultural robot platform (beyond myCobot).
- [ ] Closed-loop deployment of paper #2-trained policy.

## Completed Milestones

| Milestone | Output | Date |
|-----------|--------|------|
| MPM B-spline weight fix | avg_y stable | 2026-03-21 |
| Sequential Impulse solver (4 critical bugs fixed) | `test_physics_validation 6/6` | 2026-03-27 |
| Joint system (BallSocket/Hinge/Slider/Fixed + limits) | `test_joints 5/5`, `test_joint_limits 7/7` | 2026-03-30 |
| Contact caching (feature key + hysteresis + seconds age) | `test_contact_cache 13/13` | 2026-05-13 |
| URDF Physics Bridge | `test_urdf_physics_bridge 11/11` | 2026-05-13 |
| MPM ↔ Rigid Dirichlet (M0) | `test_mpm_rigid_coupler 4/4` | 2026-05-13 |
| **MPM ↔ Rigid PIC (M1) — 35× momentum-error improvement** | `mpm_rigid_drift_pic.png` | 2026-05-13 |
| **M4.5a (renormalized adaptive mass) — 2.2× ↓ vs M1, 33% less dwell** | `research/mpm_rigid_coupling/outputs/figures/compare_box_drop.png` | 2026-05-13 |
| Quantitative metrics framework (5 axes: max/avg \|Σp\|, \|ΔE\|, dwell, vs-M0) | `metrics.h` + per-method `_metrics.csv` | 2026-05-13 |
| **MLS-MPM Hencky-Kirchhoff stress force loop (closes long-standing stub)** | `test_mpm_stress_isolation 2/2`, `test_mpm_constitutive 3/3` | 2026-05-15 |
| **Separable floor BC + Coulomb friction** | `mpm_solver.h::update_grid` | 2026-05-15 |
| **Drucker-Prager projection corrected to Klar 2016 eq. 3.1 form** | `(3λ+2μ)/(2μ)` kohesion factor restored | 2026-05-15 |
| **Repose pile case study added** (angle_geom 22.97° vs target 30–35°) | `scenario_repose_pile.h` + `outputs/csv/repose_pile_summary.csv` | 2026-05-15 |
| **W2 Klar calibration — repose angle_geom 35.06° ∈ [30°, 35°]** | dx=25mm + α=0.55 + dt=1/960; W2 task #111 | 2026-05-15 |
| **W3 ros2_bridge scaffold — Python ament package, 3 force estimator strategies + 6 YAMLs** | extensions/ros2_bridge/; smoke test PASS | 2026-05-15 |
| **5-paper research series outline** in roadmap.md — defensive scope per paper | docs/roadmap.md; Pivot 4 long-horizon plan | 2026-05-15 |
| **Pivot 5 agricultural robotics reframing** — paper identity + venue + audience | docs/PROJECT_STATUS.md Pivot 5 + paper/main.tex title | 2026-05-15 |
| **tine_drag agricultural scenario operational** — 4 methods measured | research/.../scenarios/scenario_tine_drag.h + bench run_tine_drag | 2026-05-15 |
| **Public repo infrastructure** — LICENSE, CITATION.cff, CONTRIBUTING.md, AUTHORS.md, CI workflow | repo root + .github/workflows/ci.yml | 2026-05-15 |
| **LaTeX paper draft** — main.tex acmart + 8 sections + 17 BibTeX | paper/ | 2026-05-15 |
| **Pivot 6 — paper #1 dropped, paper #2 = Differentiable MPM-Rigid for ag-sim-to-real** | docs/notes/pivot6_differentiable_mpm.md + roadmap.md re-ordered 3-paper series | 2026-05-15 |
| Two-pass normalised sweep template (sweep<NodeUpdate>) | `couplers/m4_5_hybrid.h` | 2026-05-13 |
| Editor multi-slot snapshot + JSON I/O + label modal | 84/84 regression green | 2026-05-13 |
| Drift figure pipeline (6 figures) | `outputs/figures/*.png` | 2026-05-13 |
| CI workflow (drift_bench weekly) | `.github/workflows/drift_bench.yml` | 2026-05-13 |

## Active Decisions & Rationale

- **Vision scoped from "Earthworms to Galaxies" to MPM-Rigid for robotics.**
  Reason: scope was unbounded; current niche has a publishable contribution
  ready and a clear audience (robotics + graphics SCA reviewers). See
  `roadmap.md` reframe section.
- **Paper #1 horizon = 3 calendar months.** Reason: Taichi group is the
  likely scoop risk; arXiv preprint locks priority before peer review.
- **Paper #1 hero claim now M4.5a (Layer A only), not M1.** Reason: Round 10
  showed M4.5a achieves 2.2× improvement over M1 with 6% extra cost. M4.5b/c
  documented as honest regressions — strengthen the ablation narrative
  rather than the headline.
- **Standalone Basements engine framing is dropped.** Reason: `limitations.md`
  L-1 analysis — no production engine for this niche exists, building a
  standalone competitor is shipping into a vacuum. Phase 4 reframed as
  **Python module + Taichi interop + ROS 2 plugin**.
- **Phase 1.2 paper draft is gated.** Reason: `limitations.md` self-test —
  no paper draft starts until `rover_wheel` (Bekker baseline) and
  `foot_step` (rebound coefficient) scenarios produce finite, comparable
  numbers.
- **CPU-only path is the reference; GPU is Phase 3.** Paper figures all CPU.
- **Editor demoted to demo viewer.** No further general-purpose editor work.
- **Two-pass normalised sweep is the canonical pattern** for any future
  hybrid coupler (M4.5x, etc.). Single-pass with naive frac weighting is
  documented in `THEORY.md` as the failure mode.

## Open Questions

- [x] User track (academic / product / personal). **Resolved Pivot 5:**
      academic, with agricultural robotics (KNU Smart Agriculture /
      Biosystems Engineering) as identity anchor. Primary venues:
      Biosystems Engineering (Elsevier) / Computers and Electronics in
      Agriculture.
- [ ] Editor crash reproducer waiting for `editor_log.txt` from user.
      Until received, fix scope is speculative — see `risk_log.md::R-2`.
      *Lower-priority post-pivot:* editor demoted to demo viewer.
- [x] Canonical sand parameters. **Resolved Pivot 4 + Pivot 5:**
      Jumunjin (KS F 2308) primary, Ottawa F-65 international reference.
      α = 0.55 W2-calibrated to repose 35°. Configs in
      `extensions/ros2_bridge/config/sandbox_*.yaml`.
- [x] MPM constitutive model is currently F-only (no hyperelastic energy).
      **Resolved Pivot 4:** Hencky stress + MLS-MPM force loop added;
      D-P projection corrected to Klar 2016 eq. 3.1.
- [x] arXiv venue subcategory. **Resolved Pivot 5:** primary submission
      target is Biosystems Engineering (Elsevier); arXiv cross-list as
      `cs.RO` (robotics) primary + `physics.geo-ph` (granular mechanics)
      secondary — *not* `cs.GR`.
- [ ] M4.5b friction regression — is the projection point wrong (after
      mass-average) or is the algorithm itself flawed? Need M4.5b' variant
      that applies friction before mass-average to disambiguate.
      **Updated 2026-05-15 (Pivot 4):** With real stress sand, M4.5b
      regression flipped to "M4.5b == M4.5a on box_drop, but introduces
      slip overshoot on rover_wheel". The masking hypothesis is now the
      leading explanation — Layer B was compensating for missing
      constitutive physics. Paper §6 narrative rewrite scheduled for W2.
- [ ] L3 (hardware deployment) — myCobot pipeline scaffold to start W3
      after Klar calibration closes. Design contract locked in
      `docs/notes/l3_pipeline_design.md`. Pipeline must support
      {280, 320, Pro 600} × {kinematic-only, joint-current, F/T sensor}
      matrix; user picks at experiment time.
- [ ] M4.5c kernel regression — re-normalising after kernel application
      would restore Σm_b = M_body; trade-off vs cost of additional pass.
      Decide after M2 ASFLIP implementation (might subsume M4.5c).
- [ ] Dwell time threshold currently 5 cm (ad-hoc). Switch to
      dimensionless (e.g. half-body-extent) for paper.

## Key Outputs Registry

| Type | File | Description |
|------|------|-------------|
| Library | `lib/basements_math` (header-only) | Vec3 AVX2, Quaternion, Matrix3, SVD |
| Library | `lib/basements_scene` | SceneGraph, CommandHistory |
| Library | `lib/basements_io` | URDF parser/exporter/bridge |
| Library | `lib/basements_coupling` | MPMRigidCoupler{,PIC} |
| Library | `lib/basements_physics` (header-only sub-modules) | MPM, joints, formulas |
| Executable | `build/bin/Release/basements_editor.exe` | ImGui editor |
| Executable | `basements_editor_portable/basements_editor.exe` | Distributable bundle |
| Executable | `build/Release/bench_energy_drift.exe` | Drift benches (5 scenarios) |
| Executable | `build/Release/bench_mpm_rigid_drift.exe` | MPM ↔ rigid drift (M0/M1) |
| Figure | `outputs/figures/energy_free_fall.png` | Integrator-only KE/PE |
| Figure | `outputs/figures/momentum_isolated_pair.png` | Momentum conservation |
| Figure | `outputs/figures/energy_damped_slider.png` | Damping monotonicity |
| Figure | `outputs/figures/solver_stack_drift.png` | Solver stack drift |
| Figure | `outputs/figures/solver_pendulum.png` | Joint pendulum energy |
| Figure | `outputs/figures/mpm_rigid_drift.png` | **M0 Dirichlet baseline** |
| Figure | `outputs/figures/mpm_rigid_drift_pic.png` | **M1 PIC-Rigid (35× ↓)** |
| Workflow | `.github/workflows/drift_bench.yml` | Weekly CI long-form drift |

## Docs Registry

| File | Purpose | Created | Last Updated |
|------|---------|---------|--------------|
| `docs/PROJECT_STATUS.md` | This file — phase + decisions overview | 2026-05-13 | 2026-05-13 |
| `docs/FORK_NOTE.md` | Main-engine vs research sub-project boundary | 2026-05-13 | 2026-05-13 |
| `docs/roadmap.md` | Phase 0–5 milestone plan | 2026-05-13 | 2026-05-13 |
| `docs/checklist.md` | 30/60/90 day actionable checklist | 2026-05-13 | 2026-05-13 |
| `docs/notes/limitations.md` | Two structural limits + adoption/validation paths | 2026-05-13 | 2026-05-13 |
| `docs/notes/l3_pipeline_design.md` | myCobot sim2real bridge design contract — implementation slot W3 | 2026-05-15 | 2026-05-15 |
| `docs/notes/literature_uncertainty.md` | Per-experiment measurement uncertainty floors — pass criteria source | 2026-05-15 | 2026-05-15 |
| `docs/notes/sim_validity_domain.md` | Green/Yellow/Red regime classification for cherry-picking defence | 2026-05-15 | 2026-05-15 |
| `docs/notes/external_validation_protocol.md` | Third-party lab reproduction protocol — paper #1 Appendix C | 2026-05-15 | 2026-05-15 |
| `docs/notes/zenodo_setup.md` | DOI-backed pre-registration workflow setup guide | 2026-05-15 | 2026-05-15 |
| `docs/notes/geotech_outreach.md` | Tier 1–4 cascade for measured sand parameters | 2026-05-15 | 2026-05-15 |
| `validation/external_submissions/README.md` | External lab submission entry point | 2026-05-15 | 2026-05-15 |
| `case_studies/hardware/_template/preregistration.md` | Pre-registration template per experiment | 2026-05-15 | 2026-05-15 |
| `extensions/ros2_bridge/README.md` | W3 ROS 2 bridge scaffold — overview + quick start | 2026-05-15 | 2026-05-15 |
| `extensions/ros2_bridge/config/*.yaml` | 5 robot YAMLs + 2 sand YAMLs (Ottawa F-65 + Jumunjin) | 2026-05-15 | 2026-05-15 |
| `LICENSE` | MIT (code) + CC-BY-4.0 (data) dual licence | 2026-05-15 | 2026-05-15 |
| `CITATION.cff` | Software citation metadata (Hong-Gu Lee, KNU Smart Agriculture + Biosystems) | 2026-05-15 | 2026-05-15 |
| `CONTRIBUTING.md` | Contribution policy + pre-reg immutable commit rules + authorship | 2026-05-15 | 2026-05-15 |
| `AUTHORS.md` | Maintainer + contributors registry | 2026-05-15 | 2026-05-15 |
| `paper/main.tex` + `paper/sections/*.tex` + `paper/references.bib` + `paper/README.md` | LaTeX paper draft (8 sections, 17 BibTeX) + venue porting guide | 2026-05-15 | 2026-05-15 |
| `docs/notes/INDEX.md` | docs/notes/ category-grouped index | 2026-05-15 | 2026-05-15 |
| `research/.../scenarios/scenario_tine_drag.h` | Pivot-5 agricultural scenario — cultivator tine drag | 2026-05-15 | 2026-05-15 |
| `.github/workflows/ci.yml` | CI: build + tests + reproduce_findings + paper LaTeX | 2026-05-15 | 2026-05-15 |
| `docs/notes/pivot6_differentiable_mpm.md` | Paper #2 design — differentiable MPM-rigid + adjoint + 9-month roadmap | 2026-05-15 | 2026-05-15 |
| `docs/paper_outline.md` | **ARCHIVED at Pivot 6** — paper #1 prose, paper #2 §3.1+App.A source | 2026-05-13 | 2026-05-15 (archived) |
| `paper/` | **ARCHIVED at Pivot 6** — paper #1 LaTeX, paper #2 forward-sim source | 2026-05-15 | 2026-05-15 (archived) |
| `research/mpm_rigid_coupling/README.md` | Paper #1 benchmark suite scope | 2026-05-13 | 2026-05-13 |
| `research/mpm_rigid_coupling/THEORY.md` | Mathematical derivation per method + metric definitions | 2026-05-13 | 2026-05-13 |
| `docs/multi-scale_engine.md` | Architecture + API status (v11.0) | 2026-03-31 | 2026-05-13 |
| `docs/api_reference.md` | Full API specification | 2026-03-31 | 2026-03-31 |
| `docs/paper_outline.md` | Paper #1 outline + figure mapping | 2026-05-13 | 2026-05-13 |
| `docs/notes/element_tech.md` | Element tech lab notebook | 2026-05-13 | 2026-05-13 |
| `docs/notes/references.md` | Working bibliography | 2026-05-13 | 2026-05-13 |
| `docs/notes/integration_notes.md` | Cross-system gotchas | 2026-05-13 | 2026-05-13 |
| `docs/notes/risk_log.md` | Active risks + mitigations | 2026-05-13 | 2026-05-13 |

## Pivot Record

### Pivot 1 — 2026-05-13 (Round 7 → Round 8) — scope reframe

- **Reason:** Vision slogan ("From Earthworms to Galaxies") was unbounded;
  7 rounds of UI rework showed 1-person bandwidth cannot sustain
  general-purpose engine ambitions while also producing publications.
- **What changed:** Scope reframed to "Open MPM ↔ Rigid coupling benchmark
  suite for robotics in granular media". Galaxy / N-body removed. CUDA
  demoted to Phase 3 (no longer in "current state"). Editor demoted to demo
  viewer (no further general-purpose editor work outside crash fixes).
- **Archived:** Nothing yet — pivot is descriptive (no code deletion). The
  `outputs/before/` and `docs/before/` directories remain available if a
  hard archival event happens later.
- **Carry-forward:** Entire Phase 0 codebase + 84 tests + 6 figures + M0/M1
  couplings + paper outline.

### Pivot 6 — 2026-05-15 (Round 20) — Paper #1 dropped, Paper #2 = Differentiable MPM-Rigid Coupling

- **Reason:** Honest audit of paper #1's algorithmic contribution
  surface (M4.5a renormalised two-pass sweep + Pivot-4 meta-finding
  + agricultural framing + open benchmark scaffold) revealed that
  **the algorithmic novelty bar for SIGGRAPH-class submission is
  marginal**. Most of paper #1's content is faithful reproduction of
  Klar 2016 (Hencky + Drucker-Prager) and Hu 2018 (MLS-MPM force
  formula). M4.5a's renormalisation is a defensible engineering
  optimisation but reviewer-fragile. User instinct (*"독창성이 잘
  안보여"*) confirmed the audit. Decision: pivot to genuinely novel
  work that exploits the existing infrastructure.

- **Path chosen:** Path C1 + C3 combined — **Differentiable
  MPM-Rigid Coupling for Agricultural Robot Sim-to-Real**.
  Wet/cohesive constitutive (C3) folds into C1 as a domain
  randomization axis on the same differentiable forward sim. Paper
  #3 = Path C4 + C2 (GPU + adaptive resolution) for real-time RL.

- **Why this path:**
  - Leverages user's AI/ML expertise (gradient-based optimisation,
    PyTorch autograd, RL).
  - Direct identity fit (KNU Smart Agriculture + Biosystems
    Engineering → agricultural-robot policy learning).
  - myCobot hardware re-used for sim-to-real validation.
  - Existing paper #1 work (forward sim, calibrated continuum,
    tine_drag scenario, validation tests) is the *foundation*, not
    a sunk cost.
  - DiffTaichi (Hu 2020), DiffMPM (Huang 2022), gradSim (Murthy
    2021), Brax (Freeman 2021) define the open research frontier;
    rigid-body coupling is the underexplored corner.

- **Paper #1 disposition:** **Dropped entirely.** No JOSS, no
  SoftwareX, no arXiv preprint, no formal release. Code stays on
  GitHub as paper #2's baseline. `docs/paper_outline.md` and
  `paper/main.tex` are archived in place (no destructive moves);
  they remain useful reference material and source of §3.1 + App. A
  prose for paper #2.

- **5-paper series re-ordering:**
  - ~~Paper #1: Open Benchmark~~ → **DROPPED**.
  - **Paper #2 (was #1)** = Differentiable MPM-Rigid for
    Agricultural Sim-to-Real. Phase 2. ICRA / CoRL / NeurIPS.
  - **Paper #3** = GPU + Adaptive Resolution for Real-Time RL
    Training Loop. Phase 3. RSS / HPG / ICRA.
  - **Paper #4** = Field Industrial Deployment. Phase 5. T-RO /
    Field Robotics.
  - ~~Paper #2: Community Validation~~ → **folded into paper #2 §6**.
    External-validation protocol (paper #1 Appendix C) stays
    operational; community submissions extend paper #2.

- **Documentation impact:**
  - paper_outline.md gains ARCHIVE marker; remains valid as
    forward-sim reference text.
  - paper/main.tex + sections stay in tree but tagged archived.
    Paper #2 work starts in fresh paper2/ directory when first
    pybind11 binding is operational.
  - roadmap.md 5-paper outline re-ordered.
  - New design note `docs/notes/pivot6_differentiable_mpm.md`
    captures paper #2 architecture, adjoint method sketch,
    implementation roadmap.
  - L3 ros2_bridge scaffold + external_submissions protocol
    + pre-registration workflow remain as paper #2 deliverables.

- **Carry-forward (95%+ of paper #1 work survives in paper #2):**
  - Forward sim (MPMSolver + Hencky + Klar + M4.5a + scenarios) →
    §3.1 + Appendix A
  - §1 motivation + §2 related work + §3 method (dual-track) →
    transferred with differentiable-sim literature added
  - §5 result tables (M0/M1/M4.5 ablation) → paper #2 §4 forward
    validation
  - Lube H/D sweep + repose calibration → §4 validation
  - L3 ros2_bridge + myCobot pipeline → §4.5 sim-to-real hardware
  - External validation protocol → §6 community section
  - Documentation (literature_uncertainty, sim_validity_domain,
    l3_pipeline_design, etc.) → all carry forward

- **Lost (acceptable):** paper #1's standalone publication
  ambitions (JOSS / SoftwareX / SIGGRAPH Asia TC). These were
  marginal-novelty venues that would have consumed 2-4 weeks of
  review-cycle effort for limited citation impact.

### Pivot 5 — 2026-05-15 (Round 19) — Agricultural robotics identity reframing

- **Reason:** Paper draft after Pivot 4 was CG/SIGGRAPH-flavored
  (Hu 2018 / Klar 2016 / SIGGRAPH Asia TC target), out of alignment
  with the author's actual research identity (Hong-Gu Lee, KNU
  Interdisciplinary Program in Smart Agriculture + Department of
  Biosystems Engineering). User explicit:
  *"나는 내 실험대로 하고 내 정체성으로 가져가야지"* — own
  experiments, own identity.
- **What changed:**
  - Paper title: *"An Open MPM-Rigid Coupling Benchmark for
    Agricultural Robotics in Granular Soils: Layered Ablation
    Reverses Under Constitutive Refinement"*
  - §1 motivation reordered: agricultural robotics first (autonomous
    tractors, precision seeders, robotic cultivators), rover/foot
    as secondary regimes.
  - Primary sand: Jumunjin (KS F 2308 Korean standard); Ottawa F-65
    retained as international reference.
  - Acks emphasises KNU Smart Agriculture program.
  - Keywords: agricultural-robotics, soil-machine-interaction,
    precision-agriculture, smart-agriculture, biosystems-engineering
    moved to front.
  - Venue priority (paper/README.md):
    Biosystems Engineering → Computers and Electronics in Agriculture
    → Journal of Terramechanics → IEEE T-AgriFood → ICRA-AG workshop.
    SIGGRAPH Asia TC deprioritised to alternative.
  - References bibliography: added 6 agricultural / terramechanics
    entries (Wong 1989, Godwin 2007, He 2016 tine FEM, Kim 2010
    Jumunjin characterisation, Senatore 2014, Shaikh 2022).
  - Audience priority in roadmap.md reordered (#1 agricultural
    robotics researchers).
- **Carry-forward:** All algorithmic content (M0-M4.5 ablation,
  Pivot-4 meta-finding, Lube H/D sweep, validation protocol)
  preserved. Only framing changed.
- **Open:** Agricultural-specific scenario addition (tine drag /
  seed planter / soil compaction by autonomous wheel) deferred to
  W3 implementation — registry pattern supports it without core
  coupler changes.

### Pivot 4 — 2026-05-15 (Round 18) — MPM constitutive loop completion

- **Reason:** Round 18 Week-1 angle-of-repose work surfaced that
  `MPMSolver` was running as P2G + grid-velocity-update + G2P only,
  with no internal-stress force on the grid. Plastic projection on `F`
  was performed but never produced a Newton-3 reaction on neighbouring
  nodes. The repose pile measurement returned 0° because particles
  cannot resist deformation without stress force, only the floor BC.
- **What changed:**
  - `GridNode::force` field added; `GridBlock::clear()` zeros it.
  - `Particle::volume` now initialised in `add_particle` from
    `mass / density` (was uninitialised → 1.0 default → wrong V_p⁰).
  - `MPMSolver::p2g_single`: Hencky-Kirchhoff stress  τ = U·diag(2μ ε + λ·tr ε)·Uᵀ
    accumulated into grid via MLS-MPM force  f = -(4/dx²)·V_p⁰·τ·(x_i − x_p)·w
    (Hu 2018 eq. 17 / Klar 2016 eq. 3.6).
  - `MPMSolver::update_grid`: velocity update is now
    `v_i = (m·v + f·dt)/m + g·dt` followed by separable floor BC
    with Coulomb tangent friction (was sticky 2-cell slab).
  - `apply_plasticity` projection corrected to Klar 2016 eq. 3.1 form
    with the missing `(3λ+2μ)/(2μ)` kohesion factor. Default α set to
    0.40 (φ ≈ 36° dry sand).
  - Two new test executables: `test_mpm_stress_isolation` (2 tests)
    and `test_mpm_constitutive` (3 tests). All pass.
- **Findings impact under new physics** (re-run via
  `reproduce_findings.py`): 4 / 6 unchanged (Findings 1, 1b, 3, 5).
  Findings 2 and 4 flipped *in physically meaningful ways*:
  - **Finding 2** (rover wheel): under realistic sand, M4.5a *alone*
    suppresses slip; M4.5b's added tangent friction now overshoots.
    M4.5a sinkage = 0 (full surface support), M4.5b sinkage = 0.172 m.
    Paper narrative needs rewrite: layered composition produces
    different optima under different sand fidelities.
  - **Finding 4** (composition not free): M4.5b ≡ M4.5a on box_drop
    momentum error now — Layer B's tangent projection was masking the
    missing constitutive physics; now visibly redundant in this
    scenario. Layer C (M4.5) still regresses 30× → composition cost
    remains a defensible claim, but with a sharper interpretation.
- **Carry-forward:** All paper findings remain defensible; paper §6
  narrative for Findings 2 and 4 will be rewritten in Week 2 to reflect
  the new physics. Findings 1, 1b, 3, 5 are unchanged.
- **Open calibration gap:** repose pile measures 22.97° (geometric) /
  16.45° (slope-fit) vs literature 30–35°. Attributed to (a) grid
  under-resolution (4 cells across pile at dx = 0.1), (b) CFL-limited
  E ≤ 5×10⁵ Pa at dt = 1/240, (c) Lube 2004 column-collapse runout.
  Full Klar 2016 calibration deferred to Phase 2.4 with refined dx
  and funnel-pour scenario.

### Audit & redesign — 2026-05-15 (Round 15 → Round 16)

- **Round 15 audit** found two latent bugs in `M4.5b` and `M4.5c`:
  the `else` branch of the per-node update set `v_after` but forgot
  `node.mass = m_new`, leaving the grid in an inconsistent state.
  Issue #3 (rover-wheel rolling direction sign) and Issue #4 (grid
  mass accumulation across steps without P2G/G2P reset) flagged.
- **Round 16 redesign** restructured `sweep<>` template:
  - Lambdas now return `SweepResult { v_after, m_b_effective }` —
    pure functions of `SweepInput`. Sweep handles all node mutations
    centrally. The bug class is *structurally* impossible going forward.
  - rover-wheel `ω` sign flipped (`-ω₀` on world Z) so forward rolling
    is +X (mathematically derived in `make_wheel` comment).
  - New `test_mpm_rigid_sweep` (6 invariant tests): mass conservation
    Layers A/B, intentional violation Layer C, empty grid, dt=0,
    normalised-mass-matches-body-mass. Now `99 + 6 = 105` tests pass.
- **Measurement robustness:** all Section 5/6 numbers reproduced
  within ~1% of pre-fix values. **All paper findings hold.** Bug had
  small numerical impact because mass-update miss only affected the
  *node-mass-readout-from-next-step* path, not the current step's
  reaction force calculation.

### Pivot 3 — 2026-05-13 (Round 10) — Paper #1 hero claim moved M1 → M4.5a

- **Reason:** Round 10 implemented the two-pass normalised sweep for M4.5
  Layer A and ran the full ablation. M4.5a outperformed M1 by 2.2× on
  max|Σp| at +6% step cost. M4.5b (friction tangent) and M4.5c (B-spline
  kernel) both regress, making them honest-failure cases rather than
  improvements.
- **What changed:**
  - Paper headline shifts from "M1 PIC-Rigid 35× vs M0" to
    "M4.5a renormalised adaptive mass 2.2× vs M1 at +6% cost".
  - `paper_outline.md` updated to a 3-finding ablation narrative.
  - `roadmap.md::Phase 4` reframed: drop standalone engine, embrace
    Python module + Taichi interop + ROS 2 plugin per `limitations.md::L-1`.
- **Archived:** Nothing (results-only pivot).
- **Carry-forward:** All Round 9 code, plus M4.5a as the new hero,
  M4.5b/c as paper Section 4 ablation evidence.

### Pivot 2 — 2026-05-13 (Round 8) — work fork

- **Reason:** Even with the reframed scope of Pivot 1, main-engine work
  (UI / GPU / editor) was still competing with paper-track work for
  bandwidth. Solution: physically separate the two tracks into independent
  build targets so each can move at its own pace.
- **What changed:** New `research/mpm_rigid_coupling/` sub-directory with
  its own `bench_compare_all` executable, scenarios, six coupler IDs
  (M0..M4.5), Python visualisation scripts, and `outputs/{csv,figures,videos}/`.
  Main-engine code is FROZEN for non-critical work; only critical fixes
  allowed. See `FORK_NOTE.md` for the canonical boundary map.
- **Archived:** Nothing.
- **Carry-forward:** Main engine still builds, 84/84 tests still pass,
  M0/M1 in `basements_coupling` are re-exported (not duplicated) as
  `research::mpm_rigid::M0/M1`.
- **First Track B build verified:** `bench_compare_all.exe --all` writes
  six per-method CSVs + six frames CSVs; `plot_compare.py` produces
  `compare_box_drop.png` + desc.md. Pipeline end-to-end functional even
  with M2..M4.5 as stubs.

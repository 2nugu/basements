# Roadmap & open problems

The outward-facing map of where **Basements** is, where it's going, and — honestly —
what is *not* solved yet. Detailed internal planning lives in
[`docs/roadmap.md`](docs/roadmap.md) and [`docs/notes/limitations.md`](docs/notes/limitations.md);
this file is the front door and the [where to help](#where-to-help) guide.

> **One-line status.** A single-engineer research project. The MPM soil solver, the
> rigid/joint solver, the URDF robot-arm import, and the M0/M1/M4.5 coupling ablation
> are real and tested. It is **not** a production engine, and the granular results are
> **not yet validated against measured terramechanics data**.

## Where we are

**Phase 0 — Foundation (done):** AVX2 math core (Vec3/Quaternion/Matrix3/SVD);
granular MPM with Drucker–Prager plasticity; GJK+EPA collision; a sequential-impulse
rigid solver with joints (ball/hinge/slider/fixed); **URDF import + a ROS 2 myCobot
sim-to-real bridge**; and the MPM↔rigid coupling ablation — **M0 Dirichlet**,
**M1 PIC-rigid** (35× better than M0), and **M4.5a** (a further 2.2× at parity cost).

**Phase 1 — active:** the differentiable direction — adjoint method through the
Hencky/Drucker–Prager plastic projection, toward agricultural-robot sim-to-real.

**Phases 2–5 — planned:** more coupling methods (M2 ASFLIP, M3 MLS-MPM, M4 Lagrange),
terramechanics calibration, a GPU CUDA backend, a pybind11 Python module + ROS 2
plugin, and eventually a smart-farm application. See [`docs/roadmap.md`](docs/roadmap.md)
for the full phase plan and the three-paper series.

## Open problems & unknowns

Stated plainly (drawn verbatim-in-spirit from [`docs/notes/limitations.md`](docs/notes/limitations.md),
which is deliberately un-softened). Each is an invitation.

1. **No production engine ships friction-dominated MPM↔rigid coupling** (Bullet,
   MuJoCo, Isaac, PhysX are rigid-only). This is an academic *strength* (an open
   benchmark for a real gap) but an industrial *risk* — hence the decision to ship a
   **coupling library for use inside other engines**, not a standalone engine.
2. **The test environment is still a toy:** idealised dry sand + a 1 kg box primitive.
   No wet/cohesive soil, no non-convex bodies (cylindrical wheels, articulated legs),
   no real load-cell / sinkage / drawbar-pull validation. Until this is addressed the
   results read as *synthetic only*. There's a concrete mitigation ladder in
   `limitations.md` (rover_wheel vs Bekker, foot_step rebound, static pile repose angle).
3. **M2 / M3 / M4 couplers are placeholder stubs** in the released code — only M0, M1,
   and M4.5{a,b,c} are real implementations. Any comparison must disclose this.
4. **A methodological caution (meta-finding):** two of five coupling-ablation findings
   *reverse direction* when re-run under validated sand stress vs the earlier stub
   stress. Conclusions from coupling ablations are fragile to the underlying
   constitutive fidelity — this is itself a contribution, not a bug.
5. **Differentiable-mode caveats:** gradient consistency is verified only at small
   scale (a tractable CPU rollout, not a full training loop — needs the GPU backend);
   the plastic-projection adjoint has a measure-zero discontinuity on the yield cone
   (subgradient treatment, FD-verified on the smooth interior); and sim-to-real is so
   far single-platform / single-soil / single-author.

## Where to help

Mapped to GitHub issues by label — [good first issue], [help wanted], [open-problem].

**Good first issues** (small, well-scoped):

- Add a **cylinder collider primitive** to `MPMRigidColliderState` (currently box-only)
  — a prerequisite for the rover-wheel scenario.
- Add a **`static_pile`** scenario and check the sand's angle of repose lands in the
  textbook 30–35° band for dry sand.
- Regenerate / tidy a drift or comparison figure via the existing plot scripts.

**Help wanted** (larger, more context):

- Implement the **`rover_wheel`** and **`foot_step`** validation scenarios and compare
  against the Bekker sinkage formula / dimensionless rebound coefficient (unblocks
  open problem #2 and gates the paper).
- Implement the **M2 ASFLIP** coupler (F-bar projection at the rigid–grid boundary) as
  the third real method in the ablation.
- Build the **pybind11 Python module** exposing `MPMSolver::step` + the couplers — the
  ranked-#1 adoption path for Taichi / Isaac Lab / ROS 2 users.

**Open problems** (research-scale):

- Validate against **measured terramechanics data** (wet/cohesive soil; a published
  Bekker/Wong or field-plot dataset). This is the threshold for the work to be taken
  seriously beyond a synthetic benchmark.
- A real **GPU CUDA backend** for the forward + backward sim (currently CPU-only).

## Contributing

Contributions of new couplers, constitutive variants, validation scenarios, or
dataset submissions are welcome. Note the **authorship policy** in
[`CONTRIBUTING.md`](CONTRIBUTING.md): landing a substantive feature (a new coupling
method, constitutive variant, or scenario via the case-study registry) makes you
eligible for co-authorship on the relevant future paper. Every new top-level feature
needs a regression test, and the repo stays buildable + green at each phase exit.

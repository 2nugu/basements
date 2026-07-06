# Element Technology Notes

Working lab notebook of *technical building blocks* required across the roadmap.
Each entry: what it is, why we need it, current state in `Basements`,
references, open questions.

---

## MPM granular material (sand / snow / regolith)

**What:** Material Point Method with sparse SP-Grid storage. Particles carry
mass, velocity, deformation gradient F, APIC momentum tensor C. Grid solves
F=ma per node, applies BC, returns velocity to particles.

**Why:** Core of the granular contribution.

**Current state:**
- `MPMSolver::step` runs P2G → grid update (gravity + floor BC) → G2P.
- Drucker–Prager plasticity via SVD per particle (`apply_plasticity`).
- B-spline weights verified after the 2026-03-21 bug.

**Canonical sand parameters (TODO — lock these before benchmarks)**

Two common reference sets in the literature:

| Source | E (Pa) | ν | μ_fric | ρ (kg/m³) | Note |
|---|---|---|---|---|---|
| Stomakhin 2013 (snow) | 1.4e5 | 0.2 | 0.2 | 400 | Soft snow; not the granular reference |
| Klar 2016 (sand) | 3.537e5 | 0.3 | 0.6 (tan φ=31°) | 2200 | **Canonical sand reference** |
| Hu 2018 (MLS-MPM)      | 1.0e4 | 0.3 | 0.4 | 1000 | Demo defaults — not physical sand |

**Decision:** Use Klar 2016 sand parameters as the canonical reference across
all paper figures. Lock into a header `include/basements/physics/mpm/material_presets.h`
once Phase 1.1 lands.

**Open question:** our MPM solver currently uses elastic-only stress
(no real constitutive model — just F evolution + Drucker–Prager projection).
For paper rigor we should add a hyperelastic energy density (Fixed-Corotated
or Neo-Hookean) so first Piola–Kirchhoff stress is principled. ~1 day of work,
but unblocks "physically accurate" claim.

**References:**
- Stomakhin et al. 2013, "A material point method for snow simulation"
- Klar et al. 2016, "Drucker-Prager elastoplasticity for sand animation"
- Hu et al. 2018, "A Moving Least Squares Material Point Method"
- Jiang et al. 2015, "The Affine Particle-In-Cell Method"

---

## PIC-Rigid coupling

**What:** rigid bodies contribute mass + velocity to grid nodes their OBB
overlaps; mass-weighted average velocity is written back; body reaction force
is accumulated from the per-node velocity delta.

**Why:** Removes the momentum-injection error of the M0 Dirichlet MVP. Measured
35× improvement in `max |Σ p_y|` on the box-on-sand benchmark.

**Current state:** `MPMRigidCouplerPIC::apply` is a real implementation. Uniform
mass distribution across the OBB volume (`body.mass / approx_nodes`).

**Known limitations:**
- Uniform mass = inaccurate when the body only partially penetrates the grid
  (per-node mass over-estimates when fewer than `approx_nodes` cells are
  actually inside).
- No friction tangent projection (Dirichlet MVP has it).
- No B-spline weighting — body contribution is uniform, not kernel-shaped.

**Refinement queue:**
1. Replace `approx_nodes` with the *measured* inside-count from a first pass.
2. Add Coulomb tangent friction.
3. Switch to quadratic B-spline weight at the body's OBB-cell intersection.

**References:**
- Stomakhin 2014 (snow PIC-FLIP)
- Hu Taichi MPM coupling examples
- Fei et al. 2021, "A Multi-Scale Model for Coupling Strands with…"

---

## ASFLIP rigid extension (M2)

**What:** Affine Particle-in-Cell + FLIP hybrid that uses F-bar projection
to split deviatoric and volumetric strain. The rigid extension applies F-bar
at the rigid-grid boundary so the rigid body sees pure deviatoric grid
feedback while volumetric momentum exchange is exact.

**Why:** Momentum-exact two-way coupling (within float roundoff), unlike PIC-Rigid
which has small unavoidable drift due to mass-weighted averaging.

**Current state:** Not implemented. Paper outline + roadmap Phase 2 only.

**Required building blocks:**
- F-bar decomposition: split F → F̄ · F_vol where F̄ is trace-preserving.
- Per-particle history of F̄.
- New `MPMRigidCouplerASFLIP::apply` integrating with the F-bar split.

**References:**
- Fei et al. 2021 SIGGRAPH, "A multi-scale model…" (primary)
- Klar 2016 (F-bar in sand context)

---

## URDF — robot description format

**What:** XML schema for robot kinematic + collision + inertial properties.
Industry standard for ROS robots.

**Current state:**
- Parser: `URDFParser` works (TinyXML2).
- Scene-graph conversion: `URDFConverter::to_scene_graph` creates link nodes
  AND (Round 7 fix) joint nodes with `connected_body_a/b` populated.
- Physics-world spawn: `URDFPhysicsBridge::spawn` with
  `initial_joint_positions` support.
- Auto-wire on Play: `EditorApp::play` reads JOINT nodes and creates
  PhysicsWorldGPU constraints in `node_to_constraint_map`.

**Known gaps:**
- FLOATING and PLANAR joint types not modeled (skipped).
- No support for `<safety_controller>`, `<mimic>`, `<transmission>`.
- Mesh visuals not rendered (collision boxes only).
- Continuous joints currently default to ±π soft range — real URDF semantics
  have no limit at all.

**References:**
- ROS Wiki — http://wiki.ros.org/urdf/XML
- Drake & MoveIt URDF parsers (cross-check edge cases)

---

## ImGui editor architecture

**What:** Dear ImGui (docking branch) + GLFW + OpenGL 3.3 viewport.

**Current state:**
- Hierarchy / Inspector / Console / PhysicsPanel docked.
- Gizmo (translate/rotate/scale) operational.
- Multi-slot snapshot (F5..F8 + Shift+F5..F8 + F9/F10 + JSON I/O + label
  modal).
- All menu/toolbar/import callbacks emit `[trace]` to stderr + Console.
- Joint slider live-driven via `set_joint_velocity`.

**Known fragility:**
- Hard-coded "scene.json" in `Open Scene` (now guarded).
- No real file dialog — drag-drop only.
- Inspector NodeType::JOINT branch is a TODO for `set_joint_position` Inspector
  slider when SIMULATE mode active.

**Design decisions to NOT revisit:**
- PIMPL on `EditorApp` — keep it, allows Impl additions without header churn.
- Single global `g_editor_app_instance` for GLFW C callbacks — annoying but
  necessary; alternatives all leak state to GLFW.

---

## Sparse grid (SP-Grid)

**What:** Block-structured sparse grid for MPM. Active nodes only allocate
their containing block (4³ = 64 nodes per block). Hash-map keyed on
`(block_x, block_y, block_z)`.

**Current state:** CPU `SPGridCPU` with `unordered_map<uint64_t, GridBlock>`
and 21-bit per-axis index packing.

**Known limits:**
- No iteration order optimisation (cache misses on random hash order).
- No GPU equivalent — `SPGridGPU` would need a different data structure
  (linearised index buffer + SoA).
- 21-bit packing limits world extent to ±1e5 cells (= ±10 km at dx=0.1).

---

## Sequential Impulse rigid solver

**What:** Constraint-based velocity solver iterating contacts + joints until
convergence (typically 10–20 iters/frame).

**Current state:** `Solver::pre_solve` (warm start + effective mass) →
N × `Solver::solve_velocity`. World-space inertia is recomputed each frame
(critical bugfix from Round 5).

**Known limits:**
- No position-level (PGS) splitting; relies on Baumgarte for penetration
  correction — small ringing on stiff stacks.
- No CCD (fast bodies tunnel).
- Single contact manifold per pair (no point reduction or persistence beyond
  the warm-start cache).

---

## How this notebook is used

- Update **at decision points**, not on every commit.
- When a new element technology is introduced (e.g. ASFLIP), add a section here
  BEFORE writing the code — forces the API and limits to be considered first.
- Register new sections in `PROJECT_STATUS.md` "Docs Registry".

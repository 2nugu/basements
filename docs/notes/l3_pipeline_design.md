# L3 Pipeline Design — myCobot Sim2Real Bridge

**Status:** Design contract only. Implementation slotted for Week 3
(after Klar W2 calibration). Do not start scaffolding until W2 closes.

**Scope:** Pipeline that lets `MPMSolver` predict end-effector forces for
a myCobot arm interacting with a granular medium, with results comparable
to physical measurement on the same arm.

---

## 1. Why this document exists now

L3 (real-world deployment) is on the project roadmap as the paper §7
differentiator. The user's hardware is a myCobot (model TBD at experiment
time). To prevent W2 work from accidentally foreclosing L3 design choices
(e.g. baking in a single-model URDF, or hardcoding force-sensing path),
this document captures the *invariants* that the W2 solver, the W3 ROS 2
bridge, and the W4 case studies must respect.

---

## 2. Hardware matrix the pipeline must accommodate

The user will pick one row at experiment time. Pipeline must support all.

| Model            | Payload  | Repeat  | Reach  | Use case                          |
|------------------|----------|---------|--------|-----------------------------------|
| myCobot 280 (Pi/M5) | 250 g  | 0.5 mm  | 280 mm | Sphere drop, mini-probe insertion |
| myCobot 320      | 1 kg     | 0.5 mm  | 320 mm | Full probe insertion, drag        |
| myCobot Pro 600  | 2 kg     | 0.2 mm  | 600 mm | All experiments, large sandbox    |

Force-sensing matrix (orthogonal to model):

| Mode                                | Accuracy | Cost     | Pipeline impact |
|-------------------------------------|----------|----------|-----------------|
| **Kinematic only** (camera depth)   | n/a      | $0       | No /wrench topic; sim publishes prediction only |
| **Joint current → torque estimate** | ±15%     | $0       | `/joint_states.effort` field; needs Jacobian inversion |
| **Wrist 6-DOF F/T sensor**          | ±2%      | $3k–5k   | `/wrench` topic direct from sensor; sim publishes parallel `/sim/wrench` for diff |

---

## 3. Architecture

```
┌──────────────────────┐         ┌───────────────────────────┐
│      myCobot         │         │       MPMSolver           │
│ (280 / 320 / Pro 600)│         │  (W1–W2: validated solver)│
└──────┬───────────────┘         └────────────┬──────────────┘
       │                                       │
       │ pymycobot / ROS 2 driver              │ basements_ros2_bridge
       │                                       │   (W3 scaffold)
       ▼                                       ▼
┌──────────────────────┐         ┌───────────────────────────┐
│ /joint_states        │  ──┐    │  joint_state_sub          │
│ /tf                  │    ├──► │  → RigidColliderState     │
│ /wrench  (optional)  │    │    │  → MPMSolver.step()       │
└──────────────────────┘    │    │  → CouplingReaction       │
                            │    │  → /sim/wrench publisher  │
                            │    └────────────┬──────────────┘
                            │                  │
                            ▼                  ▼
                       ┌───────────────────────────┐
                       │ rosbag2 logger            │
                       │  (real + sim, same clock) │
                       └────────────┬──────────────┘
                                    │
                                    ▼
                            offline compare.py
                            → §7 figure: F_real vs F_sim
```

**Two modes:**
- **Offline-coupled** (paper §7 minimum, achievable Phase 1):
  record real arm trajectory → replay in sim → diff. Sim does not need
  to be real-time. Any modern laptop suffices.
- **Runtime-coupled** (Phase 3 *dependent on GPU backend or adaptive
  resolution research*): sim mirrors arm in real-time, publishes
  predicted force live. **Fundamentally not achievable in Phase 1.**

**Why runtime-coupled is fundamentally out of reach for Phase 1:**

Conservative sandbox size (200 × 200 × 100 mm) at dx = 5 mm gives
grid = 40 × 40 × 20 = 32,000 cells. APIC standard 8 particles/cell yields
**256,000 particles**. Current MPMSolver step time (extrapolated linearly
from W1 measurement: 3.5 μs/particle) ≈ 896 ms / step. ROS 2 100 Hz
control loop demands ≤ 10 ms / step — *90× over budget*. This gap is
algorithmic, not implementation-tunable.

Mitigation paths (none in Phase 1):
- Multi-thread p2g/g2p (4 cores): ~4× → still 22× over.
- GPU CUDA (Claymore-style, Hu 2018): 50–100× → 9–18 ms. **Phase 3.**
- Adaptive-dx MPM (coarse background + fine near end-effector): ~4×.
  Rare in literature (most MPM implementations are uniform-dx including
  Hu 2018, Klar 2016, Tampubolon 2017). **Phase 3 paper candidate** —
  "Adaptive-Resolution MLS-MPM for Real-Time Robotic Granular
  Interaction".
- Particle aggregation outside influence zone: 2–3×, engineering hack.

**Paper §3 honest statement (lock-in):**
> "We focus on offline-coupled validation as the Phase-1 deliverable.
> Runtime-coupled deployment requires either GPU acceleration or
> adaptive resolution (future work)."

The W3 scaffold builds offline first — same data flow becomes runtime
when (and only when) GPU or adaptive-dx lands, with no API changes.

---

## 4. Sim-side API contract (already met by W1 solver)

The W3 bridge will use *only* these public methods. If W2 work needs to
modify any of them, surface the change here first.

| Method                                            | Why bridge needs it |
|---------------------------------------------------|---------------------|
| `MPMSolver::step(float dt)`                       | Each ROS 2 tick advances sim by `dt` |
| `MPMSolver::set_material(E, ν, ρ)`                | Sandbox material from `config/sandbox.yaml` |
| `MPMSolver::set_plastic_alpha(α)`                 | Friction angle from `config/sandbox.yaml` |
| `MPMSolver::set_floor_friction(μ_floor)`          | Container floor friction |
| `MPMSolver::set_gravity(g)`                       | Vertical orientation calibration |
| `MPMSolver::get_grid_mutable()`                   | Coupler reads/writes grid between p2g/g2p |
| `MpmRigidCoupler::apply(state, dt) → CouplingReaction` | Force/torque on end-effector |
| `RigidColliderState { center, orientation, half_extents, linear_velocity, angular_velocity, mass, friction, shape }` | End-effector geometry wrap |

All of these are *already* exposed and ROS 2-compatible. **No further API
work is required from W2.**

---

## 5. Force-sensing pluggability — strategy pattern

The bridge must not bake in one force-sensing mode. Define an interface:

```cpp
struct ForceMeasurement {
    Vec3   force_world;
    Vec3   torque_world;
    float  std_dev_estimate;   // calibrated accuracy of this source
    double timestamp_sec;
};

class ForceEstimator {
public:
    virtual ForceMeasurement read() = 0;
};

class JointCurrentEstimator : public ForceEstimator { ... };  // ±15%, free
class WrenchSensorEstimator  : public ForceEstimator { ... };  // ±2%, $3k+
class KinematicOnlyEstimator : public ForceEstimator {
    // returns null force; only depth tracking
};
```

YAML config picks the implementation:
```yaml
# config/mycobot_280_kinematic.yaml
model: mycobot_280
force_estimator: kinematic_only
```

This means *one bridge codebase supports all four rows of the matrix* and
the user picks at experiment time without touching code.

---

## 6. First-experiment design lock-in

Experiments are listed in order of `cost × difficulty`. All must produce
data that fits in `case_studies/hardware/<experiment>/data/` and can be
compared via a single `compare.py`.

### Experiment A: Sphere drop kinematics — pre-registered validity test
- **Cost:** $20 (sand + container only)
- **Hardware:** Any myCobot model + smartphone 240 fps camera
- **Force estimator:** `KinematicOnlyEstimator`
- **What we measure:** ball penetration depth after release from
  controlled heights **{5, 10, 15, 20, 25, 30, 35} cm** (7 points).
  Heights chosen to *deliberately span all three validity regimes*,
  not just the green regime — this is the cherry-picking defence.
- **Pre-registered regime classification** (declared BEFORE measurement,
  with declared parameter d50 for the sand):
  - h = 5 cm  → predicted depth d_pred ≈ 2·d50  → **yellow regime**
                (boundary: discrete grain effects)
  - h ∈ {10, 15, 20, 25} cm → predicted d_pred ∈ [4, 12]·d50,
                v_impact ∈ [1.4, 2.2] m/s → **green regime**
  - h ∈ {30, 35} cm → v_impact ∈ [2.4, 2.6] m/s, approaches the
                v_critical ≈ 5 m/s boundary → **yellow regime** (kinematic
                shock effects start mattering)
- **What sim predicts:** same depth from same release condition
- **Pass criterion (green points only):** mean depth within ±15% of
  measured, n = 10 per height (Pacheco-Vázquez 2011 uncertainty floor).
  Yellow points reported but excluded from pass count.
- **Pass criterion (validity domain claim):** the *predicted-yellow*
  points must show measurable deviation from sim (>15% error) — this
  is the *positive evidence* that our regime boundary is at the right
  location, not arbitrary.
- **Reference:** Walsh et al. 2003 ([DOI](https://doi.org/10.1103/PhysRevLett.91.104301)),
  Pacheco-Vázquez et al. 2011 ([DOI](https://doi.org/10.1103/PhysRevLett.106.218001)).

**Pre-registration commitment:** The 7-height schedule and regime
classification rule are committed to *before* any measurement begins,
and stored in `case_studies/hardware/A_sphere_drop/preregistration.md`
with the user's signature and date. This is the *cherry-picking defence*:
we tested both passing and predicted-failing cases, with the classification
fixed in advance.

### Experiment B: Vertical probe insertion
- **Cost:** $5 (aluminum rod) + sand
- **Hardware:** 280 (probe ≤ 50mm), 320 / Pro 600 (full probe)
- **Force estimator:** `JointCurrentEstimator` (need shoulder J₁ torque)
- **What we measure:** end-effector reaction force vs depth at fixed
  insertion rate (5 mm/s)
- **What sim predicts:** F(d) curve for same probe geometry, soil params
- **Pass criterion:** F(d) within ±25% across the 0–50 mm range
- **Reference:** Brzinski & Durian 2010, Stone et al. 2004

### Experiment C: Horizontal probe drag
- **Cost:** $0 above B
- **Force estimator:** `JointCurrentEstimator`
- **Measure:** lateral drag force at fixed depth, fixed velocity
- **Pass criterion:** ±25% across {1, 2, 5} cm depths

### Experiment D: Direct F/T comparison
- **Cost:** $3k+ for wrist sensor
- **Force estimator:** `WrenchSensorEstimator`
- **Measure:** 6-DOF wrench during dynamic interactions
- **Pass criterion:** RMSE of (F_real − F_sim) / max|F_real| < 20%
- **Note:** This is the "ICRA-class" experiment. Optional for Paper #1.

---

## 7. Reserved directory structure (DO NOT CREATE until W2 closes)

```
basements/
├── extensions/
│   └── ros2_bridge/
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── src/
│       │   ├── sim_node.cpp
│       │   ├── force_estimators/
│       │   │   ├── joint_current.cpp
│       │   │   ├── wrench_sensor.cpp
│       │   │   └── kinematic_only.cpp
│       │   └── config_loader.cpp
│       └── config/
│           ├── mycobot_280_kinematic.yaml
│           ├── mycobot_280_joint_current.yaml
│           ├── mycobot_320_joint_current.yaml
│           ├── mycobot_pro600_joint_current.yaml
│           ├── mycobot_with_ft.yaml
│           └── sandbox_drysand.yaml
│
├── case_studies/
│   └── hardware/
│       ├── README.md
│       ├── A_sphere_drop/
│       ├── B_vertical_probe/
│       ├── C_horizontal_drag/
│       └── D_ft_comparison/
│
├── scripts/
│   ├── log_arm_trajectory.py     # pymycobot → CSV
│   ├── replay_in_sim.py          # CSV → MPMSolver → predicted F/T
│   └── compare_real_vs_sim.py    # diff + figure generation
│
└── docs/hardware/
    ├── mycobot_setup.md
    ├── calibration_protocol.md
    └── safety.md
```

---

## 8. Anti-foreclosure checklist (for W2 work)

When making W2 design decisions, verify each item:

- [ ] No new MPMSolver public method removed or renamed (breaks bridge)
- [ ] `RigidColliderState` struct layout stable (bridge serializes it)
- [ ] `MpmRigidCoupler::apply` signature stable (bridge polls per tick)
- [ ] No global state added to MPMSolver (bridge needs per-thread sim)
- [ ] dt parameter stays explicit (bridge controls dt from ROS clock)
- [ ] Friction angle / α conversion stays Klar 2016 eq. 3.2 (bridge
      config uses φ in degrees, must map to α deterministically)

W2 changes that *would* break L3 must be flagged here for explicit
review before merging.

---

## 9. Open questions for W3 kickoff

- [ ] myCobot URDF source — official `mycobot_description` ROS 2 package?
      Verify dimensions match physical arm before first experiment.
- [ ] ROS 2 distribution: Humble (LTS) vs Jazzy (current)? affects driver
      compatibility.
- [ ] pymycobot vs ROS 2 driver? Latter is preferred for `/tf` and
      `/joint_states` standardization.
- [ ] Sandbox dimensions — must fit within arm reach minus probe length.
      280 model: ~150 × 150 × 50 mm. 320 model: ~200 × 200 × 80 mm.
      Pro 600: ~400 × 400 × 150 mm.
- [ ] Sand source — silica sand from hardware store ($10/kg) or
      reference standard (Ottawa F-65, ~$50/kg, geotech-standard)?
      Ottawa F-65 calibration values are published, simplifying L1 → L3
      calibration chain.

---

## 10. Paper §7 scope (what L3 produces)

The minimum paper §7 deliverable is a single figure:

```
F_sphere_drop_depth (mm)
   real (mean ± SD, n=10) │
   sim (single trace)     │
   literature (Pacheco-Vázquez 2011)
```

with caption noting:
1. material parameters used in sim (E, ν, φ, ρ — from a single shear box
   test or a literature reference for the specific sand)
2. release heights tested
3. agreement metric: % within ±20% tolerance band

This is achievable with Experiment A alone (lowest hardware tier). A more
ambitious paper §7 adds Experiment B's F(d) curve as second panel.

---

## 11. Cross-references

- [`roadmap.md`](../roadmap.md) — phase scheduling
- [`PROJECT_STATUS.md`](../PROJECT_STATUS.md) — current phase
- [`limitations.md`](./limitations.md) — sim2real gap discussion
- [`integration_notes.md`](./integration_notes.md) — cross-system gotchas

---

## 12. Change log

- 2026-05-15 — Initial document. Written after user confirmed myCobot
  ownership and L3 commitment. Implementation deferred to W3 (after
  Klar W2 calibration closes).

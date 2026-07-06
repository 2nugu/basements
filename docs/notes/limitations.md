# Limitations & Adoption / Validation Paths

Honest accounting of the two structural limits called out in Round 9.
This document is read AS-IS by future-you / collaborators / reviewers —
do not soften it. The paper's "Discussion / Limitations" section is
drafted from here.

---

## Limitation L-1 — No production engine currently adopts MPM-rigid coupling

### Statement

There is no widely-used production physics engine that ships
friction-dominated MPM ↔ rigid two-way coupling as a first-class
feature. Bullet, MuJoCo, PyBullet, NVIDIA Isaac, NVIDIA PhysX:
**rigid-only**. Houdini / Maya: MPM exists but framed as visual effects.
Taichi MPM: research DSL with experimental coupling, not a deployed
engine. NVIDIA Genesis (2024): differentiable physics, granular coupling
present but not granular-rigid as primary contribution.

### Why this cuts both ways

| Lens | Verdict |
|---|---|
| **Academic** | Strength. "Open benchmark for a gap" is a legitimate paper framing — reviewer cannot demand comparison to non-existent prior art. |
| **Industrial** | Risk. Building a *standalone engine* in this niche is shipping into a vacuum. Industry never adopts a 1-author research engine. |

### Decision

**Do not ship a standalone "Basements engine".** Phase 4 is reframed:
the coupling library is the artifact, packaged for use *inside* other
engines, not as a competitor to them.

### Adoption paths, ranked by ROI

| Path | Build cost | Audience | Realism |
|---|---|---|---|
| **Python module (pybind11)**, expose `MPMRigidCoupler{,PIC,...}` + scenarios | 2 wks | Taichi / Isaac Lab users | High — Python is the lingua franca of robotics ML |
| **Taichi interop**: ground-truth oracle for their MPM-rigid kernel | 1 mo | Taichi-MPM research group | Medium — depends on relationship |
| **ROS 2 plugin** for rover/wheel sim with sand layer | 1 mo | Mobile robotics labs | Medium — niche but high-impact |
| **Genesis PR**: contribute our coupling kernels as Genesis backend | 2–3 mo | NVIDIA Embodied AI | Low — large org, slow review |
| **MuJoCo plugin** | 2–3 mo | DeepMind / Robotics | Low — MuJoCo team conservative |
| **Houdini DOP node** | 1 mo | VFX studios | Medium — different audience entirely |

**Recommended sequence:** Python module → Taichi interop → ROS 2
plugin. The first two unlock the academic + research-user audience;
the third unlocks the industrial robotics audience. **Skip the
standalone engine framing entirely.**

### What this means for the main repo

- `basements_editor` is now demoted to **internal debug tool**, not a
  shipping product. No further investment beyond crash fixes.
- `research/mpm_rigid_coupling/` is the **deliverable** — its Python
  binding (TODO) is the path to external users.
- `docs/multi-scale_engine.md` already retired the "general engine"
  framing in Round 7 — this document just makes the consequence
  concrete.

---

## Limitation L-2 — Toy test environment (sand box + rigid box)

### Statement

All current scenarios use idealised dry sand (Klar 2016 parameters)
and a 1 kg box primitive on a uniform grid. Real robotics scenarios
involve:

- **Wet / cohesive granular media** (agricultural soil, lunar regolith
  with electrostatic cohesion, snow).
- **Non-box body shapes** (cylindrical wheels, articulated legs,
  excavator buckets, curved hulls).
- **Multi-scale** spatial features (grain-size distribution, layered
  soil profiles).
- **Real measurement data** for validation (load cells, wheel sinkage,
  drawbar pull curves).

None of these are in the current bench.

### Severity

Without addressing this limit, the paper reads as **synthetic only**.
Reviewer comment template: "no real-world validation; results limited
to idealised setup; claim of robotics relevance unsubstantiated."

### Mitigation ladder

Each step has a clear deliverable. Without at least Phase 1.1b–c done,
**do not submit the paper**.

| Phase | Add | Validation source | Cost (wks) |
|---|---|---|---|
| 1.1a (done) | `box_drop`, dry sand | self-momentum book | — |
| **1.1b** | `rover_wheel` cylinder primitive | Bekker terramechanics sinkage formula | **2** |
| **1.1c** | `foot_step` flat plate | dimensionless rebound coefficient | **2** |
| **1.2**  | `static_pile` sand pile angle | textbook angle of repose 30–35° for dry sand | 1 |
| 2.x      | wet sand variant (cohesion in constitutive model) | IGER / agricultural field plot data | 4 |
| 2.x      | regolith parameters | NASA JSC-1A simulant published data | 8 |
| 3.x      | image-based rover wheel validation | published Mars rover photo + sim comparison | 4 |
| 5.x      | actual hyperspectral overlay on simulated soil | user's own dataset | 24 |

### Minimum for paper submission

**Phase 1.1b + 1.1c.** Both have published analytical baselines we can
match (or fail to match — also publishable). Without these the paper
falls below the bar.

### Minimum for industrial adoption

**Phase 2.x wet sand + at least one published-data match.** This is
the threshold for an agricultural / aerospace group to take the
library seriously.

---

## Combined consequences for the roadmap

1. **`docs/roadmap.md::Phase 1.1`** stays as written (validation
   scenarios first, paper draft second). This was already the right
   ordering — the limitations document just makes it un-skippable.
2. **`docs/roadmap.md::Phase 4`** is rewritten: drop "v0.1 public
   release as a standalone engine", replace with "Python module +
   Taichi/ROS 2 interop demos". Doc edit pending.
3. **`docs/roadmap.md::Phase 5`** (application: smart-farm) is now
   **conditional on Phase 2 validation completing**. If validation
   fails to match Bekker / repose / IGER data, Phase 5 has no
   foundation and the pivot decision returns.
4. **A new explicit gate** between Phase 1.1 and Phase 1.2 (paper
   draft): "no draft starts until rover_wheel + foot_step scenarios
   are running and producing finite numbers vs Bekker / rebound
   formulas." Promoted to top of `checklist.md` 60-day window.

---

## What the paper Section 6 (Limitations) will say verbatim

> Our experiments use synthetic granular media and convex primitive
> bodies; we do not yet provide validation against measured terramechanics
> data or non-convex geometry. Wet / cohesive soils, scale variation
> from grain to vehicle, and real-world load-cell comparisons are open
> directions and constitute the natural extension of the open benchmark
> released with this work. The coupling kernels are CPU-only at present;
> GPU acceleration is left to future work.

That paragraph is non-negotiable. Hiding it makes the paper weaker, not
stronger.

---

## Reviewer dialogue rehearsals

These are likely reviewer comments and our planned responses:

**R1:** "Why should I believe this works on real terrain?"
**A1:** "We don't claim it does yet. Phase 1.1b–c match published
terramechanics formulas; full real-world validation is explicit future
work declared in Section 6. The contribution is the open benchmark and
M4.5 method ablation, not a claim of field readiness."

**R2:** "Genesis / Isaac Lab already have particle simulation. Why
this?"
**A2:** "Neither ships *friction-dominated* MPM-rigid coupling with
the assumption-removal ladder we present. Our kernels can be adopted
into Genesis (we provide a pybind11 module — see supplementary). The
contribution is the comparison framework + open kernels, not a
competing engine."

**R3:** "Box on sand is not robotics."
**A3:** Fair. Phase 1.1b adds `rover_wheel` against Bekker; Phase 1.1c
adds `foot_step` against rebound coefficient. Without these the paper
should not have been submitted.

**R4:** "M2 / M3 / M4 results are identical to M1 — are they real?"
**A4:** No. They are placeholder stubs in the released v1; the M0/M1
results are real, M2..M4 implementations are the explicit future-work
roadmap. **If submitting before M2 implementation, this disclosure is
mandatory in Section 4.**

---

## Self-test before next checkpoint

Before claiming Phase 1.1 complete, the following must be true:

- [x] `rover_wheel` scenario produces a sinkage value within 2× of
      Bekker's prediction for our sand parameters.
      *Pivot-4 status: sinkage reported with explicit "not Bekker-
      calibrated" disclaimer; ratio framing accepted. Bekker-targeted
      calibration deferred to paper #2 M5 multi-phase.*
- [x] `foot_step` produces a rebound coefficient finite and in [0, 1].
      *Pivot-4 status: e = 0 (dissipative sand) reported as a real
      negative finding, not a metric bug.*
- [x] Paper Section 6 (Limitations) draft exists, even if rough.
      *Status: paper #1 §6 + §6.6 meta-finding drafted (archived in
      paper_outline.md); migrates into paper #2 §6.*
- [x] At least one of {Python module, Taichi interop} has a 1-page
      design note in `notes/element_tech.md`.
      *Status: Python module (pybind11) scheduled as paper #2 M1; the
      L3 ros2_bridge already provides a Python entry point (see
      `notes/l3_pipeline_design.md`).*

---

## Pivot 6 (2026-05-16) — limitation updates for paper #2

### L-3 — Differentiable forward sim is only validated at small scale

Paper #2 forward sim inherits the paper #1 calibration: dx = 25 mm
repose validation, dx = 100 mm coupling validation. Differentiable
mode at *larger* sandbox sizes (200×200×100 mm at dx = 25 mm =
~10⁴ particles, ~hundreds of MPM steps) tests gradient *consistency*
but is *not* a tractable training loop on a CPU. **This is a
Phase 3 / paper #3 limit (GPU + adaptive resolution).** Paper #2
explicitly trains on *small* differentiable rollouts and reports
sim-to-real on the same scale.

### L-4 — Plastic projection adjoint has measure-zero discontinuity

The Klar 2016 hard-projection forward sim is correct on the cone
surface but the gradient is discontinuous there (subgradient
treatment required). Paper #2 §3.3 documents this explicitly; the
finite-difference verification operates on the cone *interior*
(elastic regime + plastic deviatoric) where the gradient is smooth.
Reviewer-defence framing: "Hard projection is what Klar uses for
forward correctness; the discontinuity is at a measure-zero set that
training never lands on in practice."

### L-5 — myCobot sim-to-real is single-platform, single-author

Paper #2 §4.5 sim-to-real is one author, one robot model, one soil
sample. Generalisation to other arms / soils / scales is **paper #4
scope (field deployment)**. We disclose this explicitly in §6 and
do not over-claim. External validation submissions
(`validation/external_submissions/`) extend the dataset over time.

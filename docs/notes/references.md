# References

Core papers, engines, and datasets relevant across phases. **BibTeX
entries are in [`paper/references.bib`](../../paper/references.bib)** —
this file is the human-readable working index that pairs each citation
with editorial notes.

**Sync invariant:** when adding a citation, update *both* files. The
order is: add `.bib` entry first (machine-readable), then add a human
note here (editorial). The Pivot 5 agricultural references
(Wong 1989, Godwin 2007, He 2016, Kim 2010 Jumunjin, Senatore 2014,
Shaikh 2022, Nosek 2015) are already in `.bib` and noted below.

---

## MPM granular & coupling — primary citations

### Stomakhin et al. 2013
> *A material point method for snow simulation.*  
> SIGGRAPH 2013.

The originating MPM-for-graphics paper. Defines the modern P2G/G2P framework
and elasto-plastic constitutive model for snow. Our `MPMSolver` matches its
algorithmic structure.

### Klar et al. 2016
> *Drucker-Prager elastoplasticity for sand animation.*  
> ACM Transactions on Graphics 35(4).

**Canonical sand reference.** Provides the Drucker-Prager yield criterion in
the MPM context plus the standard E/ν/μ parameter set we will lock in.

### Jiang et al. 2015
> *The Affine Particle-In-Cell Method.*  
> ACM TOG 34(4).

APIC formulation — replaces PIC's grid-velocity smearing with affine matrix
transport. Our `Particle.C` field is the APIC tensor.

### Hu et al. 2018
> *A Moving Least Squares Material Point Method with Displacement Discontinuity
> and Two-Way Rigid Body Coupling.*  
> SIGGRAPH 2018.

**Most directly comparable prior art for our M1 PIC-Rigid.** Hu's two-way
coupling uses MLS-MPM kernels rather than B-spline; for our M2 ASFLIP
comparison this is the key baseline.

### Fei et al. 2021
> *A Multi-Scale Model for Simulating Liquid-Hair Interactions.*  
> SIGGRAPH 2021.

**ASFLIP reference for M2.** Uses F-bar split at fluid-hair boundary.
Same principle transfers to rigid-MPM.

### Jiang et al. 2020
> *An Implicit Compressible SPH Solver for Snow Simulation.*  
> SCA 2020.

Lagrange-multiplier coupling alternative — listed in paper outline as M3
but probably out of scope for our 1-author paper.

---

## Robotics simulators (compared in paper Section 2)

### Bullet Physics
- General-purpose rigid-body engine, OSS since 2003.
- No native MPM coupling.
- URL: https://github.com/bulletphysics/bullet3

### MuJoCo
- Constraint-based, focuses on smooth contact (no MPM).
- Acquired by Google DeepMind, OSS since 2022.
- URL: https://github.com/google-deepmind/mujoco

### NVIDIA Isaac / Genesis
- GPU-accelerated, differentiable. Has some particle support but no granular
  MPM with friction-dominated rigid coupling as primary feature.
- Genesis (2024) URL: https://github.com/Genesis-Embodied-AI/Genesis

### Taichi MPM (Yuanming Hu et al.)
- DSL-based, GPU-first MPM with various couplings.
- Our most likely **scoop-risk competitor** for the M1/M2 contribution.
- URL: https://github.com/taichi-dev/taichi
- Monitor: arXiv `cs.GR` filtered by "PIC" + "rigid" + "MPM" weekly.

---

## URDF + ROS

- ROS Wiki URDF spec: http://wiki.ros.org/urdf/XML
- Drake URDF parser (reference quality): https://github.com/RobotLocomotion/drake
- MoveIt SRDF (semantic robot description): for future Phase 4 ROS work.

---

## Granular contact + robotics applications (Phase 5)

### Terzaghi 1925
> *Erdbaumechanik auf bodenphysikalischer Grundlage.*  
> Classical soil mechanics — sinkage / drawbar prediction.

Cited as the analytical baseline our rover-wheel benchmark must beat or match.

### Bekker 1969
> *Introduction to terrain-vehicle systems.*

The terramechanics bible for wheel-soil interaction. Standard parameters
(K_c, K_φ, n) used in offline rover simulators (e.g. NASA's RoSE).

---

## Agricultural soil-machine interaction (Pivot 5)

### Wong 1989 — *Theory of Ground Vehicles*

> J. Y. Wong. John Wiley & Sons, 2nd edition, 1989.

The terramechanics bible — complements Bekker 1969 with traction,
slip, and ploughing-resistance empirical fits. Standard parameters
(K_c, K_φ, n) used in offline rover simulators and as the baseline
draft-equation reference for our `tine_drag` scenario.

### Godwin & O'Dogherty 2007

> *Integrated soil tillage force prediction models.*
> Journal of Terramechanics 44(1):3–14.

Integrates ASAE EP-291.2 standard draft equations with finite-element
predictions for tillage tine forces. Direct comparator for our paper
§5.C `tine_drag` measured specific draught.

### He et al. 2016

> *The effect of tine geometry during vertical movement on soil
> penetration resistance using finite element analysis.*
> Computers and Electronics in Agriculture 130:97–108.

Tine geometry FEM study — useful comparator when we extend
`tine_drag` to varying tine rake angle and operating depth.

### Kim et al. 2010 — Jumunjin sand characterisation

> *Characteristics of Jumunjin standard sand by particle morphology
> and grain size.*
> Journal of the Korean Geotechnical Society 26(3):59–68.

Source for Korean standard sand (KS F 2308 / 주문진) parameters used
in our `config/sandbox_jumunjin.yaml`. Used as primary sand for the
Korean agricultural-robotics framing under Pivot 5.

### Senatore & Iagnemma 2014

> *Analysis of stress distributions under lightweight wheeled vehicles.*
> Journal of Terramechanics 51:1–17.

MIT mechatronics terramechanics work — bridge between empirical Bekker
fits and continuum stress modelling. Comparator for our rover_wheel
sinkage-vs-Bekker-baseline reporting.

---

## Reproducibility / pre-registration

### Nosek et al. 2015

> *Promoting an open research culture.*
> Science 348(6242):1422–1425. DOI: 10.1126/science.aab2374.

Foundational pre-registration / reproducibility argument. We cite this
in paper §2 as the methodological precedent for our Appendix B
pre-registered validation protocol — the first such instance in MPM-rigid
coupling to our knowledge.

---

## Hyperspectral imaging + smart farm (user expertise overlap)

User has expertise here per memory. Phase 5 (application) crossover:
hyperspectral overlay on soil simulation for plant-stress prediction.
Specific references TBD when Phase 5 starts.

---

## Tooling

- Dear ImGui (docking branch): https://github.com/ocornut/imgui
- GLFW: https://www.glfw.org/
- nlohmann/json: https://github.com/nlohmann/json
- TinyXML2: https://github.com/leethomason/tinyxml2
- pybind11 (Phase 4): https://github.com/pybind/pybind11
